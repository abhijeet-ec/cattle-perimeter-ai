#include <Arduino.h>
#include <WiFi.h>
#include <string.h>
#include <math.h>

#include "esp_camera.h"
#include "img_converters.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Edge Impulse
#include <adv_cattle_detection_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

// =====================================================
// WiFi
// =====================================================

const char* ssid = "Airtel_adit_3940";
const char* password = "air91177";

// =====================================================
// AI-THINKER ESP32-CAM
// ===================a==================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0

#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// =====================================================
// Detection
// =====================================================

#define LED_PIN 13
#define CATTLE_THRESHOLD 0.90f

// =====================================================
// Movement Tracking
// =====================================================

// Minimum movement in pixels before considering movement.
// This prevents small FOMO bounding-box jitter from
// being interpreted as real movement.
#define MOVEMENT_THRESHOLD 8.0f

// Number of consecutive frames required to confirm
// a movement direction.
#define REQUIRED_MOVEMENT_FRAMES 2

// Model input dimensions are normally 160x160.
// Used for zone calculation.
#define TRACKING_FRAME_WIDTH 160

// =====================================================
// AI-THINKER ESP32-CAM
// =====================================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0

#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// =====================================================
// Detection
// =====================================================

#define LED_PIN 13
#define CATTLE_THRESHOLD 0.90f

#define TRACKING_FRAME_HEIGHT 160

enum MovementDirection {
    MOVEMENT_STATIONARY,
    MOVEMENT_LEFT,
    MOVEMENT_RIGHT,
    MOVEMENT_UP,
    MOVEMENT_DOWN
};

MovementDirection movementDirection =
    MOVEMENT_STATIONARY;

float previousCenterX = -1.0f;
float previousCenterY = -1.0f;

uint8_t stableMovementFrames = 0;

String currentZone = "UNKNOWN";
String entryZone = "UNKNOWN";

// =====================================================
// Camera raw frame
// =====================================================

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240

static bool is_initialised = false;
static uint8_t* snapshot_buf = nullptr;

// =====================================================
// Shared by FOMO and app_httpd.cpp
// =====================================================

SemaphoreHandle_t cameraMutex = NULL;

// =====================================================
// Variables required by app_httpd.cpp
// =====================================================

volatile bool personDetected = false;
volatile float personConfidence = 0.0f;

volatile uint16_t personX = 0;
volatile uint16_t personY = 0;
volatile uint16_t personW = 0;
volatile uint16_t personH = 0;

// =====================================================
// Movement variables available to other source files
// =====================================================

volatile int movementDirectionCode = 0;
// 0 = stationary
// 1 = left
// 2 = right
// 3 = up
// 4 = down

// =====================================================
// Camera WebServer
// =====================================================

void startCameraServer();

// =====================================================
// Movement helper functions
// =====================================================

const char* getMovementDirectionName(
    MovementDirection direction
)
{
    switch (direction) {

        case MOVEMENT_LEFT:
            return "LEFT";

        case MOVEMENT_RIGHT:
            return "RIGHT";

        case MOVEMENT_UP:
            return "UP";

        case MOVEMENT_DOWN:
            return "DOWN";

        default:
            return "STATIONARY";
    }
}

// -----------------------------------------------------
// Determine horizontal zone
// -----------------------------------------------------

String getZone(float centerX)
{
    if (centerX < TRACKING_FRAME_WIDTH / 3.0f) {
        return "LEFT";
    }

    if (centerX >
        (TRACKING_FRAME_WIDTH * 2.0f / 3.0f)) {
        return "RIGHT";
    }

    return "CENTER";
}

// -----------------------------------------------------
// Reset movement tracking
// -----------------------------------------------------

void resetMovementTracking()
{
    previousCenterX = -1.0f;
    previousCenterY = -1.0f;

    stableMovementFrames = 0;

    movementDirection =
        MOVEMENT_STATIONARY;

    movementDirectionCode = 0;

    currentZone = "UNKNOWN";
    entryZone = "UNKNOWN";
}

// -----------------------------------------------------
// Update movement tracking
// -----------------------------------------------------

void updateMovementTracking(
    float currentCenterX,
    float currentCenterY
)
{
    currentZone =
        getZone(currentCenterX);

    // First detection establishes the reference.
    if (previousCenterX < 0.0f) {

        previousCenterX =
            currentCenterX;

        previousCenterY =
            currentCenterY;

        entryZone =
            currentZone;

        movementDirection =
            MOVEMENT_STATIONARY;

        movementDirectionCode = 0;

        return;
    }

    float deltaX =
        currentCenterX -
        previousCenterX;

    float deltaY =
        currentCenterY -
        previousCenterY;

    float absX =
        fabs(deltaX);

    float absY =
        fabs(deltaY);

    // Ignore small detection jitter.
    if (
        absX < MOVEMENT_THRESHOLD &&
        absY < MOVEMENT_THRESHOLD
    ) {

        stableMovementFrames = 0;

        movementDirection =
            MOVEMENT_STATIONARY;

        movementDirectionCode = 0;
    }

    else {

        MovementDirection candidate;

        // Select the dominant direction.
        if (absX >= absY) {

            if (deltaX > 0) {
                candidate =
                    MOVEMENT_RIGHT;
            }
            else {
                candidate =
                    MOVEMENT_LEFT;
            }
        }

        else {

            if (deltaY > 0) {
                candidate =
                    MOVEMENT_DOWN;
            }
            else {
                candidate =
                    MOVEMENT_UP;
            }
        }

        // Confirm direction over consecutive frames.
        if (candidate == movementDirection) {
            stableMovementFrames++;
        }
        else {
            stableMovementFrames = 1;
        }

        if (
            stableMovementFrames >=
            REQUIRED_MOVEMENT_FRAMES
        ) {

            movementDirection =
                candidate;

            switch (movementDirection) {

                case MOVEMENT_LEFT:
                    movementDirectionCode = 1;
                    break;

                case MOVEMENT_RIGHT:
                    movementDirectionCode = 2;
                    break;

                case MOVEMENT_UP:
                    movementDirectionCode = 3;
                    break;

                case MOVEMENT_DOWN:
                    movementDirectionCode = 4;
                    break;

                default:
                    movementDirectionCode = 0;
                    break;
            }
        }
    }

    previousCenterX =
        currentCenterX;

    previousCenterY =
        currentCenterY;
}

// =====================================================
// Camera initialization
// =====================================================

bool ei_camera_init(void)
{
    if (is_initialised) {
        return true;
    }

    camera_config_t config = {};

    config.ledc_channel =
        LEDC_CHANNEL_0;

    config.ledc_timer =
        LEDC_TIMER_0;

    config.pin_d0 =
        Y2_GPIO_NUM;

    config.pin_d1 =
        Y3_GPIO_NUM;

    config.pin_d2 =
        Y4_GPIO_NUM;

    config.pin_d3 =
        Y5_GPIO_NUM;

    config.pin_d4 =
        Y6_GPIO_NUM;

    config.pin_d5 =
        Y7_GPIO_NUM;

    config.pin_d6 =
        Y8_GPIO_NUM;

    config.pin_d7 =
        Y9_GPIO_NUM;

    config.pin_xclk =
        XCLK_GPIO_NUM;

    config.pin_pclk =
        PCLK_GPIO_NUM;

    config.pin_vsync =
        VSYNC_GPIO_NUM;

    config.pin_href =
        HREF_GPIO_NUM;

    config.pin_sccb_sda =
        SIOD_GPIO_NUM;

    config.pin_sccb_scl =
        SIOC_GPIO_NUM;

    config.pin_pwdn =
        PWDN_GPIO_NUM;

    config.pin_reset =
        RESET_GPIO_NUM;

    config.xclk_freq_hz =
        20000000;

    config.frame_size =
        FRAMESIZE_QVGA;

    config.pixel_format =
        PIXFORMAT_JPEG;

    config.jpeg_quality =
        psramFound() ? 10 : 12;

    // Two frame buffers allow the camera stream
    // and FOMO inference to operate together.
    config.fb_count = 2;

    config.grab_mode =
        CAMERA_GRAB_LATEST;

    Serial.println(
        "Calling esp_camera_init..."
    );

    esp_err_t err =
        esp_camera_init(&config);

    if (err != ESP_OK) {

        Serial.printf(
            "Camera initialization failed: 0x%x\n",
            err
        );

        return false;
    }

    sensor_t* sensor =
        esp_camera_sensor_get();

    if (sensor != nullptr) {

        sensor->set_vflip(
            sensor,
            0
        );

        sensor->set_hmirror(
            sensor,
            0
        );
    }

    is_initialised = true;

    Serial.println(
        "Camera initialization successful"
    );

    return true;
}

// =====================================================
// Camera capture
// =====================================================

bool ei_camera_capture(
    uint32_t img_width,
    uint32_t img_height
)
{
    if (!is_initialised) {

        Serial.println(
            "ERROR: Camera is not initialized"
        );

        return false;
    }

    if (
        cameraMutex == NULL ||
        xSemaphoreTake(
            cameraMutex,
            pdMS_TO_TICKS(1500)
        ) != pdTRUE
    ) {

        Serial.println(
            "ERROR: Camera mutex timeout"
        );

        return false;
    }

    camera_fb_t* fb =
        esp_camera_fb_get();

    if (!fb) {

        Serial.println(
            "ERROR: Camera capture failed"
        );

        xSemaphoreGive(
            cameraMutex
        );

        return false;
    }

    Serial.printf(
        "AI: Frame %dx%d, %u bytes\n",
        fb->width,
        fb->height,
        fb->len
    );

    bool converted =
        fmt2rgb888(
            fb->buf,
            fb->len,
            PIXFORMAT_JPEG,
            snapshot_buf
        );

    esp_camera_fb_return(fb);

    if (!converted) {

        Serial.println(
            "ERROR: RGB conversion failed"
        );

        xSemaphoreGive(
            cameraMutex
        );

        return false;
    }

    // Camera framebuffer is no longer needed.
    xSemaphoreGive(
        cameraMutex
    );

    Serial.println(
        "AI: RGB conversion OK"
    );

    if (
        img_width !=
            EI_CAMERA_RAW_FRAME_BUFFER_COLS ||
        img_height !=
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS
    ) {

        Serial.printf(
            "AI: Resizing to %ux%u...\n",
            img_width,
            img_height
        );

        ei::image::processing::
            crop_and_interpolate_rgb888(
                snapshot_buf,
                EI_CAMERA_RAW_FRAME_BUFFER_COLS,
                EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
                snapshot_buf,
                img_width,
                img_height
            );

        Serial.println(
            "AI: Resize OK"
        );
    }

    return true;
}

// =====================================================
// Edge Impulse signal callback
// =====================================================

static int ei_camera_get_data(
    size_t offset,
    size_t length,
    float* out_ptr
)
{
    size_t pixel_ix =
        offset * 3;

    for (
        size_t i = 0;
        i < length;
        i++
    ) {

        out_ptr[i] =
            (
                (snapshot_buf[pixel_ix + 2]
                    << 16) |
                (snapshot_buf[pixel_ix + 1]
                    << 8) |
                snapshot_buf[pixel_ix]
            );

        pixel_ix += 3;
    }

    return 0;
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    delay(2000);

    Serial.println();
    Serial.println(
        "================================"
    );
    Serial.println(
        " ESP32-CAM ANIMAL INTRUSION"
    );
    Serial.println(
        " DIRECTIONAL MOVEMENT TRACKING"
    );
    Serial.println(
        "================================"
    );

    // -------------------------------------------------
    // LED
    // -------------------------------------------------

    pinMode(
        LED_PIN,
        OUTPUT
    );

    digitalWrite(
        LED_PIN,
        LOW
    );

    // -------------------------------------------------
    // PSRAM
    // -------------------------------------------------

    Serial.printf(
        "PSRAM: %u bytes\n",
        ESP.getPsramSize()
    );

    Serial.printf(
        "Free PSRAM: %u bytes\n",
        ESP.getFreePsram()
    );

    // -------------------------------------------------
    // Camera
    // -------------------------------------------------

    Serial.println(
        "Initializing camera..."
    );

    if (!ei_camera_init()) {

        Serial.println(
            "Camera initialization FAILED"
        );

        while (true) {
            delay(1000);
        }
    }

    Serial.println(
        "Camera initialized successfully"
    );

    // -------------------------------------------------
    // RGB buffer
    // -------------------------------------------------

    const size_t snapshot_size =
        EI_CAMERA_RAW_FRAME_BUFFER_COLS *
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
        3;

    Serial.printf(
        "Allocating RGB buffer: %u bytes\n",
        (unsigned int)snapshot_size
    );

    snapshot_buf =
        (uint8_t*)ps_malloc(
            snapshot_size
        );

    if (!snapshot_buf) {

        Serial.println(
            "ERROR: Cannot allocate RGB buffer"
        );

        while (true) {
            delay(1000);
        }
    }

    Serial.println(
        "RGB buffer allocated successfully"
    );

    Serial.printf(
        "Free PSRAM after buffer: %u bytes\n",
        ESP.getFreePsram()
    );

    // -------------------------------------------------
    // Camera mutex
    // -------------------------------------------------

    cameraMutex =
        xSemaphoreCreateMutex();

    if (cameraMutex == NULL) {

        Serial.println(
            "ERROR: Failed to create camera mutex"
        );

        while (true) {
            delay(1000);
        }
    }

    Serial.println(
        "Camera mutex created"
    );

    // -------------------------------------------------
    // WiFi
    // -------------------------------------------------

    Serial.println(
        "Connecting to WiFi..."
    );

    WiFi.begin(
        ssid,
        password
    );

    while (
        WiFi.status() != WL_CONNECTED
    ) {

        delay(500);
        Serial.print(".");
    }

    Serial.println();

    Serial.println(
        "WiFi connected"
    );

    Serial.print(
        "IP: "
    );

    Serial.println(
        WiFi.localIP()
    );

    // -------------------------------------------------
    // Camera WebServer
    // -------------------------------------------------

    startCameraServer();

    delay(1000);

    Serial.println();

    Serial.print(
        "Camera Ready! Open: http://"
    );

    Serial.println(
        WiFi.localIP()
    );

    Serial.println();

    Serial.println(
        "Animal FOMO inference running."
    );

    Serial.println(
        "Directional movement tracking enabled."
    );
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
    Serial.println();
    Serial.println(
        "========== FOMO START =========="
    );

    // -------------------------------------------------
    // Reset current detection
    // -------------------------------------------------

    personDetected = false;

    personConfidence = 0.0f;

    personX = 0;
    personY = 0;
    personW = 0;
    personH = 0;

    digitalWrite(
        LED_PIN,
        LOW
    );

    unsigned long totalStart =
        millis();

    // -------------------------------------------------
    // Capture + preprocessing
    // -------------------------------------------------

    unsigned long captureStart =
        millis();

    bool captureOK =
        ei_camera_capture(
            EI_CLASSIFIER_INPUT_WIDTH,
            EI_CLASSIFIER_INPUT_HEIGHT
        );

    unsigned long captureTime =
        millis() - captureStart;

    if (!captureOK) {

        Serial.println(
            "ERROR: Capture/preprocessing failed"
        );

        delay(500);

        return;
    }

    Serial.printf(
        "Capture + preprocessing time: %lu ms\n",
        captureTime
    );

    // -------------------------------------------------
    // Edge Impulse signal
    // -------------------------------------------------

    signal_t signal;

    signal.total_length =
        EI_CLASSIFIER_INPUT_WIDTH *
        EI_CLASSIFIER_INPUT_HEIGHT;

    signal.get_data =
        &ei_camera_get_data;

    // -------------------------------------------------
    // FOMO inference
    // -------------------------------------------------

    Serial.println(
        "AI: Running FOMO..."
    );

    unsigned long inferenceStart =
        millis();

    ei_impulse_result_t result = {};

    EI_IMPULSE_ERROR err =
        run_classifier(
            &signal,
            &result,
            false
        );

    unsigned long inferenceTime =
        millis() - inferenceStart;

    if (err != EI_IMPULSE_OK) {

        Serial.printf(
            "FOMO ERROR: %d\n",
            err
        );

        delay(500);

        return;
    }

    Serial.println(
        "AI: FOMO completed successfully"
    );

    Serial.printf(
        "DSP: %d ms\n",
        result.timing.dsp
    );

    Serial.printf(
        "Classification: %d ms\n",
        result.timing.classification
    );

    Serial.printf(
        "Measured inference time: %lu ms\n",
        inferenceTime
    );

    // -------------------------------------------------
    // Find highest-confidence animal
    // -------------------------------------------------

    bool cattleFound = false;

    float bestConfidence = 0.0f;

    uint16_t bestX = 0;
    uint16_t bestY = 0;
    uint16_t bestW = 0;
    uint16_t bestH = 0;

    Serial.println(
        "Object detection bounding boxes:"
    );

    for (
        size_t i = 0;
        i < EI_CLASSIFIER_OBJECT_DETECTION_COUNT;
        i++
    ) {

        auto bb =
            result.bounding_boxes[i];

        if (bb.value == 0) {
            continue;
        }

        Serial.printf(
            "%s: %.2f x:%d y:%d w:%d h:%d\n",
            bb.label,
            bb.value,
            bb.x,
            bb.y,
            bb.width,
            bb.height
        );

        if (
            strcmp(
                bb.label,
                "cattle"
            ) == 0 &&
            bb.value >=
                CATTLE_THRESHOLD &&
            bb.value >
                bestConfidence
        ) {

            bestConfidence =
                bb.value;

            bestX = bb.x;
            bestY = bb.y;
            bestW = bb.width;
            bestH = bb.height;

            cattleFound = true;
        }
    }

    // -------------------------------------------------
    // Detection result
    // -------------------------------------------------

    if (cattleFound) {

        personDetected = true;

        personConfidence =
            bestConfidence;

        personX = bestX;
        personY = bestY;
        personW = bestW;
        personH = bestH;

        digitalWrite(
            LED_PIN,
            HIGH
        );

        // -------------------------------------------------
        // Bounding-box center
        // -------------------------------------------------

        float currentCenterX =
            bestX +
            (bestW / 2.0f);

        float currentCenterY =
            bestY +
            (bestH / 2.0f);

        // -------------------------------------------------
        // Direction tracking
        // -------------------------------------------------

        updateMovementTracking(
            currentCenterX,
            currentCenterY
        );

        Serial.println();
        Serial.println(
            "========== ANIMAL DETECTED =========="
        );

        Serial.printf(
            "Confidence: %.2f%%\n",
            bestConfidence * 100.0f
        );

        Serial.printf(
            "Box: x=%u y=%u w=%u h=%u\n",
            bestX,
            bestY,
            bestW,
            bestH
        );

        Serial.printf(
            "Center: X=%.1f Y=%.1f\n",
            currentCenterX,
            currentCenterY
        );

        Serial.printf(
            "Movement: %s\n",
            getMovementDirectionName(
                movementDirection
            )
        );

        Serial.printf(
            "Current Zone: %s\n",
            currentZone.c_str()
        );

        Serial.printf(
            "Entry Zone: %s\n",
            entryZone.c_str()
        );

        if (
            movementDirection ==
                MOVEMENT_RIGHT
        ) {

            Serial.println(
                "DIRECTION: --> RIGHT"
            );
        }

        else if (
            movementDirection ==
                MOVEMENT_LEFT
        ) {

            Serial.println(
                "DIRECTION: <-- LEFT"
            );
        }

        else if (
            movementDirection ==
                MOVEMENT_UP
        ) {

            Serial.println(
                "DIRECTION: ^ UP"
            );
        }

        else if (
            movementDirection ==
                MOVEMENT_DOWN
        ) {

            Serial.println(
                "DIRECTION: v DOWN"
            );
        }

        else {

            Serial.println(
                "DIRECTION: STATIONARY"
            );
        }

        Serial.println(
            "======================================"
        );
    }

    else {

        digitalWrite(
            LED_PIN,
            LOW
        );

        // No animal currently visible.
        // Reset tracking so the next animal
        // starts with a fresh reference point.
        resetMovementTracking();

        Serial.println();
        Serial.println(
            "No animal detected."
        );
    }

    // -------------------------------------------------
    // Latency
    // -------------------------------------------------

    unsigned long totalTime =
        millis() - totalStart;

    Serial.println();

    Serial.printf(
        "Capture + preprocessing: %lu ms\n",
        captureTime
    );

    Serial.printf(
        "FOMO inference: %lu ms\n",
        inferenceTime
    );

    Serial.printf(
        "TOTAL DETECTION LATENCY: %lu ms\n",
        totalTime
    );

    Serial.printf(
        "Free PSRAM: %u bytes\n",
        ESP.getFreePsram()
    );

    Serial.println(
        "========== FOMO END =========="
    );

    delay(500);
}