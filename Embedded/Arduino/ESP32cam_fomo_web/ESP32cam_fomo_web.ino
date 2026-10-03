#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_system.h"
#include <WiFi.h>

#include <adv_cattle_detection_inferencing.h>

// ============================================================
// WIFI
// ============================================================

const char* ssid = "Airtel_adit_3940";
const char* password = "air91177";

// ============================================================
// AI-THINKER ESP32-CAM
// ============================================================

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

// ============================================================
// LED
// ============================================================

#define LED_PIN 13

// Detection threshold
#define CATTLE_THRESHOLD 0.50f

// ============================================================
// DETECTION VARIABLES
// These names are kept because your existing app_httpd.cpp
// expects these exact variables.
// ============================================================

volatile bool personDetected = false;
volatile float personConfidence = 0.0f;

volatile uint16_t personX = 0;
volatile uint16_t personY = 0;
volatile uint16_t personW = 0;
volatile uint16_t personH = 0;

// ============================================================
// CAMERA BUFFER
// ============================================================

static bool is_initialised = false;

static uint8_t* snapshot_buf = nullptr;

// ============================================================
// CAMERA INITIALIZATION
// ============================================================

bool ei_camera_init(void)
{
    if (is_initialised) {
        return true;
    }

    camera_config_t config;

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;

    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;

    // IMPORTANT:
    // Camera remains 320x240.
    // Edge Impulse internally converts it to 96x96.
    config.frame_size = FRAMESIZE_QVGA;

    config.pixel_format = PIXFORMAT_JPEG;

    if (psramFound()) {
        config.jpeg_quality = 10;
        config.fb_count = 1;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    } else {
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    }

    esp_err_t err = esp_camera_init(&config);

    if (err != ESP_OK) {
        Serial.printf(
            "Camera initialization failed: 0x%x\n",
            err
        );
        return false;
    }

    sensor_t* s = esp_camera_sensor_get();

    if (s != nullptr) {
        // Keep normal image orientation
        s->set_vflip(s, 0);
        s->set_hmirror(s, 0);
    }

    is_initialised = true;

    return true;
}

// ============================================================
// CAMERA DEINITIALIZATION
// ============================================================

void ei_camera_deinit(void)
{
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK) {
        Serial.println("Camera deinit failed");
    }

    is_initialised = false;
}

// ============================================================
// CAMERA CAPTURE
// ============================================================

bool ei_camera_capture(
    uint32_t img_width,
    uint32_t img_height
)
{
    bool converted = false;

    camera_fb_t* fb = esp_camera_fb_get();

    if (!fb) {
        Serial.println("Camera capture failed");
        return false;
    }

    Serial.printf(
        "Frame %dx%d, %d bytes\n",
        fb->width,
        fb->height,
        fb->len
    );

    if (fb->format == PIXFORMAT_JPEG) {

        converted = fmt2rgb888(
            fb->buf,
            fb->len,
            fb->format,
            snapshot_buf
        );

        if (!converted) {
            Serial.println("RGB conversion failed");
            esp_camera_fb_return(fb);
            return false;
        }

        Serial.println("RGB conversion OK");

    } else {

        memcpy(
            snapshot_buf,
            fb->buf,
            fb->len
        );

        converted = true;
    }

    esp_camera_fb_return(fb);

    if (!converted) {
        return false;
    }

    Serial.printf(
        "Resizing to %dx%d...\n",
        img_width,
        img_height
    );

    // Official Edge Impulse image preprocessing
    if (fb->width == img_width && fb->height == img_height) {
        memcpy(
            snapshot_buf,
            snapshot_buf,
            img_width * img_height * 3
        );
    }

    return true;
}

// ============================================================
// IMAGE DATA CALLBACK
// ============================================================

static int get_signal_data(
    size_t offset,
    size_t length,
    float* out_ptr
)
{
    size_t pixel_ix = offset * 3;

    for (size_t i = 0; i < length; i++) {

        uint8_t r = snapshot_buf[pixel_ix];
        uint8_t g = snapshot_buf[pixel_ix + 1];
        uint8_t b = snapshot_buf[pixel_ix + 2];

        out_ptr[i] =
            (r << 16) |
            (g << 8) |
            b;

        pixel_ix += 3;
    }

    return 0;
}

// ============================================================
// BETTER CAMERA PIPELINE
// ============================================================

bool capture_and_resize()
{
    camera_fb_t* fb = esp_camera_fb_get();

    if (!fb) {
        Serial.println("Camera capture failed");
        return false;
    }

    Serial.printf(
        "AI: Frame %dx%d, %d bytes\n",
        fb->width,
        fb->height,
        fb->len
    );

    // Convert JPEG -> RGB888
    if (!fmt2rgb888(
            fb->buf,
            fb->len,
            fb->format,
            snapshot_buf
        )) {

        Serial.println("AI: RGB conversion failed");

        esp_camera_fb_return(fb);
        return false;
    }

    Serial.println("AI: RGB conversion OK");

    esp_camera_fb_return(fb);

    return true;
}

// ============================================================
// START CAMERA WEB SERVER
// ============================================================

void startCameraServer();

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" ESP32-CAM + CATTLE FOMO");
    Serial.println(" AI-Thinker ESP32-CAM");
    Serial.println("========================================");

    // --------------------------------------------------------
    // LED
    // --------------------------------------------------------

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // --------------------------------------------------------
    // PSRAM
    // --------------------------------------------------------

    Serial.printf(
        "PSRAM: %u bytes\n",
        ESP.getPsramSize()
    );

    Serial.printf(
        "Free PSRAM: %u bytes\n",
        ESP.getFreePsram()
    );

    // --------------------------------------------------------
    // Allocate AI image buffer
    // 96 x 96 x RGB
    // --------------------------------------------------------

    snapshot_buf = (uint8_t*)ps_malloc(
        EI_CLASSIFIER_INPUT_WIDTH *
        EI_CLASSIFIER_INPUT_HEIGHT *
        3
    );

    if (!snapshot_buf) {

        Serial.println(
            "ERROR: Failed to allocate snapshot buffer"
        );

        while (true) {
            delay(1000);
        }
    }

    Serial.printf(
        "AI buffer allocated: %u bytes\n",
        EI_CLASSIFIER_INPUT_WIDTH *
        EI_CLASSIFIER_INPUT_HEIGHT *
        3
    );

    // --------------------------------------------------------
    // Camera
    // --------------------------------------------------------

    Serial.println("Initializing camera...");

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

    // --------------------------------------------------------
    // WiFi
    // --------------------------------------------------------

    Serial.println("Connecting to WiFi...");

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {

        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected");

    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    // --------------------------------------------------------
    // Camera web server
    // --------------------------------------------------------

    startCameraServer();

    Serial.println();
    Serial.println("========================================");
    Serial.println(" Camera Ready!");
    Serial.print(" Open: http://");
    Serial.println(WiFi.localIP());
    Serial.println("========================================");

    Serial.println(
        "Cattle FOMO inference will run continuously."
    );
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    Serial.println();
    Serial.println("========== CATTLE FOMO START ==========");

    // --------------------------------------------------------
    // Reset detection state
    // --------------------------------------------------------

    personDetected = false;
    personConfidence = 0.0f;

    personX = 0;
    personY = 0;
    personW = 0;
    personH = 0;

    digitalWrite(LED_PIN, LOW);

    // --------------------------------------------------------
    // Total latency start
    // --------------------------------------------------------

    unsigned long totalStart = millis();

    // --------------------------------------------------------
    // Camera capture
    // --------------------------------------------------------

    unsigned long captureStart = millis();

    camera_fb_t* fb = esp_camera_fb_get();

    if (!fb) {

        Serial.println("AI: Camera capture failed");

        delay(1000);
        return;
    }

    Serial.printf(
        "AI: Frame %dx%d, %d bytes\n",
        fb->width,
        fb->height,
        fb->len
    );

    unsigned long captureTime =
        millis() - captureStart;

    // --------------------------------------------------------
    // JPEG -> RGB888
    // --------------------------------------------------------

    unsigned long rgbStart = millis();

    bool rgbOK = fmt2rgb888(
        fb->buf,
        fb->len,
        fb->format,
        snapshot_buf
    );

    esp_camera_fb_return(fb);

    unsigned long rgbTime =
        millis() - rgbStart;

    if (!rgbOK) {

        Serial.println(
            "AI: RGB conversion FAILED"
        );

        delay(1000);
        return;
    }

    Serial.println("AI: RGB conversion OK");

    // --------------------------------------------------------
    // IMPORTANT
    //
    // The camera image is 320x240.
    // Resize/crop it to 96x96 for FOMO.
    // --------------------------------------------------------

    unsigned long resizeStart = millis();

    uint8_t* resizedImage =
        (uint8_t*)ps_malloc(
            EI_CLASSIFIER_INPUT_WIDTH *
            EI_CLASSIFIER_INPUT_HEIGHT *
            3
        );

    if (!resizedImage) {

        Serial.println(
            "AI: Failed to allocate resize buffer"
        );

        delay(1000);
        return;
    }

    // Crop central square from 320x240
    // 240x240 region
    uint32_t cropSize =
        min((uint32_t)320, (uint32_t)240);

    uint32_t cropX =
        (320 - cropSize) / 2;

    uint32_t cropY =
        (240 - cropSize) / 2;

    // Nearest-neighbour resize
    for (uint32_t y = 0;
         y < EI_CLASSIFIER_INPUT_HEIGHT;
         y++) {

        for (uint32_t x = 0;
             x < EI_CLASSIFIER_INPUT_WIDTH;
             x++) {

            uint32_t srcX =
                cropX +
                (x * cropSize) /
                EI_CLASSIFIER_INPUT_WIDTH;

            uint32_t srcY =
                cropY +
                (y * cropSize) /
                EI_CLASSIFIER_INPUT_HEIGHT;

            uint32_t srcIndex =
                (srcY * 320 + srcX) * 3;

            uint32_t dstIndex =
                (y *
                 EI_CLASSIFIER_INPUT_WIDTH +
                 x) * 3;

            resizedImage[dstIndex] =
                snapshot_buf[srcIndex];

            resizedImage[dstIndex + 1] =
                snapshot_buf[srcIndex + 1];

            resizedImage[dstIndex + 2] =
                snapshot_buf[srcIndex + 2];
        }
    }

    unsigned long resizeTime =
        millis() - resizeStart;

    Serial.println("AI: Resize OK");

    // --------------------------------------------------------
    // Edge Impulse signal
    // --------------------------------------------------------

    signal_t signal;

    signal.total_length =
        EI_CLASSIFIER_INPUT_WIDTH *
        EI_CLASSIFIER_INPUT_HEIGHT;

    signal.get_data =
        [](size_t offset,
           size_t length,
           float* out_ptr) -> int {

            for (size_t i = 0; i < length; i++) {

                size_t index =
                    (offset + i) * 3;

                uint8_t r =
                    snapshot_buf[index];

                uint8_t g =
                    snapshot_buf[index + 1];

                uint8_t b =
                    snapshot_buf[index + 2];

                out_ptr[i] =
                    (r << 16) |
                    (g << 8) |
                    b;
            }

            return 0;
        };

    // Copy resized image back into snapshot buffer
    memcpy(
        snapshot_buf,
        resizedImage,
        EI_CLASSIFIER_INPUT_WIDTH *
        EI_CLASSIFIER_INPUT_HEIGHT *
        3
    );

    free(resizedImage);

    // --------------------------------------------------------
    // RUN FOMO
    // --------------------------------------------------------

    Serial.println("AI: Running FOMO...");

    unsigned long inferenceStart = millis();

    ei_impulse_result_t result =
        {};

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
            "AI: FOMO ERROR: %d\n",
            err
        );

        delay(1000);
        return;
    }

    Serial.println(
        "AI: FOMO completed successfully"
    );

    // --------------------------------------------------------
    // FIND CATTLE
    // --------------------------------------------------------

    bool cattleFound = false;

    float bestConfidence = 0.0f;

    uint16_t bestX = 0;
    uint16_t bestY = 0;
    uint16_t bestW = 0;
    uint16_t bestH = 0;

    Serial.println(
        "Object detection bounding boxes:"
    );

    for (size_t i = 0;
         i < EI_CLASSIFIER_OBJECT_DETECTION_COUNT;
         i++) {

        auto bb =
            result.bounding_boxes[i];

        if (bb.value == 0) {
            continue;
        }

        Serial.printf(
            "  %s: %.2f  x:%d y:%d w:%d h:%d\n",
            bb.label,
            bb.value,
            bb.x,
            bb.y,
            bb.width,
            bb.height
        );

        if (
            strcmp(bb.label, "cattle") == 0 &&
            bb.value >= CATTLE_THRESHOLD
        ) {

            if (bb.value > bestConfidence) {

                bestConfidence =
                    bb.value;

                bestX = bb.x;
                bestY = bb.y;
                bestW = bb.width;
                bestH = bb.height;

                cattleFound = true;
            }
        }
    }

    // --------------------------------------------------------
    // UPDATE DETECTION
    // --------------------------------------------------------

    if (cattleFound) {

        personDetected = true;

        personConfidence =
            bestConfidence;

        personX = bestX;
        personY = bestY;
        personW = bestW;
        personH = bestH;

        // LED ON
        digitalWrite(
            LED_PIN,
            HIGH
        );

        Serial.println();
        Serial.println(
            "🐄 CATTLE DETECTED!"
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

    } else {

        digitalWrite(
            LED_PIN,
            LOW
        );

        Serial.println();
        Serial.println(
            "No cattle detected."
        );
    }

    // --------------------------------------------------------
    // LATENCY
    // --------------------------------------------------------

    unsigned long totalTime =
        millis() - totalStart;

    Serial.println();
    Serial.println("---------- LATENCY ----------");

    Serial.printf(
        "Camera capture: %lu ms\n",
        captureTime
    );

    Serial.printf(
        "RGB conversion: %lu ms\n",
        rgbTime
    );

    Serial.printf(
        "Resize:         %lu ms\n",
        resizeTime
    );

    Serial.printf(
        "FOMO inference: %lu ms\n",
        inferenceTime
    );

    Serial.printf(
        "TOTAL latency:  %lu ms\n",
        totalTime
    );

    Serial.printf(
        "EI DSP:         %d ms\n",
        result.timing.dsp
    );

    Serial.printf(
        "EI classification: %d ms\n",
        result.timing.classification
    );

    Serial.printf(
        "Free PSRAM:     %u bytes\n",
        ESP.getFreePsram()
    );

    Serial.println(
        "============================="
    );

    Serial.println(
        "=========== FOMO END ==========="
    );

    // --------------------------------------------------------
    // Delay before next detection
    // --------------------------------------------------------

    delay(500);
}