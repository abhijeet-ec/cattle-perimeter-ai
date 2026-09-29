#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>

#include <adv_cattle_detection_inferencing.h>

// =====================================================
// WiFi
// =====================================================

const char* ssid = "Airtel_adit_3940";
const char* password = "air91177";

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
// LED
// =====================================================

#define LED_PIN 13
#define CATTLE_THRESHOLD 0.50f

// =====================================================
// Variables used by app_httpd.cpp
// =====================================================

volatile bool personDetected = false;
volatile float personConfidence = 0.0f;

volatile uint16_t personX = 0;
volatile uint16_t personY = 0;
volatile uint16_t personW = 0;
volatile uint16_t personH = 0;

// =====================================================
// AI IMAGE BUFFER
// =====================================================

static uint8_t* snapshot_buf = nullptr;

// =====================================================
// Camera
// =====================================================

static bool camera_init()
{
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

    config.pixel_format = PIXFORMAT_JPEG;

    // Keep camera at QVGA.
    // FOMO input is 96x96.
    config.frame_size = FRAMESIZE_QVGA;

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
            "Camera init failed: 0x%x\n",
            err
        );
        return false;
    }

    sensor_t* sensor = esp_camera_sensor_get();

    if (sensor) {
        sensor->set_vflip(sensor, 0);
        sensor->set_hmirror(sensor, 0);
    }

    return true;
}

// =====================================================
// Edge Impulse image callback
// =====================================================

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

// =====================================================
// Capture + resize using Edge Impulse helper
// =====================================================

static bool capture_image()
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

    // Convert JPEG to RGB888
    bool converted = fmt2rgb888(
        fb->buf,
        fb->len,
        fb->format,
        snapshot_buf
    );

    esp_camera_fb_return(fb);

    if (!converted) {
        Serial.println("RGB conversion failed");
        return false;
    }

    Serial.println("AI: RGB conversion OK");

    return true;
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("================================");
    Serial.println(" ESP32-CAM CATTLE DETECTION");
    Serial.println("================================");

    // LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // PSRAM
    Serial.printf(
        "PSRAM: %u bytes\n",
        ESP.getPsramSize()
    );

    Serial.printf(
        "Free PSRAM: %u bytes\n",
        ESP.getFreePsram()
    );

    // Allocate image buffer
    snapshot_buf = (uint8_t*)ps_malloc(
        EI_CLASSIFIER_INPUT_WIDTH *
        EI_CLASSIFIER_INPUT_HEIGHT *
        3
    );

    if (!snapshot_buf) {
        Serial.println(
            "ERROR: Cannot allocate AI buffer"
        );

        while (true) {
            delay(1000);
        }
    }

    Serial.println("AI buffer allocated");

    // Camera
    Serial.println("Initializing camera...");

    if (!camera_init()) {

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

    // WiFi
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

    // Start camera web server
    startCameraServer();

    Serial.println();
    Serial.print("Camera Ready! Open: http://");
    Serial.println(WiFi.localIP());

    Serial.println(
        "Cattle FOMO inference running."
    );
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
    Serial.println();
    Serial.println("========== CATTLE FOMO START ==========");

    // Reset detection
    personDetected = false;
    personConfidence = 0.0f;

    personX = 0;
    personY = 0;
    personW = 0;
    personH = 0;

    digitalWrite(LED_PIN, LOW);

    // -------------------------------------------------
    // Total latency
    // -------------------------------------------------

    unsigned long totalStart = millis();

    // -------------------------------------------------
    // Capture
    // -------------------------------------------------

    unsigned long captureStart = millis();

    if (!capture_image()) {
        delay(1000);
        return;
    }

    unsigned long captureTime =
        millis() - captureStart;

    // -------------------------------------------------
    // Edge Impulse signal
    // -------------------------------------------------

    signal_t signal;

    signal.total_length =
        EI_CLASSIFIER_INPUT_WIDTH *
        EI_CLASSIFIER_INPUT_HEIGHT;

    signal.get_data = get_signal_data;

    // -------------------------------------------------
    // FOMO
    // -------------------------------------------------

    Serial.println("AI: Running FOMO...");

    unsigned long inferenceStart = millis();

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

        delay(1000);
        return;
    }

    Serial.println(
        "AI: FOMO completed successfully"
    );

    // -------------------------------------------------
    // Search for cattle
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

    for (size_t i = 0;
         i < EI_CLASSIFIER_OBJECT_DETECTION_COUNT;
         i++) {

        auto bb = result.bounding_boxes[i];

        if (bb.value == 0) {
            continue;
        }

        Serial.printf(
            "%s: %.2f  x:%d y:%d w:%d h:%d\n",
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

                bestConfidence = bb.value;

                bestX = bb.x;
                bestY = bb.y;
                bestW = bb.width;
                bestH = bb.height;

                cattleFound = true;
            }
        }
    }

    // -------------------------------------------------
    // RESULT
    // -------------------------------------------------

    if (cattleFound) {

        personDetected = true;
        personConfidence = bestConfidence;

        personX = bestX;
        personY = bestY;
        personW = bestW;
        personH = bestH;

        digitalWrite(
            LED_PIN,
            HIGH
        );

        Serial.println();
        Serial.println("🐄 CATTLE DETECTED!");

        Serial.printf(
            "Confidence: %.2f%%\n",
            bestConfidence * 100.0f
        );

    } else {

        digitalWrite(
            LED_PIN,
            LOW
        );

        Serial.println();
        Serial.println("No cattle detected.");
    }

    // -------------------------------------------------
    // LATENCY
    // -------------------------------------------------

    unsigned long totalTime =
        millis() - totalStart;

    Serial.println();
    Serial.println("---------- LATENCY ----------");

    Serial.printf(
        "Capture:       %lu ms\n",
        captureTime
    );

    Serial.printf(
        "FOMO inference: %lu ms\n",
        inferenceTime
    );

    Serial.printf(
        "Total:         %lu ms\n",
        totalTime
    );

    Serial.printf(
        "EI DSP:        %d ms\n",
        result.timing.dsp
    );

    Serial.printf(
        "EI Classify:   %d ms\n",
        result.timing.classification
    );

    Serial.printf(
        "Free PSRAM:    %u bytes\n",
        ESP.getFreePsram()
    );

    Serial.println(
        "============================="
    );

    delay(500);
}