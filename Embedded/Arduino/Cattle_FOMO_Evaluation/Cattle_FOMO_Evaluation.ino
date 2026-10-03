#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <string.h>

#include "img_converters.h"
#include <adv_cattle_detection_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

// ============================================================
// Wi-Fi
// ============================================================

const char* ssid = "Airtel_adit_3940";
const char* password = "air91177";

// ============================================================
// Evaluation server
// ============================================================

WiFiServer evalServer(8081);

// ============================================================
// Image/model dimensions
// ============================================================

#define CAMERA_WIDTH  320
#define CAMERA_HEIGHT 240

#define MODEL_WIDTH  EI_CLASSIFIER_INPUT_WIDTH
#define MODEL_HEIGHT EI_CLASSIFIER_INPUT_HEIGHT

#define RGB_BUFFER_SIZE   (CAMERA_WIDTH * CAMERA_HEIGHT * 3)
#define MODEL_BUFFER_SIZE (MODEL_WIDTH * MODEL_HEIGHT * 3)

// Prepared JPEGs should be small.
// 300 KB is already more than enough for 320x240 JPEG.
#define MAX_JPEG_SIZE (300 * 1024)

#define CATTLE_THRESHOLD 0.50f

// ============================================================
// PSRAM buffers
// ============================================================

static uint8_t* rgb_buffer = nullptr;
static uint8_t* model_buffer = nullptr;
static uint8_t* jpeg_buffer = nullptr;

// ============================================================
// Edge Impulse signal callback
// Reads the actual 96x96 model buffer.
// ============================================================

static int get_signal_data(
    size_t offset,
    size_t length,
    float* out_ptr
) {
    size_t pixel_ix = offset * 3;

    for (size_t i = 0; i < length; i++) {

        uint8_t r = model_buffer[pixel_ix + 0];
        uint8_t g = model_buffer[pixel_ix + 1];
        uint8_t b = model_buffer[pixel_ix + 2];

        // Edge Impulse image signal is packed RGB888.
        out_ptr[i] =
            (float)(((uint32_t)r << 16) |
                    ((uint32_t)g << 8) |
                    b);

        pixel_ix += 3;
    }

    return 0;
}

// ============================================================
// Receive exactly N bytes
// ============================================================

bool receiveBytes(
    WiFiClient& client,
    uint8_t* buffer,
    size_t length
) {
    size_t received = 0;
    unsigned long lastData = millis();

    while (received < length) {

        if (!client.connected()) {
            return false;
        }

        if (client.available()) {

            int n = client.read(
                buffer + received,
                length - received
            );

            if (n > 0) {
                received += n;
                lastData = millis();
            }
        }

        // 30 seconds without receiving data = failure.
        if (millis() - lastData > 30000) {
            return false;
        }

        delay(1);
    }

    return true;
}

// ============================================================
// Read 4-byte big-endian image size
// ============================================================

uint32_t readImageSize(WiFiClient& client)
{
    uint8_t header[4];

    if (!receiveBytes(client, header, 4)) {
        return 0;
    }

    return
        ((uint32_t)header[0] << 24) |
        ((uint32_t)header[1] << 16) |
        ((uint32_t)header[2] << 8)  |
        ((uint32_t)header[3]);
}

// ============================================================
// Process one JPEG
// ============================================================

bool processImage(
    uint8_t* jpeg,
    size_t jpeg_size,
    String& prediction,
    float& confidence,
    uint32_t& inference_ms,
    uint32_t& total_ms
) {
    unsigned long total_start = millis();

    prediction = "none";
    confidence = 0.0f;
    inference_ms = 0;
    total_ms = 0;

    // --------------------------------------------------------
    // JPEG -> RGB888
    // --------------------------------------------------------

    unsigned long decode_start = millis();

    bool decoded = fmt2rgb888(
        jpeg,
        jpeg_size,
        PIXFORMAT_JPEG,
        rgb_buffer
    );

    if (!decoded) {
        Serial.println("ERROR: JPEG decode failed");
        return false;
    }

    unsigned long decode_ms =
        millis() - decode_start;

    Serial.print("JPEG decode: ");
    Serial.print(decode_ms);
    Serial.println(" ms");

    // --------------------------------------------------------
    // 320x240 RGB -> 96x96 RGB
    // --------------------------------------------------------

    unsigned long resize_start = millis();

    int resize_result =
        ei::image::processing::crop_and_interpolate_rgb888(
            rgb_buffer,
            CAMERA_WIDTH,
            CAMERA_HEIGHT,
            model_buffer,
            MODEL_WIDTH,
            MODEL_HEIGHT
        );

    unsigned long resize_ms =
        millis() - resize_start;

    if (resize_result != 0) {
        Serial.print("ERROR: Resize failed, code=");
        Serial.println(resize_result);
        return false;
    }

    Serial.print("Resize: ");
    Serial.print(resize_ms);
    Serial.println(" ms");

    // --------------------------------------------------------
    // Edge Impulse signal
    // --------------------------------------------------------

    signal_t signal;

    signal.total_length =
        MODEL_WIDTH * MODEL_HEIGHT;

    signal.get_data =
        &get_signal_data;

    // --------------------------------------------------------
    // FOMO inference
    // --------------------------------------------------------

    ei_impulse_result_t result = {};

    unsigned long inference_start = millis();

    EI_IMPULSE_ERROR err =
        run_classifier(
            &signal,
            &result,
            false
        );

    inference_ms =
        millis() - inference_start;

    if (err != EI_IMPULSE_OK) {

        Serial.print(
            "ERROR: run_classifier failed: "
        );

        Serial.println((int)err);

        return false;
    }

    // --------------------------------------------------------
    // Find strongest cattle detection
    // --------------------------------------------------------

    float strongest = 0.0f;

    for (
        size_t i = 0;
        i < EI_CLASSIFIER_OBJECT_DETECTION_COUNT;
        i++
    ) {
        auto& bb = result.bounding_boxes[i];

        if (bb.value == 0) {
            continue;
        }

        Serial.print("Detection: ");
        Serial.print(bb.label);
        Serial.print(" = ");
        Serial.println(bb.value, 3);

        if (
            strcmp(bb.label, "cattle") == 0 &&
            bb.value >= CATTLE_THRESHOLD &&
            bb.value > strongest
        ) {
            strongest = bb.value;
        }
    }

    if (strongest >= CATTLE_THRESHOLD) {
        prediction = "cattle";
        confidence = strongest;
    }
    else {
        prediction = "none";
        confidence = 0.0f;
    }

    total_ms =
        millis() - total_start;

    Serial.println("--------------------------------");
    Serial.print("Prediction: ");
    Serial.println(prediction);

    Serial.print("Confidence: ");
    Serial.println(confidence, 4);

    Serial.print("Inference latency: ");
    Serial.print(inference_ms);
    Serial.println(" ms");

    Serial.print("Total latency: ");
    Serial.print(total_ms);
    Serial.println(" ms");

    Serial.println("--------------------------------");

    return true;
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" CATTLE FOMO ESP32 EVALUATION");
    Serial.println("========================================");

    // --------------------------------------------------------
    // PSRAM
    // --------------------------------------------------------

    if (!psramFound()) {

        Serial.println("ERROR: PSRAM NOT FOUND");

        while (true) {
            delay(1000);
        }
    }

    Serial.println("PSRAM detected");

    Serial.print("PSRAM size: ");
    Serial.println(ESP.getPsramSize());

    // --------------------------------------------------------
    // Allocate PSRAM
    // --------------------------------------------------------

    rgb_buffer =
        (uint8_t*)ps_malloc(RGB_BUFFER_SIZE);

    model_buffer =
        (uint8_t*)ps_malloc(MODEL_BUFFER_SIZE);

    jpeg_buffer =
        (uint8_t*)ps_malloc(MAX_JPEG_SIZE);

    if (
        rgb_buffer == nullptr ||
        model_buffer == nullptr ||
        jpeg_buffer == nullptr
    ) {
        Serial.println(
            "ERROR: PSRAM allocation failed"
        );

        while (true) {
            delay(1000);
        }
    }

    Serial.println(
        "PSRAM buffers allocated"
    );

    Serial.print("RGB buffer: ");
    Serial.println(RGB_BUFFER_SIZE);

    Serial.print("Model buffer: ");
    Serial.println(MODEL_BUFFER_SIZE);

    Serial.print("JPEG buffer: ");
    Serial.println(MAX_JPEG_SIZE);

    // --------------------------------------------------------
    // Wi-Fi
    // --------------------------------------------------------

    Serial.println();
    Serial.print("Connecting to Wi-Fi");

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {

        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("Wi-Fi connected");

    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());

    // --------------------------------------------------------
    // TCP server
    // --------------------------------------------------------

    evalServer.begin();

    Serial.println();
    Serial.println("Evaluation server started");
    Serial.println("TCP port: 8081");

    Serial.println();
    Serial.println("Waiting for laptop...");
    Serial.println("========================================");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    WiFiClient client =
        evalServer.available();

    if (!client) {
        delay(10);
        return;
    }

    Serial.println();
    Serial.println("Laptop connected");

    while (client.connected()) {

        // ----------------------------------------------------
        // Read JPEG size
        // ----------------------------------------------------

        uint32_t jpeg_size =
            readImageSize(client);

        if (jpeg_size == 0) {
            break;
        }

        Serial.print(
            "Incoming JPEG size: "
        );

        Serial.print(jpeg_size);
        Serial.println(" bytes");

        if (jpeg_size > MAX_JPEG_SIZE) {

            Serial.println(
                "ERROR: JPEG too large"
            );

            client.println(
                "error,0,0,0"
            );

            continue;
        }

        // ----------------------------------------------------
        // Receive JPEG
        // ----------------------------------------------------

        if (!receiveBytes(
                client,
                jpeg_buffer,
                jpeg_size
            )) {

            Serial.println(
                "ERROR: JPEG receive failed"
            );

            client.println(
                "error,0,0,0"
            );

            break;
        }

        // ----------------------------------------------------
        // Run model
        // ----------------------------------------------------

        String prediction;
        float confidence;

        uint32_t inference_ms;
        uint32_t total_ms;

        bool success =
            processImage(
                jpeg_buffer,
                jpeg_size,
                prediction,
                confidence,
                inference_ms,
                total_ms
            );

        if (!success) {

            client.println(
                "error,0,0,0"
            );

            continue;
        }

        // ----------------------------------------------------
        // Send result
        // ----------------------------------------------------

        client.print(prediction);
        client.print(",");
        client.print(confidence, 4);
        client.print(",");
        client.print(inference_ms);
        client.print(",");
        client.println(total_ms);
    }

    client.stop();

    Serial.println(
        "Laptop disconnected"
    );
}