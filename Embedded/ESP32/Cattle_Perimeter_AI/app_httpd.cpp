#include "Arduino.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "esp32-hal-ledc.h"
#include "sdkconfig.h"
#include "camera_index.h"
#include "board_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

extern volatile bool personDetected;
extern volatile float personConfidence;
extern volatile uint16_t personX;
extern volatile uint16_t personY;
extern volatile uint16_t personW;
extern volatile uint16_t personH;
extern SemaphoreHandle_t cameraMutex;

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY =
    "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n"
    "X-Timestamp: %d.%06d\r\n\r\n";

static esp_err_t detection_handler(httpd_req_t *req)
{
    char response[256];

    snprintf(response, sizeof(response),
        "{\"detected\":%s,\"confidence\":%.3f,\"x\":%u,\"y\":%u,\"width\":%u,\"height\":%u}",
        personDetected ? "true" : "false",
        personConfidence,
        personX, personY, personW, personH);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, response, strlen(response));
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;

    if (cameraMutex == NULL ||
        xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    fb = esp_camera_fb_get();

    if (!fb) {
        xSemaphoreGive(cameraMutex);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t res = httpd_resp_send(
        req, (const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    xSemaphoreGive(cameraMutex);

    return res;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    struct timeval timestamp;
    esp_err_t res = ESP_OK;

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "15");

    while (true) {
        camera_fb_t *fb = NULL;
        uint8_t *jpg_buf = NULL;
        size_t jpg_len = 0;
        bool copied = false;

        // IMPORTANT:
        // Hold the mutex only while accessing the camera.
        // Never hold it while sending data over Wi-Fi.
        if (cameraMutex == NULL ||
            xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
            log_e("Camera mutex timeout");
            res = ESP_FAIL;
            break;
        }

        fb = esp_camera_fb_get();

        if (!fb) {
            xSemaphoreGive(cameraMutex);
            log_e("Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        timestamp.tv_sec = fb->timestamp.tv_sec;
        timestamp.tv_usec = fb->timestamp.tv_usec;

        // Copy JPEG out of the camera framebuffer before returning it.
        jpg_len = fb->len;
        jpg_buf = (uint8_t *)ps_malloc(jpg_len);

        if (jpg_buf != NULL) {
            memcpy(jpg_buf, fb->buf, jpg_len);
            copied = true;
        }

        esp_camera_fb_return(fb);
        fb = NULL;

        xSemaphoreGive(cameraMutex);

        if (!copied) {
            log_e("JPEG copy allocation failed");
            res = ESP_FAIL;
            break;
        }

        char part_buf[128];

        size_t hlen = snprintf(
            part_buf, sizeof(part_buf), _STREAM_PART,
            (unsigned int)jpg_len,
            (int)timestamp.tv_sec,
            (int)timestamp.tv_usec);

        if (res == ESP_OK)
            res = httpd_resp_send_chunk(
                req, _STREAM_BOUNDARY,
                strlen(_STREAM_BOUNDARY));

        if (res == ESP_OK)
            res = httpd_resp_send_chunk(
                req, part_buf, hlen);

        if (res == ESP_OK)
            res = httpd_resp_send_chunk(
                req, (const char *)jpg_buf, jpg_len);

        free(jpg_buf);
        jpg_buf = NULL;

        if (res != ESP_OK) {
            log_e("Send frame failed");
            break;
        }

        // Small yield so the FOMO loop gets CPU/camera opportunities.
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return res;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    sensor_t *s = esp_camera_sensor_get();
    char json[512];

    if (!s) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    snprintf(json, sizeof(json),
        "{\"xclk\":%u,\"pixformat\":%u,\"framesize\":%u,\"quality\":%u,"
        "\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,"
        "\"hmirror\":%u,\"vflip\":%u}",
        s->xclk_freq_hz / 1000000,
        s->pixformat,
        s->status.framesize,
        s->status.quality,
        s->status.brightness,
        s->status.contrast,
        s->status.saturation,
        s->status.hmirror,
        s->status.vflip);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

    sensor_t *s = esp_camera_sensor_get();

    if (!s) {
        return httpd_resp_send_500(req);
    }

    if (s->id.PID == OV3660_PID) {
        return httpd_resp_send(req,
            (const char *)index_ov3660_html_gz,
            index_ov3660_html_gz_len);
    }

    if (s->id.PID == OV5640_PID) {
        return httpd_resp_send(req,
            (const char *)index_ov5640_html_gz,
            index_ov5640_html_gz_len);
    }

    return httpd_resp_send(req,
        (const char *)index_ov2640_html_gz,
        index_ov2640_html_gz_len);
}

void startCameraServer()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL
    };

    httpd_uri_t detection_uri = {
        .uri = "/detection",
        .method = HTTP_GET,
        .handler = detection_handler,
        .user_ctx = NULL
    };

    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = NULL
    };

    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
        .user_ctx = NULL
    };

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL
    };

    log_i("Starting web server on port: '%u'", config.server_port);

    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &detection_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
    }

    config.server_port += 1;
    config.ctrl_port += 1;

    log_i("Starting stream server on port: '%u'", config.server_port);

    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}
