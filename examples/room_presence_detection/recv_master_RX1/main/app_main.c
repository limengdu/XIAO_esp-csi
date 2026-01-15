/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file app_main.c
 * @brief CSI Master Receiver for Room Presence Detection
 *
 * This firmware runs on the master receiver device. It:
 * - Receives ESP-NOW packets and extracts CSI data (Link 1)
 * - Receives detection results from slave nodes (Link 2, 3)
 * - Fuses multi-link results using voting mechanism
 * - Creates WiFi AP hotspot for web access
 * - Provides HTTP server with WebSocket for real-time status
 * - Shows status via LED
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "esp_event.h"

#include "led_strip.h"
#include "esp_radar.h"

static const char *TAG = "recv_master";

/* Embedded web files */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
extern const uint8_t style_css_start[] asm("_binary_style_css_start");
extern const uint8_t style_css_end[] asm("_binary_style_css_end");
extern const uint8_t app_js_start[] asm("_binary_app_js_start");
extern const uint8_t app_js_end[] asm("_binary_app_js_end");

/* Configuration */
#define CONFIG_WIFI_CHANNEL             11
#define CONFIG_AP_SSID                  "RoomSensor"
#define CONFIG_AP_PASSWORD              "12345678"
#define CONFIG_AP_MAX_CONN              4
#define RADAR_BUFF_MAX_LEN              25
#define MAX_SLAVE_NODES                 2
#define LINK_TIMEOUT_MS                 3000  /* Consider link dead after 3s */

/* Sender's MAC address - used for CSI filtering */
static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};

/* LED GPIO */
#if CONFIG_IDF_TARGET_ESP32C5
#define WS2812_GPIO 27
#elif CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
#define WS2812_GPIO 8
#elif CONFIG_IDF_TARGET_ESP32S3
#define WS2812_GPIO 38
#elif CONFIG_IDF_TARGET_ESP32C3
#define WS2812_GPIO 8
#else
#define WS2812_GPIO 4
#endif

static led_strip_handle_t g_led_strip = NULL;
static httpd_handle_t g_httpd = NULL;
static SemaphoreHandle_t g_state_mutex = NULL;

/* Per-link status and sensitivity */
typedef struct {
    bool active;
    bool room_status;       /* Recalculated by master based on sensitivity */
    bool human_status;      /* Recalculated by master based on sensitivity */
    float wander;           /* Raw value from sensor */
    float jitter;           /* Raw value from sensor */
    int8_t rssi;
    uint32_t last_update;
    
    /* Per-link sensitivity (independently adjustable) */
    float wander_sensitivity;
    float jitter_sensitivity;
} link_status_t;

/* Global detection state */
typedef struct {
    /* Local detection buffers (Link 0) */
    float wander_buff[RADAR_BUFF_MAX_LEN];
    float jitter_buff[RADAR_BUFF_MAX_LEN];
    uint32_t buff_count;
    
    /* Global thresholds (from calibration) */
    float wander_threshold;
    float jitter_threshold;
    
    /* Per-link status */
    link_status_t links[3];  /* 0=local, 1=slave1, 2=slave2 */
    
    /* Fused result */
    bool room_status;        /* >=2 links detect presence/motion -> has person */
    bool human_status;       /* >=2 links detect motion -> moving; else stationary */
    
    /* Calibration */
    bool calibrating;
    uint32_t calibration_start_time;
    uint32_t calibration_duration_ms;
} master_state_t;

static master_state_t g_state = {
    .buff_count = 0,
    .wander_threshold = 0.0f,
    .jitter_threshold = 0.0003f,
    .room_status = false,
    .human_status = false,
    .calibrating = false,
    .calibration_start_time = 0,
    .calibration_duration_ms = 30000,
    .links = {
        { .wander_sensitivity = 0.15f, .jitter_sensitivity = 0.20f },  /* Link 0 (local) */
        { .wander_sensitivity = 0.15f, .jitter_sensitivity = 0.20f },  /* Link 1 (slave1) */
        { .wander_sensitivity = 0.15f, .jitter_sensitivity = 0.20f },  /* Link 2 (slave2) */
    },
};

/* ESP-NOW message from slave */
typedef struct __attribute__((packed)) {
    uint8_t msg_type;
    uint8_t node_id;
    uint8_t room_status;
    uint8_t human_status;
    float wander;
    float jitter;
    int8_t rssi;
    uint32_t timestamp;
} slave_report_t;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define ESP_IF_WIFI_STA ESP_MAC_WIFI_STA
#endif

/* Utility functions */
static float trimmean(const float *array, size_t len, float percent)
{
    if (len == 0) return 0.0f;
    float *sorted = malloc(len * sizeof(float));
    if (!sorted) return 0.0f;
    memcpy(sorted, array, len * sizeof(float));
    
    for (size_t i = 0; i < len - 1; i++) {
        for (size_t j = 0; j < len - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    
    size_t trim = (size_t)(len * percent / 2);
    float sum = 0;
    size_t count = 0;
    for (size_t i = trim; i < len - trim; i++) {
        sum += sorted[i];
        count++;
    }
    free(sorted);
    return count > 0 ? sum / count : 0.0f;
}

static float median(const float *array, size_t len)
{
    if (len == 0) return 0.0f;
    float *sorted = malloc(len * sizeof(float));
    if (!sorted) return 0.0f;
    memcpy(sorted, array, len * sizeof(float));
    
    for (size_t i = 0; i < len - 1; i++) {
        for (size_t j = 0; j < len - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    
    float result = (len % 2 == 0) ? 
        (sorted[len/2 - 1] + sorted[len/2]) / 2.0f : sorted[len/2];
    free(sorted);
    return result;
}

/* NVS Storage - persist calibration and sensitivity across power cycles */
#define NVS_NAMESPACE "presence"

static void nvs_save_settings(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return;
    }
    
    /* Save thresholds */
    nvs_set_blob(handle, "wander_th", &g_state.wander_threshold, sizeof(float));
    nvs_set_blob(handle, "jitter_th", &g_state.jitter_threshold, sizeof(float));
    
    /* Save sensitivity for all links */
    nvs_set_blob(handle, "link0_w_sens", &g_state.links[0].wander_sensitivity, sizeof(float));
    nvs_set_blob(handle, "link0_j_sens", &g_state.links[0].jitter_sensitivity, sizeof(float));
    nvs_set_blob(handle, "link1_w_sens", &g_state.links[1].wander_sensitivity, sizeof(float));
    nvs_set_blob(handle, "link1_j_sens", &g_state.links[1].jitter_sensitivity, sizeof(float));
    nvs_set_blob(handle, "link2_w_sens", &g_state.links[2].wander_sensitivity, sizeof(float));
    nvs_set_blob(handle, "link2_j_sens", &g_state.links[2].jitter_sensitivity, sizeof(float));
    
    nvs_commit(handle);
    nvs_close(handle);
    
    ESP_LOGI(TAG, "Settings saved to NVS");
}

static void nvs_load_settings(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No saved settings found, using defaults");
        return;
    }
    
    size_t len = sizeof(float);
    
    /* Load thresholds */
    nvs_get_blob(handle, "wander_th", &g_state.wander_threshold, &len);
    len = sizeof(float);
    nvs_get_blob(handle, "jitter_th", &g_state.jitter_threshold, &len);
    
    /* Load sensitivity for all links */
    len = sizeof(float);
    nvs_get_blob(handle, "link0_w_sens", &g_state.links[0].wander_sensitivity, &len);
    len = sizeof(float);
    nvs_get_blob(handle, "link0_j_sens", &g_state.links[0].jitter_sensitivity, &len);
    len = sizeof(float);
    nvs_get_blob(handle, "link1_w_sens", &g_state.links[1].wander_sensitivity, &len);
    len = sizeof(float);
    nvs_get_blob(handle, "link1_j_sens", &g_state.links[1].jitter_sensitivity, &len);
    len = sizeof(float);
    nvs_get_blob(handle, "link2_w_sens", &g_state.links[2].wander_sensitivity, &len);
    len = sizeof(float);
    nvs_get_blob(handle, "link2_j_sens", &g_state.links[2].jitter_sensitivity, &len);
    
    nvs_close(handle);
    
    ESP_LOGI(TAG, "Settings loaded from NVS: wander_th=%.6f, jitter_th=%.6f",
             g_state.wander_threshold, g_state.jitter_threshold);
    ESP_LOGI(TAG, "Link sensitivities: [0]%.2f/%.2f [1]%.2f/%.2f [2]%.2f/%.2f",
             g_state.links[0].wander_sensitivity, g_state.links[0].jitter_sensitivity,
             g_state.links[1].wander_sensitivity, g_state.links[1].jitter_sensitivity,
             g_state.links[2].wander_sensitivity, g_state.links[2].jitter_sensitivity);
}

/* LED functions */
static esp_err_t led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags = { .with_dma = false }
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &g_led_strip));
    led_strip_clear(g_led_strip);
    return ESP_OK;
}

static void led_update(void)
{
    if (!g_led_strip) return;
    
    if (g_state.calibrating) {
        static bool blink = false;
        led_strip_set_pixel(g_led_strip, 0, blink ? 255 : 0, blink ? 255 : 0, 0);
        blink = !blink;
    } else if (g_state.room_status) {
        if (g_state.human_status) {
            led_strip_set_pixel(g_led_strip, 0, 0, 255, 0);  /* Green = moving */
        } else {
            led_strip_set_pixel(g_led_strip, 0, 255, 255, 255);  /* White = present */
        }
    } else {
        led_strip_set_pixel(g_led_strip, 0, 0, 0, 0);  /* Off = empty */
    }
    led_strip_refresh(g_led_strip);
}

/**
 * @brief Recalculate detection status for a single link based on its sensitivity
 * 
 * Detection logic (same as esp-radar):
 *   value * sensitivity > threshold  →  detection triggered
 * 
 * Lower sensitivity = harder to trigger (need bigger signal change)
 * Higher sensitivity = easier to trigger (more sensitive to small changes)
 */
static void recalculate_link_status(int link_idx)
{
    link_status_t *link = &g_state.links[link_idx];
    
    if (!link->active) return;
    
    /* Presence: wander * sensitivity > threshold */
    if (g_state.wander_threshold > 0) {
        link->room_status = (link->wander * link->wander_sensitivity > g_state.wander_threshold);
    } else {
        link->room_status = false;  /* No calibration done yet */
    }
    
    /* Motion: jitter * sensitivity > threshold */
    if (g_state.jitter_threshold > 0) {
        link->human_status = (link->jitter * link->jitter_sensitivity > g_state.jitter_threshold);
    } else {
        link->human_status = false;  /* No calibration done yet */
    }
}

/**
 * @brief Fuse multi-link detection results using voting
 * 
 * Logic:
 * - Link 0 (local): use sensitivity settings on master
 * - Link 1, 2 (slaves): use their own detection results (they have their own calibration)
 * - >=2 links detect (presence OR motion) -> room has person
 * - >=2 links detect motion -> person is moving
 */
static void fuse_detection_results(void)
{
    uint32_t now = esp_log_timestamp();
    int detection_count = 0;  /* Links detecting anything (presence or motion) */
    int motion_count = 0;     /* Links detecting motion specifically */
    int active_count = 0;
    
    for (int i = 0; i < 3; i++) {
        /* Check if link is still active */
        if (g_state.links[i].active && 
            (now - g_state.links[i].last_update) < LINK_TIMEOUT_MS) {
            
            active_count++;
            
            /* Only recalculate for local link (0), slaves send their own detection */
            if (i == 0) {
                recalculate_link_status(i);
            }
            /* Slaves (i=1,2) already have room_status/human_status set from their report */
            
            /* Count detections */
            if (g_state.links[i].room_status || g_state.links[i].human_status) {
                detection_count++;
            }
            if (g_state.links[i].human_status) {
                motion_count++;
            }
        } else {
            g_state.links[i].active = false;
        }
    }
    
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    
    /* Adaptive threshold based on active links */
    int min_detection = (active_count >= 2) ? 2 : 1;
    
    /* Room has person: >=min_detection links detect something */
    g_state.room_status = (detection_count >= min_detection);
    
    /* Person moving: >=min_detection links detect motion */
    g_state.human_status = g_state.room_status && (motion_count >= min_detection);
    
    xSemaphoreGive(g_state_mutex);
    
    led_update();
}

/**
 * @brief WiFi radar callback for local CSI processing (Link 1)
 */
static void wifi_radar_cb(void *ctx, const wifi_radar_info_t *info)
{
    uint32_t buff_min_size = 5;
    
    g_state.wander_buff[g_state.buff_count % RADAR_BUFF_MAX_LEN] = info->waveform_wander;
    g_state.jitter_buff[g_state.buff_count % RADAR_BUFF_MAX_LEN] = info->waveform_jitter;
    g_state.buff_count++;
    
    if (g_state.buff_count < buff_min_size) return;
    
    /* Calculate smoothed values */
    float wander_avg = trimmean(g_state.wander_buff, RADAR_BUFF_MAX_LEN, 0.5f);
    float jitter_med = median(g_state.jitter_buff, RADAR_BUFF_MAX_LEN);
    
    /* Update Link 0 (local) - store raw values only */
    /* Status will be calculated by recalculate_link_status() based on per-link sensitivity */
    g_state.links[0].active = true;
    g_state.links[0].wander = wander_avg;
    g_state.links[0].jitter = jitter_med;
    g_state.links[0].last_update = esp_log_timestamp();
    
    /* Fuse all links (this calls recalculate_link_status for each) */
    fuse_detection_results();
}

/**
 * @brief ESP-NOW receive callback - handle reports from slaves
 */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (len < sizeof(slave_report_t)) return;
    
    slave_report_t *report = (slave_report_t *)data;
    
    if (report->msg_type == 0x01 && report->node_id >= 1 && report->node_id <= MAX_SLAVE_NODES) {
        int link_idx = report->node_id;  /* 1 or 2 */
        
        /* Use slave's own detection results - they have their own calibrated thresholds */
        /* This is more accurate because each device has different signal characteristics */
        g_state.links[link_idx].active = true;
        g_state.links[link_idx].room_status = report->room_status;    /* Trust slave's detection */
        g_state.links[link_idx].human_status = report->human_status;  /* Trust slave's detection */
        g_state.links[link_idx].wander = report->wander;
        g_state.links[link_idx].jitter = report->jitter;
        g_state.links[link_idx].rssi = report->rssi;
        g_state.links[link_idx].last_update = esp_log_timestamp();
        
        ESP_LOGD(TAG, "Slave %d: room=%d, move=%d, wander=%.6f, jitter=%.6f",
                 report->node_id, report->room_status, report->human_status,
                 report->wander, report->jitter);
        
        fuse_detection_results();
    }
}

/**
 * @brief Broadcast calibration command to slaves
 */
static void broadcast_calibration_cmd(uint8_t cmd)
{
    uint8_t broadcast_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    esp_now_send(broadcast_addr, &cmd, 1);
}

/* HTTP Handlers */
static esp_err_t http_get_index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, 
                    index_html_end - index_html_start);
    return ESP_OK;
}

static esp_err_t http_get_style(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/css");
    httpd_resp_send(req, (const char *)style_css_start,
                    style_css_end - style_css_start);
    return ESP_OK;
}

static esp_err_t http_get_script(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_send(req, (const char *)app_js_start,
                    app_js_end - app_js_start);
    return ESP_OK;
}

static esp_err_t http_get_status(httpd_req_t *req)
{
    char buf[512];
    
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    int len = snprintf(buf, sizeof(buf),
        "{\"room\":%d,\"moving\":%d,\"calibrating\":%d,"
        "\"links\":[{\"active\":%d,\"room\":%d,\"move\":%d,\"wander\":%.6f,\"jitter\":%.6f},"
        "{\"active\":%d,\"room\":%d,\"move\":%d,\"wander\":%.6f,\"jitter\":%.6f},"
        "{\"active\":%d,\"room\":%d,\"move\":%d,\"wander\":%.6f,\"jitter\":%.6f}]}",
        g_state.room_status ? 1 : 0,
        g_state.human_status ? 1 : 0,
        g_state.calibrating ? 1 : 0,
        g_state.links[0].active ? 1 : 0, g_state.links[0].room_status ? 1 : 0,
        g_state.links[0].human_status ? 1 : 0, g_state.links[0].wander, g_state.links[0].jitter,
        g_state.links[1].active ? 1 : 0, g_state.links[1].room_status ? 1 : 0,
        g_state.links[1].human_status ? 1 : 0, g_state.links[1].wander, g_state.links[1].jitter,
        g_state.links[2].active ? 1 : 0, g_state.links[2].room_status ? 1 : 0,
        g_state.links[2].human_status ? 1 : 0, g_state.links[2].wander, g_state.links[2].jitter);
    xSemaphoreGive(g_state_mutex);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

/**
 * @brief Stop calibration and get thresholds
 */
static void finish_calibration(void)
{
    if (!g_state.calibrating) return;
    
    ESP_LOGI(TAG, "Stopping calibration...");
    esp_radar_train_stop(&g_state.wander_threshold, &g_state.jitter_threshold);
    g_state.calibrating = false;
    broadcast_calibration_cmd(0x11);
    
    ESP_LOGI(TAG, "Calibration done: wander_th=%.6f, jitter_th=%.6f",
             g_state.wander_threshold, g_state.jitter_threshold);
    
    /* Save to NVS */
    nvs_save_settings();
}

static esp_err_t http_post_calibrate(httpd_req_t *req)
{
    char buf[64];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    if (strstr(buf, "start")) {
        ESP_LOGI(TAG, "Starting calibration (30 seconds)...");
        g_state.calibrating = true;
        g_state.calibration_start_time = esp_log_timestamp();
        esp_radar_train_start();
        broadcast_calibration_cmd(0x10);
        httpd_resp_sendstr(req, "{\"status\":\"calibrating\",\"duration\":30}");
    } else if (strstr(buf, "stop")) {
        finish_calibration();
        
        char resp[128];
        snprintf(resp, sizeof(resp), 
                 "{\"status\":\"done\",\"wander_th\":%.6f,\"jitter_th\":%.6f}",
                 g_state.wander_threshold, g_state.jitter_threshold);
        httpd_resp_sendstr(req, resp);
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid action");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * @brief API to get/set per-link sensitivity parameters
 * GET: returns all links' sensitivity
 * POST: {"link":0, "wander_sens":0.15, "jitter_sens":0.20}
 */
static esp_err_t http_post_sensitivity(httpd_req_t *req)
{
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    
    httpd_resp_set_type(req, "application/json");
    
    if (ret <= 0) {
        /* GET - return all links' sensitivity and thresholds */
        char resp[512];
        snprintf(resp, sizeof(resp),
            "{\"wander_th\":%.6f,\"jitter_th\":%.6f,"
            "\"links\":["
            "{\"wander_sens\":%.3f,\"jitter_sens\":%.3f},"
            "{\"wander_sens\":%.3f,\"jitter_sens\":%.3f},"
            "{\"wander_sens\":%.3f,\"jitter_sens\":%.3f}]}",
            g_state.wander_threshold, g_state.jitter_threshold,
            g_state.links[0].wander_sensitivity, g_state.links[0].jitter_sensitivity,
            g_state.links[1].wander_sensitivity, g_state.links[1].jitter_sensitivity,
            g_state.links[2].wander_sensitivity, g_state.links[2].jitter_sensitivity);
        httpd_resp_sendstr(req, resp);
        return ESP_OK;
    }
    buf[ret] = '\0';
    
    /* Parse JSON: {"link":0, "wander_sens":0.15, "jitter_sens":0.20} */
    int link_idx = -1;
    float wander_sens = -1;
    float jitter_sens = -1;
    
    /* Simple JSON parsing */
    char *p = strstr(buf, "\"link\"");
    if (p) {
        p = strchr(p, ':');
        if (p) {
            p++;
            while (*p == ' ') p++;
            link_idx = atoi(p);
        }
    }
    
    p = strstr(buf, "wander_sens");
    if (p) {
        p = strchr(p, ':');
        if (p) {
            p++;
            while (*p == ' ' || *p == '"') p++;
            wander_sens = strtof(p, NULL);
        }
    }
    
    p = strstr(buf, "jitter_sens");
    if (p) {
        p = strchr(p, ':');
        if (p) {
            p++;
            while (*p == ' ' || *p == '"') p++;
            jitter_sens = strtof(p, NULL);
        }
    }
    
    /* Validate and update */
    if (link_idx < 0 || link_idx > 2) {
        httpd_resp_sendstr(req, "{\"error\":\"Invalid link index (0-2)\"}");
        return ESP_OK;
    }
    
    /* Update per-link sensitivity (store locally for display) */
    /* Allow very low values (0.001) for fine-tuning false positives */
    if (wander_sens >= 0.001f && wander_sens <= 5.0f) {
        g_state.links[link_idx].wander_sensitivity = wander_sens;
    }
    if (jitter_sens >= 0.001f && jitter_sens <= 5.0f) {
        g_state.links[link_idx].jitter_sensitivity = jitter_sens;
    }
    
    /* For slaves (link 1, 2), send sensitivity command via ESP-NOW */
    if (link_idx > 0) {
        uint8_t cmd_buf[10];
        cmd_buf[0] = 0x13;  /* Set sensitivity command */
        cmd_buf[1] = (uint8_t)link_idx;  /* Target node ID */
        memcpy(&cmd_buf[2], &g_state.links[link_idx].wander_sensitivity, 4);
        memcpy(&cmd_buf[6], &g_state.links[link_idx].jitter_sensitivity, 4);
        
        uint8_t broadcast_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
        esp_err_t err = esp_now_send(broadcast_addr, cmd_buf, sizeof(cmd_buf));
        
        ESP_LOGI(TAG, "Sent sensitivity to slave %d: wander=%.3f, jitter=%.3f (err=%d)",
                 link_idx, 
                 g_state.links[link_idx].wander_sensitivity,
                 g_state.links[link_idx].jitter_sensitivity,
                 err);
    } else {
        ESP_LOGI(TAG, "Master (Link 0) sensitivity updated: wander=%.3f, jitter=%.3f",
                 g_state.links[0].wander_sensitivity,
                 g_state.links[0].jitter_sensitivity);
    }
    
    /* Save to NVS */
    nvs_save_settings();
    
    /* Return updated values for this link */
    char resp[128];
    snprintf(resp, sizeof(resp),
             "{\"link\":%d,\"wander_sens\":%.3f,\"jitter_sens\":%.3f}",
             link_idx,
             g_state.links[link_idx].wander_sensitivity,
             g_state.links[link_idx].jitter_sensitivity);
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

/* WebSocket client management */
#define MAX_WS_CLIENTS 4
static int g_ws_clients[MAX_WS_CLIENTS] = {-1, -1, -1, -1};
static SemaphoreHandle_t g_ws_mutex = NULL;

static void ws_add_client(int fd)
{
    xSemaphoreTake(g_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (g_ws_clients[i] < 0) {
            g_ws_clients[i] = fd;
            ESP_LOGI(TAG, "WebSocket client added: fd=%d, slot=%d", fd, i);
            break;
        }
    }
    xSemaphoreGive(g_ws_mutex);
}

static void ws_remove_client(int fd)
{
    xSemaphoreTake(g_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (g_ws_clients[i] == fd) {
            g_ws_clients[i] = -1;
            ESP_LOGI(TAG, "WebSocket client removed: fd=%d", fd);
            break;
        }
    }
    xSemaphoreGive(g_ws_mutex);
}

/* Async send work structure */
typedef struct {
    httpd_handle_t hd;
    int fd;
    char data[512];
    size_t len;
} ws_async_arg_t;

static void ws_async_send(void *arg)
{
    ws_async_arg_t *a = (ws_async_arg_t *)arg;
    
    httpd_ws_frame_t ws_pkt = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)a->data,
        .len = a->len,
        .final = true,
    };
    
    esp_err_t ret = httpd_ws_send_frame_async(a->hd, a->fd, &ws_pkt);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "WS send failed fd=%d: %s", a->fd, esp_err_to_name(ret));
        ws_remove_client(a->fd);
    }
    
    free(arg);
}

static void ws_broadcast(const char *data, size_t len)
{
    if (!g_httpd || !g_ws_mutex) return;
    
    xSemaphoreTake(g_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        int fd = g_ws_clients[i];
        if (fd >= 0) {
            ws_async_arg_t *arg = malloc(sizeof(ws_async_arg_t));
            if (arg) {
                arg->hd = g_httpd;
                arg->fd = fd;
                memcpy(arg->data, data, len);
                arg->len = len;
                
                if (httpd_queue_work(g_httpd, ws_async_send, arg) != ESP_OK) {
                    free(arg);
                }
            }
        }
    }
    xSemaphoreGive(g_ws_mutex);
}

/* WebSocket handler */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ws_add_client(fd);
        ESP_LOGI(TAG, "WebSocket handshake, fd=%d", fd);
        return ESP_OK;
    }
    
    /* Handle incoming WebSocket messages */
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(ws_pkt));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        int fd = httpd_req_to_sockfd(req);
        ws_remove_client(fd);
        return ret;
    }
    
    return ESP_OK;
}

/**
 * @brief WebSocket status broadcast task - runs at 4Hz
 */
static void ws_broadcast_task(void *arg)
{
    char buf[768];
    ESP_LOGI(TAG, "WebSocket broadcast task started");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(250));  /* 4Hz update for smoother UI */
        
        /* Check calibration timeout - auto-stop after 30 seconds */
        if (g_state.calibrating) {
            uint32_t elapsed = esp_log_timestamp() - g_state.calibration_start_time;
            if (elapsed >= g_state.calibration_duration_ms) {
                ESP_LOGI(TAG, "Calibration auto-stopping after %lu ms", (unsigned long)elapsed);
                finish_calibration();
            }
        }
        
        /* Calculate remaining calibration time */
        int calib_remaining = 0;
        if (g_state.calibrating) {
            uint32_t elapsed = esp_log_timestamp() - g_state.calibration_start_time;
            calib_remaining = (int)(g_state.calibration_duration_ms - elapsed) / 1000;
            if (calib_remaining < 0) calib_remaining = 0;
        }
        
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        int len = snprintf(buf, sizeof(buf),
            "{\"room\":%d,\"moving\":%d,\"calibrating\":%d,\"calib_remaining\":%d,"
            "\"wander_th\":%.6f,\"jitter_th\":%.6f,"
            "\"links\":["
            "{\"active\":%d,\"room\":%d,\"move\":%d,\"wander\":%.6f,\"jitter\":%.6f,\"w_sens\":%.3f,\"j_sens\":%.3f},"
            "{\"active\":%d,\"room\":%d,\"move\":%d,\"wander\":%.6f,\"jitter\":%.6f,\"w_sens\":%.3f,\"j_sens\":%.3f},"
            "{\"active\":%d,\"room\":%d,\"move\":%d,\"wander\":%.6f,\"jitter\":%.6f,\"w_sens\":%.3f,\"j_sens\":%.3f}]}",
            g_state.room_status ? 1 : 0,
            g_state.human_status ? 1 : 0,
            g_state.calibrating ? 1 : 0,
            calib_remaining,
            g_state.wander_threshold, g_state.jitter_threshold,
            /* Link 0 */
            g_state.links[0].active ? 1 : 0, g_state.links[0].room_status ? 1 : 0,
            g_state.links[0].human_status ? 1 : 0, g_state.links[0].wander, g_state.links[0].jitter,
            g_state.links[0].wander_sensitivity, g_state.links[0].jitter_sensitivity,
            /* Link 1 */
            g_state.links[1].active ? 1 : 0, g_state.links[1].room_status ? 1 : 0,
            g_state.links[1].human_status ? 1 : 0, g_state.links[1].wander, g_state.links[1].jitter,
            g_state.links[1].wander_sensitivity, g_state.links[1].jitter_sensitivity,
            /* Link 2 */
            g_state.links[2].active ? 1 : 0, g_state.links[2].room_status ? 1 : 0,
            g_state.links[2].human_status ? 1 : 0, g_state.links[2].wander, g_state.links[2].jitter,
            g_state.links[2].wander_sensitivity, g_state.links[2].jitter_sensitivity);
        xSemaphoreGive(g_state_mutex);
        
        ws_broadcast(buf, len);
    }
}

/**
 * @brief Start HTTP server
 */
static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 10;
    config.stack_size = 8192;
    
    if (httpd_start(&g_httpd, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }
    
    /* Register URI handlers */
    httpd_uri_t uri_index = { .uri = "/", .method = HTTP_GET, .handler = http_get_index };
    httpd_uri_t uri_style = { .uri = "/style.css", .method = HTTP_GET, .handler = http_get_style };
    httpd_uri_t uri_script = { .uri = "/app.js", .method = HTTP_GET, .handler = http_get_script };
    httpd_uri_t uri_status = { .uri = "/api/status", .method = HTTP_GET, .handler = http_get_status };
    httpd_uri_t uri_calibrate = { .uri = "/api/calibrate", .method = HTTP_POST, .handler = http_post_calibrate };
    httpd_uri_t uri_sensitivity = { .uri = "/api/sensitivity", .method = HTTP_POST, .handler = http_post_sensitivity };
    httpd_uri_t uri_ws = { .uri = "/ws", .method = HTTP_GET, .handler = ws_handler, .is_websocket = true };
    
    httpd_register_uri_handler(g_httpd, &uri_index);
    httpd_register_uri_handler(g_httpd, &uri_style);
    httpd_register_uri_handler(g_httpd, &uri_script);
    httpd_register_uri_handler(g_httpd, &uri_status);
    httpd_register_uri_handler(g_httpd, &uri_calibrate);
    httpd_register_uri_handler(g_httpd, &uri_sensitivity);
    httpd_register_uri_handler(g_httpd, &uri_ws);
    
    ESP_LOGI(TAG, "HTTP server started");
    return g_httpd;
}

/**
 * @brief Initialize WiFi AP mode
 */
static void wifi_ap_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_create_default_wifi_ap();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    /* Configure AP + STA mode for both AP hotspot and CSI reception */
    wifi_config_t ap_config = {
        .ap = {
            .ssid = CONFIG_AP_SSID,
            .password = CONFIG_AP_PASSWORD,
            .ssid_len = strlen(CONFIG_AP_SSID),
            .channel = CONFIG_WIFI_CHANNEL,
            .max_connection = CONFIG_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    /* Set STA channel same as AP */
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_WIFI_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    /* Set fixed MAC for sender identification */
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "STA MAC: " MACSTR, MAC2STR(mac));
    
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);
    ESP_LOGI(TAG, "AP IP: " IPSTR, IP2STR(&ip_info.ip));
}

/**
 * @brief Initialize radar CSI reception
 */
static void radar_init(void)
{
    /* CSI configuration */
    esp_radar_csi_config_t csi_config = ESP_RADAR_CSI_CONFIG_DEFAULT();
    memcpy(csi_config.filter_mac, CONFIG_CSI_SEND_MAC, 6);
    csi_config.csi_recv_interval = 10;
    
    /* ESP-NOW configuration for receiving slave reports */
    esp_radar_espnow_config_t espnow_config = ESP_RADAR_ESPNOW_CONFIG_DEFAULT();
    
    /* Decoder configuration */
    esp_radar_dec_config_t dec_config = ESP_RADAR_DEC_CONFIG_DEFAULT();
    dec_config.wifi_radar_cb = wifi_radar_cb;
    
    /* Initialize subsystems (WiFi already initialized in AP mode) */
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_radar_csi_init(&csi_config));
    
    /* Initialize ESP-NOW */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    
    /* Add broadcast peer */
    esp_now_peer_info_t peer = {
        .channel = CONFIG_WIFI_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    };
    esp_now_add_peer(&peer);
    
    /* Register receive callback */
    esp_now_register_recv_cb(espnow_recv_cb);
    
    ESP_ERROR_CHECK(esp_radar_dec_init(&dec_config));
}

void app_main(void)
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    /* Load saved settings from NVS */
    nvs_load_settings();
    
    /* Initialize mutexes */
    g_state_mutex = xSemaphoreCreateMutex();
    g_ws_mutex = xSemaphoreCreateMutex();
    
    /* Initialize LED */
    led_init();
    
    ESP_LOGI(TAG, "================ RECV MASTER ================");
    ESP_LOGI(TAG, "AP SSID: %s, Password: %s", CONFIG_AP_SSID, CONFIG_AP_PASSWORD);
    ESP_LOGI(TAG, "Web interface: http://192.168.4.1");
    
    /* Initialize WiFi AP */
    wifi_ap_init();
    
    /* Initialize radar */
    radar_init();
    
    /* Start radar processing */
    ESP_ERROR_CHECK(esp_radar_start());
    
    /* Start HTTP server */
    start_webserver();
    
    /* Start WebSocket broadcast task */
    xTaskCreate(ws_broadcast_task, "ws_broadcast", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Master receiver started");
    ESP_LOGI(TAG, "Connect to WiFi '%s' and open http://192.168.4.1", CONFIG_AP_SSID);
    
    /* Main loop - periodic status logging */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        ESP_LOGI(TAG, "Status: Room=%d, Moving=%d, Links: [%d,%d,%d]",
                 g_state.room_status, g_state.human_status,
                 g_state.links[0].active, g_state.links[1].active, g_state.links[2].active);
    }
}
