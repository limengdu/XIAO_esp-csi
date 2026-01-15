/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file app_main.c
 * @brief CSI Slave Receiver for Room Presence Detection
 *
 * This firmware runs on slave receiver devices. It:
 * - Receives ESP-NOW packets and extracts CSI data
 * - Calculates presence/movement indicators using esp-radar
 * - Sends detection results to the master node via ESP-NOW
 * - Shows status via LED
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"

#include "led_strip.h"
#include "esp_radar.h"

static const char *TAG = "recv_slave";

/* Configuration */
#define CONFIG_WIFI_CHANNEL             11
#define RADAR_BUFF_MAX_LEN              25

/* Sender's MAC address - used for CSI filtering */
static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};

/* Master receiver's MAC address - set during pairing or use broadcast */
static uint8_t g_master_mac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/* LED GPIO - different for different boards */
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

/* Detection state */
typedef struct {
    float wander_buff[RADAR_BUFF_MAX_LEN];
    float jitter_buff[RADAR_BUFF_MAX_LEN];
    uint32_t buff_count;
    float wander_threshold;
    float jitter_threshold;
    float wander_sensitivity;
    float jitter_sensitivity;
    bool room_status;      /* Someone present */
    bool human_status;     /* Someone moving */
    bool calibrating;
} detection_state_t;

static detection_state_t g_detect = {
    .buff_count = 0,
    .wander_threshold = 0.01f,    /* Non-zero default to avoid always-detect bug */
    .jitter_threshold = 0.001f,   /* Non-zero default */
    .wander_sensitivity = 0.15f,
    .jitter_sensitivity = 0.20f,
    .room_status = false,
    .human_status = false,
    .calibrating = false,
};

/* ESP-NOW message structure for sending to master */
typedef struct __attribute__((packed)) {
    uint8_t msg_type;      /* 0x01 = detection result */
    uint8_t node_id;       /* Slave node ID (1 or 2) */
    uint8_t room_status;   /* 1 = someone present */
    uint8_t human_status;  /* 1 = someone moving */
    float wander;          /* Raw wander value */
    float jitter;          /* Raw jitter value */
    int8_t rssi;           /* Signal strength */
    uint32_t timestamp;    /* Local timestamp */
} slave_report_t;

/* Node ID: Change this before flashing each slave!
 * RX2 (first slave)  -> node_id = 1
 * RX3 (second slave) -> node_id = 2
 */
#ifndef CONFIG_SLAVE_NODE_ID
#define CONFIG_SLAVE_NODE_ID 2
#endif
static uint8_t g_node_id = CONFIG_SLAVE_NODE_ID;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define ESP_IF_WIFI_STA ESP_MAC_WIFI_STA
#endif

/**
 * @brief Calculate trimmed mean (remove outliers)
 */
static float trimmean(const float *array, size_t len, float percent)
{
    if (len == 0) return 0.0f;
    
    /* Simple implementation: sort and trim edges */
    float *sorted = malloc(len * sizeof(float));
    if (!sorted) return 0.0f;
    
    memcpy(sorted, array, len * sizeof(float));
    
    /* Bubble sort (small array, acceptable) */
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

/**
 * @brief Calculate median
 */
static float median(const float *array, size_t len)
{
    if (len == 0) return 0.0f;
    
    float *sorted = malloc(len * sizeof(float));
    if (!sorted) return 0.0f;
    
    memcpy(sorted, array, len * sizeof(float));
    
    /* Bubble sort */
    for (size_t i = 0; i < len - 1; i++) {
        for (size_t j = 0; j < len - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    
    float result;
    if (len % 2 == 0) {
        result = (sorted[len / 2 - 1] + sorted[len / 2]) / 2.0f;
    } else {
        result = sorted[len / 2];
    }
    
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
    nvs_set_blob(handle, "wander_th", &g_detect.wander_threshold, sizeof(float));
    nvs_set_blob(handle, "jitter_th", &g_detect.jitter_threshold, sizeof(float));
    
    /* Save sensitivity */
    nvs_set_blob(handle, "wander_sens", &g_detect.wander_sensitivity, sizeof(float));
    nvs_set_blob(handle, "jitter_sens", &g_detect.jitter_sensitivity, sizeof(float));
    
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
    nvs_get_blob(handle, "wander_th", &g_detect.wander_threshold, &len);
    len = sizeof(float);
    nvs_get_blob(handle, "jitter_th", &g_detect.jitter_threshold, &len);
    
    /* Load sensitivity */
    len = sizeof(float);
    nvs_get_blob(handle, "wander_sens", &g_detect.wander_sensitivity, &len);
    len = sizeof(float);
    nvs_get_blob(handle, "jitter_sens", &g_detect.jitter_sensitivity, &len);
    
    nvs_close(handle);
    
    ESP_LOGI(TAG, "Settings loaded: wander_th=%.6f, jitter_th=%.6f, w_sens=%.2f, j_sens=%.2f",
             g_detect.wander_threshold, g_detect.jitter_threshold,
             g_detect.wander_sensitivity, g_detect.jitter_sensitivity);
}

/**
 * @brief Initialize LED strip
 */
static esp_err_t led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,
        .max_leds = 1,
    };
    
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags = {
            .with_dma = false,
        }
    };
    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &g_led_strip));
    led_strip_clear(g_led_strip);
    
    return ESP_OK;
}

/**
 * @brief Update LED based on detection status
 */
static void led_update_status(bool room_status, bool human_status, bool calibrating)
{
    if (!g_led_strip) return;
    
    if (calibrating) {
        /* Yellow blink during calibration */
        static bool blink = false;
        if (blink) {
            led_strip_set_pixel(g_led_strip, 0, 255, 255, 0);
        } else {
            led_strip_set_pixel(g_led_strip, 0, 0, 0, 0);
        }
        blink = !blink;
    } else if (room_status) {
        if (human_status) {
            /* Green = someone moving */
            led_strip_set_pixel(g_led_strip, 0, 0, 255, 0);
        } else {
            /* White = someone present but still */
            led_strip_set_pixel(g_led_strip, 0, 255, 255, 255);
        }
    } else {
        /* Off = no one */
        led_strip_set_pixel(g_led_strip, 0, 0, 0, 0);
    }
    
    led_strip_refresh(g_led_strip);
}

/**
 * @brief Send detection result to master
 */
static void send_result_to_master(float wander, float jitter, int8_t rssi)
{
    slave_report_t report = {
        .msg_type = 0x01,
        .node_id = g_node_id,
        .room_status = g_detect.room_status ? 1 : 0,
        .human_status = g_detect.human_status ? 1 : 0,
        .wander = wander,
        .jitter = jitter,
        .rssi = rssi,
        .timestamp = esp_log_timestamp(),
    };
    
    esp_err_t ret = esp_now_send(g_master_mac, (uint8_t *)&report, sizeof(report));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send report to master: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief WiFi radar callback - called when radar data is available
 */
static void wifi_radar_cb(void *ctx, const wifi_radar_info_t *info)
{
    static uint32_t s_last_send_time = 0;
    uint32_t buff_max_size = 5;
    uint32_t buff_outliers_num = 2;
    
    /* Store in circular buffer */
    g_detect.wander_buff[g_detect.buff_count % RADAR_BUFF_MAX_LEN] = info->waveform_wander;
    g_detect.jitter_buff[g_detect.buff_count % RADAR_BUFF_MAX_LEN] = info->waveform_jitter;
    g_detect.buff_count++;
    
    if (g_detect.buff_count < buff_max_size) {
        return;
    }
    
    /* Calculate smoothed values */
    float wander_average = trimmean(g_detect.wander_buff, RADAR_BUFF_MAX_LEN, 0.5f);
    float jitter_median = median(g_detect.jitter_buff, RADAR_BUFF_MAX_LEN);
    
    /* Count detections in window */
    uint32_t someone_count = 0;
    uint32_t move_count = 0;
    
    /* Only detect if threshold is positive (calibrated) */
    bool has_valid_threshold = (g_detect.wander_threshold > 0.0001f);
    
    for (uint32_t i = 0; i < buff_max_size; i++) {
        if (has_valid_threshold && 
            wander_average * g_detect.wander_sensitivity > g_detect.wander_threshold) {
            someone_count++;
        }
        
        uint32_t index = (g_detect.buff_count - 1 - i) % RADAR_BUFF_MAX_LEN;
        if (g_detect.jitter_threshold > 0.0001f &&
            (g_detect.jitter_buff[index] * g_detect.jitter_sensitivity > g_detect.jitter_threshold
            || (g_detect.jitter_buff[index] * g_detect.jitter_sensitivity > jitter_median 
                && g_detect.jitter_buff[index] > 0.0002f))) {
            move_count++;
        }
    }
    
    /* Update status */
    g_detect.room_status = has_valid_threshold && (someone_count >= 1);
    g_detect.human_status = (g_detect.jitter_threshold > 0.0001f) && (move_count >= buff_outliers_num);
    
    /* Skip processing during calibration */
    if (g_detect.calibrating) {
        led_update_status(false, false, true);
        return;
    }
    
    /* Update LED */
    led_update_status(g_detect.room_status, g_detect.human_status, false);
    
    /* Send to master at ~10Hz */
    uint32_t now = esp_log_timestamp();
    if (now - s_last_send_time >= 100) {
        send_result_to_master(wander_average, jitter_median, 0);
        s_last_send_time = now;
        
        ESP_LOGI(TAG, "Room: %d, Moving: %d, Wander: %.6f, Jitter: %.6f",
                 g_detect.room_status, g_detect.human_status, wander_average, jitter_median);
    }
}

/**
 * @brief ESP-NOW receive callback - handle commands from master
 */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (len < 1) return;
    
    /* Only process messages that start with our command prefix (0x10-0x1F reserved for commands) */
    /* This filters out CSI data packets from the TX sender */
    uint8_t cmd = data[0];
    
    /* Skip if not a command (commands are 0x10-0x1F) */
    if (cmd < 0x10 || cmd > 0x1F) {
        return;  /* Silently ignore non-command packets (likely CSI data) */
    }
    
    /* Log who sent this command */
    ESP_LOGI(TAG, "Received command 0x%02x from " MACSTR, cmd, MAC2STR(recv_info->src_addr));
    
    switch (cmd) {
        case 0x10:  /* Start calibration */
            ESP_LOGI(TAG, "Starting calibration...");
            g_detect.calibrating = true;
            esp_radar_train_start();
            break;
            
        case 0x11:  /* Stop calibration */
            ESP_LOGI(TAG, "Stopping calibration...");
            esp_radar_train_stop(&g_detect.wander_threshold, &g_detect.jitter_threshold);
            g_detect.calibrating = false;
            ESP_LOGI(TAG, "Calibration complete: wander_th=%.6f, jitter_th=%.6f",
                     g_detect.wander_threshold, g_detect.jitter_threshold);
            nvs_save_settings();  /* Save to NVS */
            break;
            
        case 0x12:  /* Set thresholds */
            if (len >= 9) {
                memcpy(&g_detect.wander_threshold, data + 1, 4);
                memcpy(&g_detect.jitter_threshold, data + 5, 4);
                ESP_LOGI(TAG, "Thresholds updated: wander=%.6f, jitter=%.6f",
                         g_detect.wander_threshold, g_detect.jitter_threshold);
            }
            break;
        
        case 0x13:  /* Set sensitivity - format: [cmd][node_id][wander_sens(4)][jitter_sens(4)] */
            if (len >= 10) {
                uint8_t target_node = data[1];
                if (target_node == g_node_id) {
                    memcpy(&g_detect.wander_sensitivity, data + 2, 4);
                    memcpy(&g_detect.jitter_sensitivity, data + 6, 4);
                    ESP_LOGI(TAG, "Sensitivity updated: wander=%.3f, jitter=%.3f",
                             g_detect.wander_sensitivity, g_detect.jitter_sensitivity);
                    nvs_save_settings();  /* Save to NVS */
                } else {
                    ESP_LOGD(TAG, "Sensitivity command for node %d, I am node %d, ignoring",
                             target_node, g_node_id);
                }
            }
            break;
            
        default:
            ESP_LOGD(TAG, "Unknown command in valid range: 0x%02x", cmd);
            break;
    }
}

/**
 * @brief Initialize WiFi and ESP-NOW
 */
static void wifi_radar_init(void)
{
    /* WiFi configuration */
    esp_radar_wifi_config_t wifi_config = ESP_RADAR_WIFI_CONFIG_DEFAULT();
    wifi_config.channel = CONFIG_WIFI_CHANNEL;
    
    /* CSI configuration */
    esp_radar_csi_config_t csi_config = ESP_RADAR_CSI_CONFIG_DEFAULT();
    memcpy(csi_config.filter_mac, CONFIG_CSI_SEND_MAC, 6);
    csi_config.csi_recv_interval = 10;  /* 100Hz */
    
    /* ESP-NOW configuration */
    esp_radar_espnow_config_t espnow_config = ESP_RADAR_ESPNOW_CONFIG_DEFAULT();
    
    /* Decoder configuration with radar callback */
    esp_radar_dec_config_t dec_config = ESP_RADAR_DEC_CONFIG_DEFAULT();
    dec_config.wifi_radar_cb = wifi_radar_cb;
    dec_config.wifi_radar_cb_ctx = NULL;
    
    /* Initialize radar subsystems */
    ESP_ERROR_CHECK(esp_radar_wifi_init(&wifi_config));
    ESP_ERROR_CHECK(esp_radar_csi_init(&csi_config));
    ESP_ERROR_CHECK(esp_radar_espnow_init(&espnow_config));
    ESP_ERROR_CHECK(esp_radar_dec_init(&dec_config));
    
    /* Add master as ESP-NOW peer for sending reports */
    esp_now_peer_info_t peer = {
        .channel = CONFIG_WIFI_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, g_master_mac, 6);
    
    esp_err_t ret = esp_now_add_peer(&peer);
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "Failed to add master peer: %s", esp_err_to_name(ret));
    }
    
    /* Register receive callback */
    esp_now_register_recv_cb(espnow_recv_cb);
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
    
    /* Load saved calibration and sensitivity settings */
    nvs_load_settings();
    
    /* Initialize LED */
    led_init();
    
    /* Get node ID from NVS or use default */
    nvs_handle_t nvs;
    if (nvs_open("config", NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u8(nvs, "node_id", &g_node_id);
        nvs_close(nvs);
    }
    
    ESP_LOGI(TAG, "================ RECV SLAVE ================");
    ESP_LOGI(TAG, "Node ID: %d", g_node_id);
    ESP_LOGI(TAG, "Sender MAC filter: " MACSTR, MAC2STR(CONFIG_CSI_SEND_MAC));
    ESP_LOGI(TAG, "Thresholds: wander=%.6f, jitter=%.6f", 
             g_detect.wander_threshold, g_detect.jitter_threshold);
    ESP_LOGI(TAG, "Sensitivity: wander=%.3f, jitter=%.3f",
             g_detect.wander_sensitivity, g_detect.jitter_sensitivity);
    
    /* Initialize WiFi radar */
    wifi_radar_init();
    
    /* Start radar processing */
    ESP_ERROR_CHECK(esp_radar_start());
    
    ESP_LOGI(TAG, "Slave receiver started, waiting for CSI data...");
    
    /* Main task just keeps running, all work is done in callbacks */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
