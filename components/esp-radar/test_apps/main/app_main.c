/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "nvs_flash.h"

#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_radar_t.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_heap_trace.h"

static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x2a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const char *TAG = "csi_recv";
static heap_trace_record_t s_trace_record[256];

static void radar_rx_cb(void *ctx, const wifi_radar_info_t *info)
{
    ESP_LOGI(TAG, "waveform_jitter %lf, waveform_wander %lf", info->waveform_jitter, info->waveform_wander);
}
static void wifi_csi_rx_cb_1(void *ctx, const wifi_csi_filtered_info_t *filtered_info)
{
    if (filtered_info->data_type == WIFI_CSI_DATA_TYPE_INT16) {
        const int16_t *csi = filtered_info->valid_data_i16;
        int sample_cnt = filtered_info->valid_len / sizeof(int16_t);
        ets_printf("CSI_DATA," MACSTR ",%d,%d,\"[%d", MAC2STR(filtered_info->mac), sample_cnt, 0, csi[0]);
        for (int k = 1; k < sample_cnt; ++k) {
            ets_printf(",%d", csi[k]);
        }
        ets_printf("]\"\n");

        const int8_t *csi1 = filtered_info->raw_data;
        ets_printf("RAW_DATA,%d,%d,\"[%d", filtered_info->raw_len, 0, (int16_t)(((int16_t)csi1[1]) << 12) >> 4 | (uint8_t)csi1[0]);
        for (int i = 2; i < filtered_info->raw_len; i += 2) {
            ets_printf(",%d", (int16_t)(((int16_t)csi1[i + 1]) << 12) >> 4 | (uint8_t)csi1[i]);
        }
        ets_printf("]\"\n");

    } else {
        const int8_t *csi = filtered_info->valid_data;
        int sample_cnt = filtered_info->valid_len;
        ESP_LOGI(TAG, "rx_gain_compensation: %lf", filtered_info->rx_gain_compensation);
        ets_printf("CSI_DATA," MACSTR ",%d,%d,\"[%d", MAC2STR(filtered_info->mac), sample_cnt, 0, csi[0]);
        for (int k = 1; k < sample_cnt; ++k) {
            ets_printf(",%d", csi[k]);
        }
        ets_printf("]\"\n");

        const int8_t *csi1 = filtered_info->raw_data;
        ets_printf("RAW_DATA,%d,%d,\"[%d", filtered_info->raw_len, 0, csi1[0]);
        for (int i = 1; i < filtered_info->raw_len; i++) {
            ets_printf(",%d", csi1[i]);
        }
        ets_printf("]\"\n");
    }
}

// 内存监控任务
static void memory_monitor_task(void *arg)
{
    const char *TAG_MEM = "memory_monitor";
    uint32_t free_heap_size = 0;
    uint32_t min_free_heap_size = 0;
    uint32_t last_free_heap_size = 0;

    uint32_t free_psram_size = 0;
    uint32_t last_free_psram_size = 0;
    uint32_t free_internal_size = 0;
    uint32_t last_free_internal_size = 0;

    ESP_LOGI(TAG_MEM, "Memory monitor task started");

    while (1) {
        free_heap_size = esp_get_free_heap_size();
        min_free_heap_size = esp_get_minimum_free_heap_size();
        free_internal_size = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        free_psram_size = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

        // 打印详细的内存信息
        ESP_LOGI(TAG_MEM, "=== Memory Status ===");
        ESP_LOGI(TAG_MEM, "Total free heap: %lu bytes (%.2f KB)",
                 free_heap_size, free_heap_size / 1024.0f);
        ESP_LOGI(TAG_MEM, "Min free heap: %lu bytes (%.2f KB)",
                 min_free_heap_size, min_free_heap_size / 1024.0f);

        // 打印内存分配详情（按能力）
        ESP_LOGI(TAG_MEM, "Internal RAM free: %lu bytes (%.2f KB)",
                 free_internal_size, free_internal_size / 1024.0f);
        ESP_LOGI(TAG_MEM, "PSRAM free: %lu bytes (%.2f KB)",
                 free_psram_size, free_psram_size / 1024.0f);
        ESP_LOGI(TAG_MEM, "DMA capable free: %lu bytes (%.2f KB)",
                 heap_caps_get_free_size(MALLOC_CAP_DMA),
                 heap_caps_get_free_size(MALLOC_CAP_DMA) / 1024.0f);

        // 打印最大可分配块
        ESP_LOGI(TAG_MEM, "Largest free block (Internal): %lu bytes",
                 heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        ESP_LOGI(TAG_MEM, "Largest free block (PSRAM): %lu bytes",
                 heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));

        // 检测总内存泄漏
        if (last_free_heap_size > 0 && free_heap_size < last_free_heap_size) {
            int32_t leaked = last_free_heap_size - free_heap_size;
            ESP_LOGW(TAG_MEM, "Total memory decreased by %ld bytes!", leaked);
        }

        // 检测 Internal RAM 泄漏
        if (last_free_internal_size > 0 && free_internal_size < last_free_internal_size) {
            int32_t leaked = last_free_internal_size - free_internal_size;
            ESP_LOGW(TAG_MEM, "Internal RAM decreased by %ld bytes!", leaked);
        }

        // 检测 PSRAM 泄漏
        if (last_free_psram_size > 0 && free_psram_size < last_free_psram_size) {
            int32_t leaked = last_free_psram_size - free_psram_size;
            ESP_LOGW(TAG_MEM, "PSRAM decreased by %ld bytes!", leaked);
        }

        last_free_heap_size = free_heap_size;
        last_free_internal_size = free_internal_size;
        last_free_psram_size = free_psram_size;

        // 每5秒打印一次
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    /**
     * @brief Initialize NVS
     */

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_radar_csi_config_t csi_config = ESP_RADAR_CSI_CONFIG_DEFAULT();
    memcpy(csi_config.filter_mac, CONFIG_CSI_SEND_MAC, 6);
    csi_config.csi_filtered_cb = wifi_csi_rx_cb_1;
    // csi_config.acquire_csi_lltf = false;
    csi_config.csi_compensate_en = false;
    // csi_config.csi_filtered_cb_ctx = CONFIG_CSI_SEND_MAC;
    // csi_config.csi_recv_interval = 50;
    esp_radar_wifi_config_t wifi_config = ESP_RADAR_WIFI_CONFIG_DEFAULT();
    esp_radar_espnow_config_t espnow_config = ESP_RADAR_ESPNOW_CONFIG_DEFAULT();
    esp_radar_config_t radar_config = ESP_RADAR_CONFIG_DEFAULT();
    radar_config.wifi_radar_cb = radar_rx_cb;
    ESP_ERROR_CHECK(esp_radar_wifi_init(&wifi_config, &espnow_config));
    ESP_ERROR_CHECK(esp_radar_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_radar_init(&radar_config));
    // esp_radar_start();
    // float wander_threshold = 0;
    // float jitter_threshold = 0;
    // vTaskDelay(pdMS_TO_TICKS(3000));
    // esp_radar_train_start();
    // vTaskDelay(pdMS_TO_TICKS(5000));
    // esp_radar_train_stop(&wander_threshold, &jitter_threshold);
    // ESP_LOGI(TAG, "wander_threshold: %f, jitter_threshold: %f", wander_threshold, jitter_threshold);
    // 启动内存监控任务
    // 初始化堆跟踪，记录泄漏

    xTaskCreate(memory_monitor_task, "mem_monitor", 4096, NULL, 5, NULL);
    // ESP_LOGI(TAG, "Memory monitor task created");

}
