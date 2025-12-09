/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/param.h>
#include "esp_wifi.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ADDR_IS_FULL(addr)        (((addr)[0] & (addr)[1] & (addr)[2] & (addr)[3] & (addr)[4] & (addr)[5]) == 0xFF)
#define ADDR_IS_EMPTY(addr)         (((addr)[0] | (addr)[1] | (addr)[2] | (addr)[3] | (addr)[4] | (addr)[5]) == 0x0)

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
#define WIFI_CSI_PHY_GAIN_ENABLE          1
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 1)
#define WIFI_CSI_SEND_NULL_DATA_ENABLE    1
#endif

#ifdef CONFIG_SPIRAM
#define MALLOC_CAP_INDICATE MALLOC_CAP_SPIRAM
#else
#define MALLOC_CAP_INDICATE MALLOC_CAP_DEFAULT
#endif

#define RADAR_MALLOC_RETRY(size) ({                                                                                                        \
    void *ptr = NULL;                                                                                                                  \
    while (size > 0 && !(ptr = heap_caps_malloc(size, MALLOC_CAP_INDICATE))) {                                                         \
        ESP_LOGW("esp_radar", "<ESP_ERR_NO_MEM> %s:%d Realloc size: %d, ptr: %p, heap free: %d", __func__, __LINE__, (int)size, ptr, esp_get_free_heap_size()); \
        vTaskDelay(pdMS_TO_TICKS(100));                                                                                                \
    }                                                                                                                                  \
    ptr;                                                                                                                               \
})

#define RADAR_FREE(ptr) do { \
    if (ptr) { \
        ESP_LOGI("esp_radar", "free ptr=%p @%s:%d, heap free: %d", (void *)(ptr), __func__, __LINE__, esp_get_free_heap_size()); \
        free(ptr); \
        ptr = NULL; \
    } \
} while(0)

typedef enum {
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    RX_FORMAT_11B      = 0,           /**< the reception frame is a 11b MPDU */
    RX_FORMAT_11G      = 1,           /**< the reception frame is a 11g MPDU */
    RX_FORMAT_11A = RX_FORMAT_11G, /**< the reception frame is a 11a MPDU */
    RX_FORMAT_HT       = 2,           /**< the reception frame is a HT MPDU */
    RX_FORMAT_VHT      = 3,           /**< the reception frame is a VHT MPDU */
    RX_FORMAT_HE_SU    = 4,           /**< the reception frame is a HE SU MPDU */
    RX_FORMAT_HE_MU    = 5,           /**< the reception frame is a HE MU MPDU */
#elif CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
    RX_FORMAT_NON_HT   = 0,           /**< the reception frame is a 11b MPDU */
    RX_FORMAT_HT       = 1,           /**< the reception frame is a HT MPDU */
    RX_FORMAT_VHT      = 2,           /**< the reception frame is a VHT MPDU */
#endif
} wifi_rx_format_t;

typedef enum {
    WIFI_SIGNAL_MODE_NON_HT,  /** 11b/g */
    WIFI_SIGNAL_MODE_HT,      /** 11n   */
    WIFI_SIGNAL_MODE_HE,      /** 11ax  */
} wifi_signal_mode_t;

typedef enum {
    WIFI_CHANNEL_BANDWIDTH_20MHZ = 0,  /** 20MHz */
    WIFI_CHANNEL_BANDWIDTH_40MHZ = 1,  /** 40MHz */
} wifi_channel_bandwidth_t;

typedef struct {
    float waveform_jitter;  /**< Jitter of the radar waveform, 用于检测人体移动 */
    float waveform_wander;  /**< Wander of the radar waveform，用于检测人体存在 */
} wifi_radar_info_t;

typedef struct {
    int8_t rssi;
    uint8_t rate;
    wifi_rx_format_t rx_format;
    wifi_signal_mode_t signal_mode;
    uint8_t mcs;
    uint8_t cwb;
    uint8_t stbc;
    uint8_t agc_gain;
    int8_t fft_gain;
    uint32_t timestamp;
    int8_t noise_floor;
    uint8_t channel;
    uint8_t secondary_channel;
} esp_radar_rx_ctrl_info_t;

typedef struct {
    uint16_t start;
    uint16_t stop;
} sub_carrier_range_t;

typedef struct {
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
    wifi_second_chan_t second;
#endif
    wifi_signal_mode_t signal_mode;
    wifi_channel_bandwidth_t channel_bandwidth;
    bool stbc; /**< Space Time Block Code(STBC). 0: non STBC packet; 1: STBC packet */
    size_t total_bytes;
    size_t valid_bytes;
    union {
        struct {
            uint16_t llft_bytes;
            uint16_t ht_lft_bytes;
            uint16_t stbc_ht_lft_bytes;
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
            uint16_t he_ltf_bytes;
            uint16_t stbc_he_ltf_bytes;
#endif
        };
        uint16_t sub_carrier_bytes[5];
    };
    sub_carrier_range_t llft[2];
    sub_carrier_range_t ht_lft[4];
    sub_carrier_range_t stbc_ht_lft[4];
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    sub_carrier_range_t he_ltf[4];
    sub_carrier_range_t stbc_he_ltf[4];
#endif
} csi_sub_carrier_table_t;

extern const csi_sub_carrier_table_t sub_carrier_table[];
extern const size_t sub_carrier_table_size;

typedef enum {
    WIFI_CSI_DATA_TYPE_INT8  = 0,   /**< CSI samples are stored as 8-bit signed integers */
    WIFI_CSI_DATA_TYPE_INT16 = 1,   /**< CSI samples are stored as 16-bit signed integers */
} wifi_csi_data_type_t;

/**
 * @brief Channel state information(CSI) configuration type
 */
typedef struct {
    wifi_csi_info_t *info;
    esp_radar_rx_ctrl_info_t rx_ctrl_info;     /**< received packet radio metadata header of the CSI data */
    uint32_t seq_id;                /**< 自增序号，用于帧级追踪 */
    uint8_t mac[6];                 /**< source MAC address of the CSI data */
    uint8_t dmac[6];                /**< destination MAC address of the CSI data */
    float rx_gain_compensation;    /**< RX gain compensation */
    wifi_csi_data_type_t data_type; /**< actual data width used by valid_data */
    uint16_t raw_len;               /**< length of the CSI data */
    int8_t *raw_data;               /**< pointer to the CSI data，Unfiltered contains invalid subcarriers*/
    uint16_t valid_len;              /**< length of the CSI data after filtering */
    uint16_t valid_llft_len;         /**< length of the LL-LTF data after filtering */
    uint16_t valid_ht_lft_len;       /**< length of the HT-LTF data after filtering */
    uint16_t valid_stbc_ht_lft_len;  /**< length of the STBC-HT-LTF data after filtering */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    uint16_t valid_he_ltf_len;       /**< length of the HE-LTF data after filtering */
    uint16_t valid_stbc_he_ltf_len;  /**< length of the STBC-HE-LTF data after filtering */
#endif
    union {
        int8_t  valid_data[0];      /**< CSI data interpreted as 8-bit samples */
        int16_t valid_data_i16[0];  /**< CSI data interpreted as 16-bit samples */
    };
} wifi_csi_filtered_info_t;

/**
  * @brief The RX callback function of Wi-Fi radar data.
  *
  *        Each time Wi-Fi radar data analysis, the callback function will be called.
  *
  * @param info Wi-Fi radar data received. The memory that it points to will be deallocated after callback function returns.
  * @param ctx context argument, passed to esp_radar_set_config() when registering callback function.
  *
  */
typedef void (*wifi_radar_cb_t)(void *ctx, const wifi_radar_info_t *info);

/**
 * @brief The RX callback function of Wi-Fi CSI data.
 *
 */
/**
 * @brief The RX callback function of Wi-Fi CSI data.
 *
 * @param info       Pointer to the CSI data (preprocessed or raw).
 * @param preprocess Whether to preprocess the data:
 *                   - false: output original/raw data
 *                   - true : output preprocessed data according to wifi packet type (e.g., lltf, htltf1, htltf2, heltf, etc.)
 */
typedef void (*wifi_csi_filtered_cb_t)(void *ctx, const wifi_csi_filtered_info_t *info);

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
typedef enum {
    CSI_HE_STBC_MODE_LTF1 = 0,      // acquire the complete HE-LTF1
    CSI_HE_STBC_MODE_LTF2 = 1,      // acquire the complete HE-LTF2
    CSI_HE_STBC_MODE_AVERAGE = 2,   // acquire average of HE-LTF1 and HE-LTF2
} csi_he_stbc_mode_t;
#endif

typedef struct {
    wifi_csi_filtered_cb_t csi_filtered_cb; /**< Register the callback function of Wi-Fi CSI data */
    void *csi_filtered_cb_ctx;                 /**< Context argument, passed to callback function of Wi-Fi CSI */
    bool csi_compensate_en;         /**< Whether to enable CSI compensation */
    uint8_t filter_mac[6];                 /**< Get the mac of the specified device, no filtering: [0xff:0xff:0xff:0xff:0xff:0xff] */
    uint8_t filter_dmac[6];                /**< Get the destination mac of the specified device, no filtering: [0xff:0xff:0xff:0xff:0xff:0xff] */
    bool filter_dmac_flag;                 /**< Whether to enable destination MAC address filtering */
    uint16_t csi_recv_interval;       /**< The interval of receiving CSI data, unit: ms */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    bool acquire_csi_lltf       : 1;    /**< enable to acquire L-LTF */
    bool acquire_csi_ht20       : 1;    /**< enable to acquire HT-LTF when receiving an HT20 PPDU */
    bool acquire_csi_ht40       : 1;    /**< enable to acquire HT-LTF when receiving an HT40 PPDU */
    bool acquire_csi_vht        : 1;    /**< enable to acquire VHT-LTF when receiving an VHT20 PPDU */
    bool acquire_csi_su         : 1;    /**< enable to acquire HE-LTF when receiving an HE20 SU PPDU */
    bool acquire_csi_mu         : 1;    /**< enable to acquire HE-LTF when receiving an HE20 MU PPDU */
    bool acquire_csi_dcm        : 1;    /**< enable to acquire HE-LTF when receiving an HE20 DCM applied PPDU */
    bool acquire_csi_beamformed : 1;    /**< enable to acquire HE-LTF when receiving an HE20 Beamformed applied PPDU */
    csi_he_stbc_mode_t acquire_csi_he_stbc_mode;   /**/
    uint8_t val_scale_cfg           : 4;   /**< value 0-8 */
    bool dump_ack_en             : 1;   /**< enable to dump 802.11 ACK frame, default disabled */
#endif
#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32
    bool acquire_csi_lltf;          /**< enable to receive legacy long training field(lltf) data. Default enabled */
    bool acquire_csi_htltf;         /**< enable to receive HT long training field(htltf) data. Default enabled */
    bool acquire_csi_stbc_htltf2;   /**< enable to receive space time block code HT long training field(stbc-htltf2) data. Default enabled */
    bool ltf_merge_en;              /**< enable to generate htlft data by averaging lltf and ht_ltf data when receiving HT packet. Otherwise, use ht_ltf data directly. Default enabled */
    bool channel_filter_en;         /**< enable to turn on channel filter to smooth adjacent sub-carrier. Disable it to keep independence of adjacent sub-carrier. Default enabled */
    bool manu_scale;                /**< manually scale the CSI data by left shifting or automatically scale the CSI data. If set true, please set the shift bits. false: automatically. true: manually. Default false */
    uint8_t shift;                  /**< manually left shift bits of the scale of the CSI data. The range of the left shift bits is 0~15 */
#endif
} esp_radar_csi_config_t;

typedef struct {
    wifi_band_mode_t band_mode;
    wifi_protocols_t protocols;
    wifi_bandwidths_t bandwidths;
    uint8_t channel;
    wifi_second_chan_t second_chan;
    uint8_t mac_address[6];
} esp_radar_wifi_config_t;

typedef struct {
    wifi_phy_rate_t rate;
    wifi_phy_mode_t phy_mode;
} esp_radar_espnow_config_t;

typedef enum {
    RADAR_LTF_TYPE_LLTF = 0,        /**< Use Legacy Long Training Field (L-LTF) */
    RADAR_LTF_TYPE_HTLTF,           /**< Use HT Long Training Field (HT-LTF) */
    RADAR_LTF_TYPE_STBC_HTLTF,      /**< Use STBC HT Long Training Field (STBC-HT-LTF) */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    RADAR_LTF_TYPE_HELTF,           /**< Use HE Long Training Field (HE-LTF) */
    RADAR_LTF_TYPE_STBC_HELTF,      /**< Use STBC HE Long Training Field (STBC-HE-LTF) */
#endif
} esp_radar_ltf_type_t;

typedef struct {
    wifi_radar_cb_t wifi_radar_cb;         /**< Register the callback function of Wi-Fi radar data */
    void *wifi_radar_cb_ctx;               /**< Context argument, passed to callback function of Wi-Fi radar */
    esp_radar_ltf_type_t ltf_type;         /**< LTF type to use for radar analysis */
    uint8_t sub_carrier_step_size;         /**< Sub-carrier step size for CSI data sampling, default: 4 */
    uint8_t outliers_threshold;            /**< CSI outliers threshold for filtering, set to 0 to disable, default: 8 */
    struct {                                /**< Algorithm configuration */
        UBaseType_t csi_handle_priority;  /**< The priority of the task that handles the CSI data */
        UBaseType_t csi_combine_priority; /**< The priority of the task that combines the CSI data */
        uint16_t csi_handle_time;         /**< The time of handling CSI data, unit: ms */
        uint8_t pca_window_size;          /**< PCA结果滑动窗口大小，默认4 */
    };
} esp_radar_config_t;

#define ESP_RADAR_WIFI_CONFIG_DEFAULT() (esp_radar_wifi_config_t){ \
    .band_mode = WIFI_BAND_MODE_AUTO, \
    .protocols = { \
        .ghz_2g = WIFI_PROTOCOL_11N, \
        .ghz_5g = WIFI_PROTOCOL_11N, \
    }, \
    .bandwidths = { \
        .ghz_2g = WIFI_BW_HT40, \
        .ghz_5g = WIFI_BW_HT40, \
    }, \
    .channel = 11, \
    .second_chan = WIFI_SECOND_CHAN_BELOW, \
    .mac_address = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00}, \
}

#define ESP_RADAR_ESPNOW_CONFIG_DEFAULT() (esp_radar_espnow_config_t){ \
    .rate = WIFI_PHY_RATE_MCS0_LGI, \
    .phy_mode = WIFI_PHY_MODE_HT40, \
}

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
#define ESP_RADAR_CSI_CONFIG_DEFAULT() (esp_radar_csi_config_t){ \
    .csi_filtered_cb = NULL, \
    .csi_filtered_cb_ctx = NULL, \
    .csi_compensate_en = true, \
    .filter_mac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, \
    .filter_dmac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, \
    .filter_dmac_flag = false, \
    .csi_recv_interval = 10, \
    .acquire_csi_lltf = false, \
    .acquire_csi_ht20 = true, \
    .acquire_csi_ht40 = true, \
    .acquire_csi_vht = true, \
    .acquire_csi_su = true, \
    .acquire_csi_mu = true, \
    .acquire_csi_dcm = true, \
    .acquire_csi_beamformed = true, \
    .acquire_csi_he_stbc_mode = CSI_HE_STBC_MODE_AVERAGE, \
    .val_scale_cfg = 0, \
    .dump_ack_en = false, \
}
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32
#define ESP_RADAR_CSI_CONFIG_DEFAULT() (esp_radar_csi_config_t){ \
    .csi_filtered_cb = NULL, \
    .csi_filtered_cb_ctx = NULL, \
    .csi_compensate_en = true, \
    .filter_mac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, \
    .filter_dmac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, \
    .filter_dmac_flag = false, \
    .csi_recv_interval = 10, \
    .acquire_csi_lltf = true, \
    .acquire_csi_htltf = true, \
    .acquire_csi_stbc_htltf2 = true, \
    .ltf_merge_en = false, \
    .channel_filter_en = false, \
    .manu_scale = false, \
    .shift = 0, \
}
#else
#define ESP_RADAR_CSI_CONFIG_DEFAULT() (esp_radar_csi_config_t){0}
#endif

#define ESP_RADAR_CONFIG_DEFAULT() (esp_radar_config_t){ \
    .wifi_radar_cb = NULL, \
    .wifi_radar_cb_ctx = NULL, \
    .ltf_type = RADAR_LTF_TYPE_HTLTF, \
    .sub_carrier_step_size = 4, \
    .outliers_threshold = 8, \
    .csi_handle_priority = configMAX_PRIORITIES - 1, \
    .csi_combine_priority = configMAX_PRIORITIES - 1, \
    .csi_handle_time = 100, \
    .pca_window_size = 4, \
}
esp_err_t esp_radar_start(void);
esp_err_t esp_radar_set_csi_config(esp_radar_csi_config_t *config);
esp_err_t esp_radar_wifi_init(esp_radar_wifi_config_t *wifi_cfg, esp_radar_espnow_config_t *espnow_cfg);
esp_err_t esp_radar_init(esp_radar_config_t *config);
esp_err_t esp_radar_train_start(void);
esp_err_t esp_radar_train_remove(void);
esp_err_t esp_radar_train_stop(float *wander_threshold, float *jitter_threshold);

#ifdef __cplusplus
}
#endif
