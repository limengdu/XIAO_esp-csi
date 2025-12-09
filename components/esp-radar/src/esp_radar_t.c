/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_mac.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_private/esp_wifi_he_types_private.h"
#include "esp_radar_t.h"
#include "wifi_rx_gain.h"
#include "utils.h"
static const char *TAG                          = "esp_radar";
static const char *TAG_DETECTION                = "csi_detection_task";
static const char *TAG_TRAIN                     = "esp_radar_train";
// CSI 任务退出事件位
#define CSI_DETECTION_EXIT_BIT      BIT0
#define CSI_PREPROCESSING_EXIT_BIT  BIT1

// PCA 结果滑动窗口大小默认值（可通过配置覆盖）
#define RADAR_PCA_WINDOW_DEFAULT      4

/**
 * @brief CSI 振幅环形缓冲区
 *
 * - 只负责存储数据本身（振幅和时间戳）
 * - 环形读写的索引、窗口滑动等逻辑由上层管理（见 csi_preprocessing_task 及辅助函数）
 */
typedef struct {
    float **amplitude;      // 振幅数据二维数组: [buff_size][subcarrier_len]
    uint32_t *timestamp;    // 时间戳数组: [buff_size]
    uint32_t *seq_id;       // 帧序号数组: [buff_size]
} csi_data_buff_t;

typedef struct {
    uint32_t buff_size;         // 环形缓冲区容量（帧数）
    uint32_t handle_window;     // 目标处理窗口大小（帧数）
    uint32_t window_start_seq;  // 当前窗口起始帧的绝对序号
    uint32_t next_seq;          // 下一个要写入的帧绝对序号
    uint32_t last_timestamp;    // 上一次已处理帧的时间戳（ms）
} csi_window_ctx_t;

typedef struct {
    EventGroupHandle_t          task_exit_group;
    QueueHandle_t               csi_info_queue;
    QueueHandle_t               csi_data_queue;
    esp_radar_config_t          radar_config;
    esp_radar_csi_config_t      csi_config;
    esp_radar_wifi_config_t     wifi_config;
    esp_radar_espnow_config_t   espnow_config;
    bool                        init_flag;
    bool                        run_flag;
    csi_window_ctx_t            window_ctx;         // 环形缓冲 + 窗口管理上下文
    uint16_t                    subcarrier_len;     // 子载波长度 (动态计算)
    csi_data_buff_t             *csi_data_buff;
    bool                        lltf_bit_mode;
} radar_ctx_t;

typedef struct {
    union  {
        struct {
            uint8_t begin;      // 窗口起始位置
            uint8_t end;        // 窗口结束位置
            uint8_t window;     // 窗口大小
            uint8_t reserved;   // 保留字节
        };
        uint32_t data;          // 作为32位整数访问
    };
} csi_data_buff_index_t;

#define CSI_CORR_NUM              10
#define CSI_CORR_THRESHOLD        0.998f
#define RADAR_BUFF_NUM            3
#define RADAR_OUTLIERS_THRESHOLD  0.005f

typedef enum {
    RADAR_CALIBRATE_NO = 0,
    RADAR_CALIBRATE_PROGRESS,
    RADAR_CALIBRATE_COMPLETE,
} radar_calibrate_status_t;

typedef struct {
    radar_calibrate_status_t calibrate_status;
    float *data[CSI_CORR_NUM];
    size_t buff_size;
    float waveform_jitter_buff[RADAR_BUFF_NUM];
    size_t data_num;
    float none_corr_sum;
    float none_corr_count;
    float none_corr;
    float static_corr;
    uint16_t subcarrier_len;
} radar_calibrate_t;

static radar_ctx_t s_ctx = {0};
static uint32_t s_csi_seq = 0;              // CSI 帧自增序号
static uint32_t s_pca_subcarrier_buff_num = 0;
static radar_calibrate_t *s_radar_calibrate = NULL;
static float s_waveform_wander_last = 0.0f;

// #define BB_REG 0x600a790c
// uint32_t reg = REG_READ(BB_REG);
// printf("reg:0x%x, value:0x%lx\n", BB_REG, reg);
// reg |= (BIT(12) | BIT(11));
// REG_WRITE(BB_REG, reg);
// reg = REG_READ(BB_REG);
// printf("reg:0x%x, value:0x%lx\n", BB_REG, reg);

static esp_err_t esp_radar_extract_rx_ctrl_info(const wifi_pkt_rx_ctrl_t *rx_ctrl, esp_radar_rx_ctrl_info_t *info)
{
    if (!rx_ctrl || !info) {
        return ESP_ERR_INVALID_ARG;
    }

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    info->rssi = rx_ctrl->rssi;
    info->rate = rx_ctrl->rate;
    info->rx_format = rx_ctrl->cur_bb_format;
    esp_radar_get_rx_gain(rx_ctrl, &info->agc_gain, &info->fft_gain);
    info->timestamp = rx_ctrl->timestamp;
    info->noise_floor = rx_ctrl->noise_floor;
    info->channel = rx_ctrl->channel;
    info->secondary_channel = rx_ctrl->second;
    switch (rx_ctrl->cur_bb_format) {
    case RX_FORMAT_11B:
    case RX_FORMAT_11G:  // RX_FORMAT_11A has the same value as RX_FORMAT_11G
        info->signal_mode = WIFI_SIGNAL_MODE_NON_HT;
        info->mcs = ((esp_wifi_htsig_t *)&rx_ctrl->he_siga1)->mcs;
        info->cwb = ((esp_wifi_htsig_t *)&rx_ctrl->he_siga1)->cbw;
        info->stbc = ((esp_wifi_htsig_t *)&rx_ctrl->he_siga1)->stbc;
        break;
    case RX_FORMAT_HT:
        info->signal_mode = WIFI_SIGNAL_MODE_HT;
        info->mcs = ((esp_wifi_htsig_t *)&rx_ctrl->he_siga1)->mcs;
        info->cwb = ((esp_wifi_htsig_t *)&rx_ctrl->he_siga1)->cbw;
        info->stbc = ((esp_wifi_htsig_t *)&rx_ctrl->he_siga1)->stbc;
        break;
    case RX_FORMAT_VHT:
        info->signal_mode = WIFI_SIGNAL_MODE_HT;
        info->mcs = ((esp_wifi_vht_siga1_t *)&rx_ctrl->he_siga1)->su_mcs;
        info->cwb = ((esp_wifi_vht_siga1_t *)&rx_ctrl->he_siga1)->cbw;
        info->stbc = ((esp_wifi_vht_siga1_t *)&rx_ctrl->he_siga1)->stbc;
        break;
    case RX_FORMAT_HE_SU:
        info->signal_mode = WIFI_SIGNAL_MODE_HE;
        info->mcs = ((esp_wifi_su_siga1_t *)&rx_ctrl->he_siga1)->he_mcs;
        info->cwb = ((esp_wifi_su_siga1_t *)&rx_ctrl->he_siga1)->bw;
        info->stbc = ((esp_wifi_su_siga2_t *)&rx_ctrl->he_siga2)->stbc;
        break;
    case RX_FORMAT_HE_MU:
        info->signal_mode = WIFI_SIGNAL_MODE_HE;
        info->mcs = ((esp_wifi_mu_siga1_t *)&rx_ctrl->he_siga1)->sigb_mcs;
        info->cwb = ((esp_wifi_mu_siga1_t *)&rx_ctrl->he_siga1)->bw;
        info->stbc = ((esp_wifi_mu_siga2_t *)&rx_ctrl->he_siga2)->stbc;
        break;
    default:
        return ESP_FAIL;
    }

#elif CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
    info->rssi = rx_ctrl->rssi;
    info->rate = rx_ctrl->rate;
    info->rx_format = rx_ctrl->sig_mode;
    if (rx_ctrl->sig_mode == RX_FORMAT_NON_HT) {
        info->signal_mode = WIFI_SIGNAL_MODE_NON_HT;
    } else {
        info->signal_mode = WIFI_SIGNAL_MODE_HT;
    }
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
    info->agc_gain = 0;
    info->fft_gain = 0;
#else
    esp_radar_get_rx_gain(rx_ctrl, &info->agc_gain, &info->fft_gain);
#endif
    info->timestamp = rx_ctrl->timestamp;
    info->noise_floor = rx_ctrl->noise_floor;
    info->channel = rx_ctrl->channel;
    info->secondary_channel = rx_ctrl->secondary_channel;
    info->mcs = rx_ctrl->mcs;
    info->cwb = rx_ctrl->cwb;
    info->stbc = rx_ctrl->stbc;
#endif
    return ESP_OK;
}

static esp_err_t esp_radar_mac_addr_filter(wifi_csi_info_t *info)
{
    static const char *TAG_FILTER = "esp_radar_filter";

    // 检查源MAC地址过滤
    if (!ADDR_IS_FULL(s_ctx.csi_config.filter_mac) &&  memcmp(info->mac, s_ctx.csi_config.filter_mac, 6) != 0) {
        ESP_LOGD(TAG_FILTER, "Source MAC mismatch - mac: " MACSTR ", filter_mac: " MACSTR,  MAC2STR(info->mac), MAC2STR(s_ctx.csi_config.filter_mac));
        return ESP_FAIL;
    }

#if WIFI_CSI_SEND_NULL_DATA_ENABLE
    // 如果filter_mac为空，则只接受null data包（payload_len == 14）
    if (ADDR_IS_EMPTY(s_ctx.csi_config.filter_mac) && info->payload_len != 14) {
        ESP_LOGD(TAG_FILTER, "Not null data packet - payload_len: %d", info->payload_len);
        return ESP_FAIL;
    }
#endif

    // 检查目标MAC地址过滤
    if (s_ctx.csi_config.filter_dmac_flag && memcmp(info->dmac, s_ctx.csi_config.filter_dmac, 6) != 0) {
        ESP_LOGD(TAG_FILTER, "Dest MAC mismatch - dmac: " MACSTR ", filter_dmac: " MACSTR, MAC2STR(info->dmac), MAC2STR(s_ctx.csi_config.filter_dmac));
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t esp_radar_check_timestamp_interval(uint32_t timestamp)
{
    static uint32_t s_last_timestamp = 0;

    uint32_t min_interval_us = s_ctx.csi_config.csi_recv_interval * 1000 / 5;

    if (timestamp - s_last_timestamp <= min_interval_us) {
        return ESP_FAIL;
    }

    s_last_timestamp = timestamp;
    return ESP_OK;
}

static void copy_subcarrier_data(wifi_csi_filtered_info_t *out_filtered_info, const int8_t *src_buf, const sub_carrier_range_t *range_array, uint16_t target_bytes, uint16_t *field_len)
{
    uint8_t i = 0;
    while (*field_len < target_bytes) {
        uint16_t size = range_array[i].stop - range_array[i].start;

        if (s_ctx.lltf_bit_mode) {
            // 12位有符号数符号扩展：左移4位再算术右移4位
            int16_t *dest = (int16_t *)(out_filtered_info->valid_data + out_filtered_info->valid_len);
            const uint16_t *src = (const uint16_t *)(src_buf + range_array[i].start);
            for (uint16_t j = 0; j < size / 2; j++) {
                dest[j] = ((int16_t)(src[j] << 4)) >> 4;
                printf("(%d,%d)", src[j], dest[j]);
            }
            printf("\n");
        } else {
            memcpy(out_filtered_info->valid_data + out_filtered_info->valid_len, src_buf + range_array[i].start, size);
        }

        *field_len += size;
        out_filtered_info->valid_len += size;
        i++;
    }
}

static esp_err_t esp_radar_rebuild_csi_data(const wifi_csi_info_t *info, const esp_radar_rx_ctrl_info_t *rx_ctrl_info, wifi_csi_filtered_info_t **out_filtered_info)
{
    static const char *TAG_FILTER_CSI = "esp_radar_csi_data_rebuild";
    if (!info || !rx_ctrl_info || !out_filtered_info) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_filtered_info = NULL;

    for (size_t i = 0; i < sub_carrier_table_size; ++i) {
        // 条件1: INT16模式下，检查是否不是NON_HT 20MHz且无STBC
        bool int16_mismatch = (s_ctx.lltf_bit_mode && sub_carrier_table[i].total_bytes != info->len);

        // 条件2: INT8模式下，检查参数是否不匹配
        bool int8_mismatch = (!s_ctx.lltf_bit_mode &&
                              (rx_ctrl_info->signal_mode != sub_carrier_table[i].signal_mode ||
                               rx_ctrl_info->cwb != sub_carrier_table[i].channel_bandwidth ||
                               rx_ctrl_info->stbc != sub_carrier_table[i].stbc
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
                               || rx_ctrl_info->secondary_channel != sub_carrier_table[i].second
#endif
                              ));

        // 如果任一条件满足，跳过当前表项
        if (int16_mismatch || int8_mismatch) {
            continue;
        }

        if (sub_carrier_table[i].total_bytes != info->len) {
            ESP_LOGE(TAG_FILTER_CSI, "total_bytes mismatch! Configured: %d, Received: %d", sub_carrier_table[i].total_bytes, info->len);
            continue;
        }
        const csi_sub_carrier_table_t *sub_carrier_index =  sub_carrier_table + i;
        *out_filtered_info = RADAR_MALLOC_RETRY(sizeof(wifi_csi_filtered_info_t) + sub_carrier_index->valid_bytes);
        ESP_LOGI(TAG_FILTER_CSI, "malloc filtered_info: %p, size: %d", *out_filtered_info, sub_carrier_index->valid_bytes);
        memset(*out_filtered_info, 0, sizeof(wifi_csi_filtered_info_t) + sub_carrier_index->valid_bytes);

        // 使用局部指针变量提高可读性
        wifi_csi_filtered_info_t *filtered_info = *out_filtered_info;

        filtered_info->rx_ctrl_info = *rx_ctrl_info;
        filtered_info->data_type = s_ctx.lltf_bit_mode ? WIFI_CSI_DATA_TYPE_INT16 : WIFI_CSI_DATA_TYPE_INT8;
        filtered_info->raw_data = info->buf;
        filtered_info->raw_len  = info->len;
        memcpy(filtered_info->mac, info->mac, 6);
        memcpy(filtered_info->dmac, info->dmac, 6);

        if (sub_carrier_index->llft_bytes > 0) {
            copy_subcarrier_data(filtered_info, info->buf, sub_carrier_index->llft,
                                 sub_carrier_index->llft_bytes, &filtered_info->valid_llft_len);
        }

        if (sub_carrier_index->ht_lft_bytes > 0) {
            copy_subcarrier_data(filtered_info, info->buf, sub_carrier_index->ht_lft,
                                 sub_carrier_index->ht_lft_bytes, &filtered_info->valid_ht_lft_len);
        }

        if (sub_carrier_index->stbc_ht_lft_bytes > 0) {
            copy_subcarrier_data(filtered_info, info->buf, sub_carrier_index->stbc_ht_lft,
                                 sub_carrier_index->stbc_ht_lft_bytes, &filtered_info->valid_stbc_ht_lft_len);
        }
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
        if (sub_carrier_index->he_ltf_bytes > 0) {
            copy_subcarrier_data(filtered_info, info->buf, sub_carrier_index->he_ltf,
                                 sub_carrier_index->he_ltf_bytes, &filtered_info->valid_he_ltf_len);
        }
        if (sub_carrier_index->stbc_he_ltf_bytes > 0) {
            copy_subcarrier_data(filtered_info, info->buf, sub_carrier_index->stbc_he_ltf,
                                 sub_carrier_index->stbc_he_ltf_bytes, &filtered_info->valid_stbc_he_ltf_len);
        }
#endif

        ESP_LOGD(TAG_FILTER_CSI, "raw_len: %d, valid_len: %d, valid_llft_len: %d, valid_ht_lft_len: %d, valid_stbc_ht_lft_len: %d, "
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
                 "valid_he_ltf_len: %d, valid_stbc_he_ltf_len: %d, "
#endif
                 , filtered_info->raw_len, filtered_info->valid_len, filtered_info->valid_llft_len, filtered_info->valid_ht_lft_len, filtered_info->valid_stbc_ht_lft_len,
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
                 filtered_info->valid_he_ltf_len, filtered_info->valid_stbc_he_ltf_len
#endif
                );
        return ESP_OK;
    }

    ESP_LOGW(TAG_FILTER_CSI, "value fail, len: %d, secondary_channel: %d, sig_mode: %d, cwb: %d, stbc: %d",
             info->len, rx_ctrl_info->secondary_channel, rx_ctrl_info->rx_format, rx_ctrl_info->cwb, rx_ctrl_info->stbc);

    return ESP_FAIL;
}

static void esp_radar_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    static const char *TAG_CB = "esp_radar_csi_rx_cb";
    if (!info || !info->buf) {
        ESP_LOGE(TAG_CB, "<%s> esp_radar_csi_rx_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    if (esp_radar_mac_addr_filter(info) != ESP_OK) {
        return;
    }
    // ESP_RETURN_VOID_ON_ERROR(esp_radar_check_timestamp_interval(info->rx_ctrl.timestamp), TAG_CB, "Timestamp interval too short");

    esp_radar_rx_ctrl_info_t rx_ctrl_info;
    esp_radar_extract_rx_ctrl_info(&info->rx_ctrl, &rx_ctrl_info);
    ESP_LOGI(TAG_CB, "timestamp: %ld, mac: " MACSTR", channel: %d, secondary_channel: %d, rssi: %d, rx_format: %d, cwb: %d, rate: %d, mcs: %d, stbc: %d, noise_floor: %d, len: %d, agc_gain: %d, fft_gain: %d",
             rx_ctrl_info.timestamp, MAC2STR(info->mac), rx_ctrl_info.channel, rx_ctrl_info.secondary_channel, rx_ctrl_info.rssi, rx_ctrl_info.rx_format, rx_ctrl_info.cwb, rx_ctrl_info.rate, rx_ctrl_info.mcs,
             rx_ctrl_info.stbc, rx_ctrl_info.noise_floor, info->len, rx_ctrl_info.agc_gain, rx_ctrl_info.fft_gain);

    wifi_csi_filtered_info_t *filtered_info = NULL;
    ESP_RETURN_VOID_ON_ERROR(esp_radar_rebuild_csi_data(info, &rx_ctrl_info, &filtered_info), TAG_CB, "Failed to filter CSI data");

    uint32_t seq = s_csi_seq++;
    filtered_info->seq_id = seq;
    filtered_info->info = info;

    float compensate_gain = 0;
#if WIFI_CSI_PHY_GAIN_ENABLE
    esp_radar_record_rx_gain(rx_ctrl_info.agc_gain, rx_ctrl_info.fft_gain);
    if (s_ctx.csi_config.csi_compensate_en) {
        bool samples_are_16bit = (filtered_info->data_type == WIFI_CSI_DATA_TYPE_INT16);
        esp_radar_compensate_rx_gain(filtered_info->valid_data, filtered_info->valid_len, samples_are_16bit,
                                     &compensate_gain, rx_ctrl_info.agc_gain, rx_ctrl_info.fft_gain);
    }
#endif
    filtered_info->rx_gain_compensation = compensate_gain;
    if (s_ctx.csi_config.csi_filtered_cb) {
        s_ctx.csi_config.csi_filtered_cb(s_ctx.csi_config.csi_filtered_cb_ctx, filtered_info);
    }

    if (!s_ctx.run_flag) {
        static bool run_flag_warned = false;
        if (!run_flag_warned) {
            ESP_LOGW(TAG_CB, "esp_radar not running, CSI data dropped");
            // run_flag_warned = true;
        }
        RADAR_FREE(filtered_info);
    } else if (!s_ctx.csi_info_queue || xQueueSend(s_ctx.csi_info_queue, &filtered_info, 0) == pdFALSE) {
        ESP_LOGW(TAG_CB, "Failed to send CSI data to queue, data dropped");
        RADAR_FREE(filtered_info);
    }
}
#define my_hypotf(a, b) sqrtf((a) * (a) + (b) * (b))

/**
 * @brief 根据配置的 LTF 类型获取对应的数据指针和长度
 *
 * @param filtered_info CSI 过滤后的信息
 * @param data_ptr 输出：数据指针
 * @param data_len 输出：数据长度
 * @return esp_err_t 成功返回 ESP_OK，失败返回错误码
 */
static esp_err_t esp_radar_get_ltf_data(const wifi_csi_filtered_info_t *filtered_info, void **data_ptr, uint16_t *data_len)
{
    if (!filtered_info || !data_ptr || !data_len) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t *ptr = (uint8_t *)filtered_info->valid_data;
    uint16_t offset = 0;

    switch (s_ctx.radar_config.ltf_type) {
    case RADAR_LTF_TYPE_LLTF:
        *data_ptr = ptr;
        *data_len = filtered_info->valid_llft_len;
        break;

    case RADAR_LTF_TYPE_HTLTF:
        offset = filtered_info->valid_llft_len;
        *data_ptr = ptr + offset;
        *data_len = filtered_info->valid_ht_lft_len;
        break;

    case RADAR_LTF_TYPE_STBC_HTLTF:
        offset = filtered_info->valid_llft_len + filtered_info->valid_ht_lft_len;
        *data_ptr = ptr + offset;
        *data_len = filtered_info->valid_stbc_ht_lft_len;
        break;

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    case RADAR_LTF_TYPE_HELTF:
        offset = filtered_info->valid_llft_len + filtered_info->valid_ht_lft_len +
                 filtered_info->valid_stbc_ht_lft_len;
        *data_ptr = ptr + offset;
        *data_len = filtered_info->valid_he_ltf_len;
        break;

    case RADAR_LTF_TYPE_STBC_HELTF:
        offset = filtered_info->valid_llft_len + filtered_info->valid_ht_lft_len +
                 filtered_info->valid_stbc_ht_lft_len + filtered_info->valid_he_ltf_len;
        *data_ptr = ptr + offset;
        *data_len = filtered_info->valid_stbc_he_ltf_len;
        break;
#endif

    default:
        ESP_LOGE(TAG, "Unsupported LTF type: %d", s_ctx.radar_config.ltf_type);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (*data_len == 0) {
        ESP_LOGW(TAG, "LTF type %d has no data", s_ctx.radar_config.ltf_type);
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

static esp_err_t csi_outlier_filter_process(uint16_t subcarrier_len)
{
    const uint8_t threshold = s_ctx.radar_config.outliers_threshold;
    uint32_t curr_seq = s_ctx.window_ctx.next_seq;

    if (curr_seq < 3) {
        return ESP_OK;
    }

    float *csi_data_buff_first  = s_ctx.csi_data_buff->amplitude[(curr_seq - 3) % s_ctx.window_ctx.buff_size];
    float *csi_data_buff_second = s_ctx.csi_data_buff->amplitude[(curr_seq - 2) % s_ctx.window_ctx.buff_size];
    float *csi_data_buff_third  = s_ctx.csi_data_buff->amplitude[(curr_seq - 1) % s_ctx.window_ctx.buff_size];
    float *csi_data_buff_fourth = s_ctx.csi_data_buff->amplitude[curr_seq % s_ctx.window_ctx.buff_size];

    // 异常值检测
    uint32_t outliers_count = 0;

    for (int i = 0; i < subcarrier_len; ++i) {
        float diff_0 = csi_data_buff_second[i] - csi_data_buff_first[i];
        float diff_1 = csi_data_buff_second[i] - csi_data_buff_third[i];
        float diff_2 = csi_data_buff_second[i] - csi_data_buff_fourth[i];

        if ((diff_0 > threshold && diff_1 > threshold)
                || (diff_0 < -threshold && diff_1 < -threshold)
                || (diff_0 > threshold && diff_2 > threshold)) {
            outliers_count++;
            // ESP_LOGW(TAG, "data: %f, diff: %f, %f", csi_data_buff_second[i], diff_0, diff_1);
        }
    }

    if (outliers_count >= subcarrier_len / 2) {
        ESP_LOGW(TAG, "Outliers detected: %lu/%d at seq=%lu, discarding frame", outliers_count, subcarrier_len, curr_seq);
        return ESP_FAIL;
    }

    return ESP_OK;  // 数据正常，可以继续处理
}

static esp_err_t csi_window_update(csi_data_buff_index_t *buff_index)
{
    if (!s_ctx.csi_data_buff || s_ctx.window_ctx.buff_size == 0) {
        return ESP_FAIL;
    }

    buff_index->begin  = s_ctx.window_ctx.window_start_seq % s_ctx.window_ctx.buff_size;
    buff_index->end    = s_ctx.window_ctx.next_seq % s_ctx.window_ctx.buff_size;
    buff_index->window = s_ctx.window_ctx.next_seq - s_ctx.window_ctx.window_start_seq;

    int32_t spent_time = s_ctx.csi_data_buff->timestamp[buff_index->end] - s_ctx.csi_data_buff->timestamp[buff_index->begin];
    int32_t time_tamp  = s_ctx.csi_data_buff->timestamp[buff_index->end] - (int32_t)(s_ctx.window_ctx.last_timestamp);

    esp_err_t ret = ESP_FAIL;

    // 1) 时间间隔异常：认为当前窗口中断，必要时仍然输出一个窗口
    if (time_tamp < 0 || time_tamp > (int32_t)(s_ctx.radar_config.csi_handle_time / 2)) {
        ESP_LOGW(TAG, "time_tamp: %d, spent_time: %d,csi_handle_time: %d, end: %d, last: %d, window: %d,handle_window: %d",
                 time_tamp, spent_time, s_ctx.radar_config.csi_handle_time, s_ctx.csi_data_buff->timestamp[buff_index->end], s_ctx.window_ctx.last_timestamp,
                 buff_index->window, s_ctx.window_ctx.handle_window);

        if (buff_index->window > s_ctx.window_ctx.handle_window / 3) {
            // 收缩一个点，避免窗口过大；end 回退一帧
            buff_index->window--;
            buff_index->end = (s_ctx.window_ctx.next_seq - 1) % s_ctx.window_ctx.buff_size;
            ret = ESP_OK;
        }
        s_ctx.window_ctx.window_start_seq = s_ctx.window_ctx.next_seq;
        goto UPDATE_STATE;
    }

    // 2) 正常情况下，当时间跨度或窗口大小达到阈值时触发处理
    if (spent_time >= (int32_t)(s_ctx.radar_config.csi_handle_time * 2) || buff_index->window >= s_ctx.window_ctx.handle_window) {
        if (buff_index->window < s_ctx.window_ctx.handle_window / 3) {
            s_ctx.window_ctx.window_start_seq = s_ctx.window_ctx.next_seq;
            ESP_LOGW(TAG, "buff_index.window: %d, spent_time: %d, csi_handle_time: %d, handle_window: %d",
                     buff_index->window, spent_time, s_ctx.radar_config.csi_handle_time, s_ctx.window_ctx.handle_window);
            goto UPDATE_STATE;
        }
        ESP_LOGI(TAG, "buff_index.window: %d, time_tamp: %d, spent_time: %d, csi_handle_time: %d, handle_window: %d,buff_index-begin: %d, buff_index-end: %d",
                 buff_index->window, time_tamp, spent_time, s_ctx.radar_config.csi_handle_time, s_ctx.window_ctx.handle_window, buff_index->begin, buff_index->end);
        ret = ESP_OK;
        s_ctx.window_ctx.window_start_seq += buff_index->window / 2;
    }

UPDATE_STATE:
    s_ctx.window_ctx.last_timestamp = s_ctx.csi_data_buff->timestamp[buff_index->end];
    s_ctx.window_ctx.next_seq++;
    return ret;
}

static void csi_prepare_amplitude(uint16_t subcarrier_len)
{

    size_t pointer_bytes = (size_t)s_ctx.window_ctx.buff_size * sizeof(float *);
    size_t data_bytes    = (size_t)s_ctx.window_ctx.buff_size * subcarrier_len * sizeof(float);
    size_t total_bytes   = pointer_bytes + data_bytes;
    uint8_t *amplitude_block = RADAR_MALLOC_RETRY(total_bytes);
    memset(amplitude_block, 0, total_bytes);

    s_ctx.csi_data_buff->amplitude = (float **)amplitude_block;
    float *data_base = (float *)(amplitude_block + pointer_bytes);
    for (uint32_t i = 0; i < s_ctx.window_ctx.buff_size; i++) {
        s_ctx.csi_data_buff->amplitude[i] = data_base + (i * subcarrier_len);
    }
}

static void csi_write_frame_to_ring(const void *ltf_data, uint16_t subcarrier_len, wifi_csi_filtered_info_t *filtered_info)
{
    if (!s_ctx.csi_data_buff || !s_ctx.csi_data_buff->amplitude ||
            !s_ctx.csi_data_buff->seq_id || !s_ctx.csi_data_buff->timestamp) {
        return;
    }

    uint32_t write_index = s_ctx.window_ctx.next_seq % s_ctx.window_ctx.buff_size;

    s_ctx.csi_data_buff->seq_id[write_index] = filtered_info->seq_id;
    s_ctx.csi_data_buff->timestamp[write_index] = filtered_info->rx_ctrl_info.timestamp / 1000;;

    float *csi_data = s_ctx.csi_data_buff->amplitude[write_index];
    const uint8_t step = s_ctx.radar_config.sub_carrier_step_size;

    for (int i = 0; i < subcarrier_len; i++) {
        if (filtered_info->data_type == WIFI_CSI_DATA_TYPE_INT16) {
            csi_data[i] = my_hypotf(((const int16_t *)ltf_data)[i * step * 2], ((const int16_t *)ltf_data)[i * step * 2 + 1]);
        } else {
            csi_data[i] = my_hypotf(((const int8_t *)ltf_data)[i * step * 2], ((const int8_t *)ltf_data)[i * step * 2 + 1]);
        }
    }
}

static void csi_preprocessing_task(void *arg)
{
    wifi_csi_filtered_info_t *filtered_info = NULL;

    while (xQueueReceive(s_ctx.csi_info_queue, &filtered_info, portMAX_DELAY) && s_ctx.run_flag) {
        void *ltf_data = NULL;
        uint16_t ltf_len = 0;
        uint16_t subcarrier_len = 0;
        ESP_LOGE(TAG, "ltf_data: %p, s_ctx.subcarrier_len %d, subcarrier_len: %d", ltf_data, s_ctx.subcarrier_len, subcarrier_len);
        if (esp_radar_get_ltf_data(filtered_info, &ltf_data, &ltf_len) != ESP_OK) {
            goto FREE_MEM;
        }

        uint8_t component_bytes = (filtered_info->data_type == WIFI_CSI_DATA_TYPE_INT16) ? sizeof(int16_t) : sizeof(int8_t);
        subcarrier_len = (uint16_t)((ltf_len / (2U * component_bytes)) / s_ctx.radar_config.sub_carrier_step_size);
        ESP_LOGE(TAG, "ltf_data: %p, s_ctx.subcarrier_len %d, subcarrier_len: %d, filtered_info: %p", ltf_data, s_ctx.subcarrier_len, subcarrier_len, filtered_info);
        if (s_ctx.subcarrier_len == 0) {
            s_ctx.subcarrier_len = subcarrier_len;
            csi_prepare_amplitude(subcarrier_len);
            ESP_LOGI(TAG, "First frame detected: type=%d, LTF length=%d, subcarrier length=%d, step size=%d, allocated buffer: %d x %d",
                     s_ctx.radar_config.ltf_type, ltf_len, subcarrier_len, s_ctx.radar_config.sub_carrier_step_size, s_ctx.window_ctx.buff_size, subcarrier_len);
        } else if (subcarrier_len != s_ctx.subcarrier_len) {
            ESP_LOGE(TAG, "Frame length mismatch detected! Expected type=%d, subcarrier_len=%d, got=%d (LTF len=%d). Discarding frame.",
                     s_ctx.radar_config.ltf_type, s_ctx.subcarrier_len, subcarrier_len, ltf_len);
            goto FREE_MEM;
        }

        csi_write_frame_to_ring(ltf_data, subcarrier_len, filtered_info);
        if (s_ctx.radar_config.outliers_threshold > 0) {
            if (csi_outlier_filter_process(subcarrier_len) != ESP_OK) {
                goto FREE_MEM;
            }
        }

        csi_data_buff_index_t buff_index = {0};
        if (csi_window_update(&buff_index) == ESP_OK) {
            if (!s_ctx.csi_data_queue || xQueueSend(s_ctx.csi_data_queue, &buff_index, portMAX_DELAY) == pdFALSE) {
                ESP_LOGW(TAG, "The buffer is full");
            }
        }

FREE_MEM:
        RADAR_FREE(filtered_info);
    }

    ESP_LOGW(TAG, "csi_preprocessing_task exit");
    xEventGroupSetBits(s_ctx.task_exit_group, CSI_PREPROCESSING_EXIT_BIT);
    vTaskDelete(NULL);
}

static void radar_calibrate_free_entries(radar_calibrate_t *cal)
{
    if (!cal) {
        return;
    }
    for (int i = 0; i < CSI_CORR_NUM; ++i) {
        if (cal->data[i]) {
            RADAR_FREE(cal->data[i]);
            cal->data[i] = NULL;  // 避免悬空指针和重复释放
        }
    }
}

static void radar_calibrate_reset_stats(radar_calibrate_t *cal)
{
    if (!cal) {
        return;
    }

    cal->buff_size = 0;
    cal->none_corr_sum = 0.0f;
    cal->none_corr_count = 0.0f;
    cal->none_corr = 1.0f;
    cal->static_corr = 1.0f;
    cal->subcarrier_len = 0;
}

esp_err_t esp_radar_train_start(void)
{
    if (!s_radar_calibrate) {
        s_radar_calibrate = RADAR_MALLOC_RETRY(sizeof(radar_calibrate_t));
        memset(s_radar_calibrate, 0, sizeof(radar_calibrate_t));
        s_radar_calibrate->data_num = 0;
    }

    radar_calibrate_reset_stats(s_radar_calibrate);
    s_radar_calibrate->calibrate_status = RADAR_CALIBRATE_PROGRESS;
    s_waveform_wander_last = 0.0f;
    s_pca_subcarrier_buff_num = 0;
    ESP_LOGI(TAG_TRAIN, "esp_radar_train_start");
    return ESP_OK;
}

esp_err_t esp_radar_train_remove(void)
{
    if (!s_radar_calibrate) {
        return ESP_OK;
    }

    radar_calibrate_free_entries(s_radar_calibrate);
    radar_calibrate_reset_stats(s_radar_calibrate);
    s_radar_calibrate->data_num = 0;
    s_radar_calibrate->calibrate_status = RADAR_CALIBRATE_NO;
    s_waveform_wander_last = 0.0f;
    ESP_LOGI(TAG_TRAIN, "esp_radar_train_remove");
    return ESP_OK;
}

esp_err_t esp_radar_train_stop(float *wander_threshold, float *jitter_threshold)
{
    if (!s_radar_calibrate) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!s_radar_calibrate->data_num || s_radar_calibrate->none_corr_count == 0.0f) {
        return ESP_ERR_INVALID_STATE;
    }

    s_radar_calibrate->calibrate_status = RADAR_CALIBRATE_COMPLETE;

    if (wander_threshold) {
        *wander_threshold = 1.0f - (s_radar_calibrate->none_corr_sum / s_radar_calibrate->none_corr_count);
    }
    if (jitter_threshold) {
        *jitter_threshold = 1.0f - s_radar_calibrate->static_corr;
    }

    ESP_LOGI(TAG_TRAIN, "esp_radar_train_stop");
    return ESP_OK;
}

static esp_err_t csi_detection_handle_auto_rx_gain(void)
{
#if WIFI_CSI_PHY_GAIN_ENABLE
    if (esp_radar_auto_rx_gain_status()) {
        if (s_radar_calibrate && s_radar_calibrate->calibrate_status == RADAR_CALIBRATE_PROGRESS) {
            ESP_LOGW(TAG_TRAIN, "Auto RX gain active, reset calibration data");
            esp_radar_train_remove();
        }
        // 自动增益调节时丢弃当前窗口并重置 PCA 历史
        s_pca_subcarrier_buff_num = 0;
        return ESP_FAIL;
    }
#endif
    return ESP_OK;
}

static esp_err_t csi_detection_compute_pca(uint16_t cols, const csi_data_buff_index_t *buff_index, float *pca_subcarrier_buff_current)
{
    float (*csi_data_0)[cols] = (float (*)[cols])s_ctx.csi_data_buff->amplitude[buff_index->begin];
    float (*csi_data_1)[cols] = (float (*)[cols])s_ctx.csi_data_buff->amplitude[0];

    uint8_t csi_data_0_len = (buff_index->begin <= buff_index->end) ? buff_index->window : (s_ctx.window_ctx.buff_size - buff_index->begin);
    uint8_t csi_data_1_len = buff_index->window - csi_data_0_len;

    if (csi_data_0_len == 0) {
        return ESP_FAIL;
    }

    return pca(cols, csi_data_0_len, csi_data_0, csi_data_1_len, csi_data_1, pca_subcarrier_buff_current);
}

static void csi_detection_update_waveform_metrics(wifi_radar_info_t *radar_info, float *pca_subcarrier_buff_current, float **pca_subcarrier_buff, uint16_t cols)
{
    uint8_t move_buffer_size = s_ctx.radar_config.pca_window_size;
    if (s_pca_subcarrier_buff_num < move_buffer_size) {
        return;
    }

    radar_info->waveform_jitter = 0.0f;
    float_t corr_temp[move_buffer_size];

    for (int i = 0; i < move_buffer_size - 1; ++i) {
        corr_temp[i] = fabs(corr(pca_subcarrier_buff_current, pca_subcarrier_buff[(s_pca_subcarrier_buff_num - 2 - i) % move_buffer_size], cols));

        if (radar_info->waveform_jitter < corr_temp[i]) {
            radar_info->waveform_jitter = corr_temp[i];
        }
    }

}

/**
 * @brief 计算当前数据与训练样本的波形漂移相关性（检测阶段）
 *
 * @param radar_info 输出雷达信息，设置 waveform_wander 值
 * @param pca_subcarrier_buff_current 当前 PCA 处理后的子载波数据
 * @param cols 子载波长度
 *
 * @note 此函数在检测和训练阶段都会执行
 */
static void csi_detection_compute_wander(wifi_radar_info_t *radar_info, float *pca_subcarrier_buff_current, uint16_t cols)
{
    // 没有训练样本时，wander 为 0（完全不同）
    if (s_radar_calibrate->data_num == 0) {
        radar_info->waveform_wander = 1.0f;
        return;
    }

    // 计算与所有训练样本的最大相关性
    radar_info->waveform_wander = 0.0f;
    for (size_t i = 0; i < s_radar_calibrate->data_num && i < CSI_CORR_NUM; ++i) {
        float *record = s_radar_calibrate->data[i % CSI_CORR_NUM];
        if (!record) {
            continue;
        }
        float corr_val = fabs(corr(record, pca_subcarrier_buff_current, cols));
        if (radar_info->waveform_wander < corr_val) {
            radar_info->waveform_wander = corr_val;
        }
    }
}

/**
 * @brief 采集静态环境训练样本（训练阶段）
 *
 * @param radar_info 雷达信息，可能被修改 waveform_wander
 * @param pca_subcarrier_buff PCA 子载波缓冲区数组
 * @param cols 子载波长度
 *
 * @note 仅在 RADAR_CALIBRATE_PROGRESS 状态下执行
 *       根据上一次的 waveform_wander 决定是否保存训练样本
 */
static void csi_training_collect_sample(wifi_radar_info_t *radar_info, float **pca_subcarrier_buff, uint16_t cols)
{
    uint8_t move_buffer_size = s_ctx.radar_config.pca_window_size;

    // 异常值检测：更新抖动缓冲区
    size_t idx_first = (s_radar_calibrate->buff_size + RADAR_BUFF_NUM - 2) % RADAR_BUFF_NUM;
    size_t idx_second = (s_radar_calibrate->buff_size + RADAR_BUFF_NUM - 1) % RADAR_BUFF_NUM;
    size_t idx_third = s_radar_calibrate->buff_size % RADAR_BUFF_NUM;

    s_radar_calibrate->waveform_jitter_buff[idx_third] = radar_info->waveform_jitter;
    s_radar_calibrate->buff_size++;

    if (s_radar_calibrate->buff_size < RADAR_BUFF_NUM) {
        return;
    }

    // 获取最近3个抖动值
    float first = s_radar_calibrate->waveform_jitter_buff[idx_first];
    float second = s_radar_calibrate->waveform_jitter_buff[idx_second];
    float third = s_radar_calibrate->waveform_jitter_buff[idx_third];

    if ((first - second > RADAR_OUTLIERS_THRESHOLD) && (third - second > RADAR_OUTLIERS_THRESHOLD)) {
        ESP_LOGI(TAG_TRAIN, "Jitter outlier detected: %.4f < %.4f, %.4f", second, first, third);
        return;
    }

    if (s_radar_calibrate->static_corr > radar_info->waveform_jitter) {
        s_radar_calibrate->static_corr = second;
    }

    if (s_waveform_wander_last < CSI_CORR_THRESHOLD) {
        // 上次环境静止 → 保存为训练样本
        float *pca_prev = pca_subcarrier_buff[(s_pca_subcarrier_buff_num + move_buffer_size - 2) % move_buffer_size];
        if (pca_prev) {
            size_t index = s_radar_calibrate->data_num % CSI_CORR_NUM;

            // 分配内存（如果需要）
            if (!s_radar_calibrate->data[index]) {
                s_radar_calibrate->data[index] = RADAR_MALLOC_RETRY((size_t)cols * sizeof(float));
            }

            // 保存训练样本
            memcpy(s_radar_calibrate->data[index], pca_prev, (size_t)cols * sizeof(float));
            s_radar_calibrate->data_num++;
            s_radar_calibrate->none_corr = 1.0f;
            radar_info->waveform_wander = 1.0f;

            ESP_LOGI(TAG_TRAIN, "Training sample collected: num=%zu, corr=%.4f",
                     s_radar_calibrate->data_num, s_waveform_wander_last);
        }
    } else {
        // 环境有活动 → 统计相关性值
        s_radar_calibrate->none_corr = s_waveform_wander_last;
        if (s_waveform_wander_last < 0.99999f) {
            s_radar_calibrate->none_corr_sum += s_waveform_wander_last;
            s_radar_calibrate->none_corr_count += 1.0f;

            ESP_LOGI(TAG_TRAIN, "Training stats: sum=%.4f, count=%.0f, avg=%.4f",
                     s_radar_calibrate->none_corr_sum, s_radar_calibrate->none_corr_count,
                     s_radar_calibrate->none_corr_sum / s_radar_calibrate->none_corr_count);
        }
    }

    // 更新历史状态
    s_waveform_wander_last = radar_info->waveform_wander;
}

static void csi_detection_task(void *arg)
{
    uint8_t move_buffer_size = s_ctx.radar_config.pca_window_size;
    float **pca_subcarrier_buff = RADAR_MALLOC_RETRY(move_buffer_size * sizeof(float *));
    memset(pca_subcarrier_buff, 0, move_buffer_size * sizeof(float *));
    uint16_t cols = 0;
    bool pca_buff_allocated = false;
    csi_data_buff_index_t buff_index = {0};

    while (xQueueReceive(s_ctx.csi_data_queue, &buff_index, portMAX_DELAY) && s_ctx.run_flag) {
        cols = s_ctx.subcarrier_len;
        if (!s_ctx.csi_data_buff->amplitude || cols == 0) {
            ESP_LOGW(TAG_DETECTION, "CSI buffer not ready");
            continue;
        }

        if (!pca_buff_allocated) {
            for (int i = 0; i < move_buffer_size; ++i) {
                pca_subcarrier_buff[i] = RADAR_MALLOC_RETRY(cols * sizeof(float));
            }
            pca_buff_allocated = true;
            ESP_LOGI(TAG_DETECTION, "Allocated PCA subcarrier buffer: %d x %d", move_buffer_size, cols);
        }

        wifi_radar_info_t radar_info = {
            .waveform_jitter = 1,
            .waveform_wander = 1,
        };

        if (csi_detection_handle_auto_rx_gain() != ESP_OK) {
            ESP_LOGW(TAG_DETECTION, "Auto RX gain failed");
            continue;
        }

        uint32_t timestamp_start = esp_log_timestamp();

        float *pca_subcarrier_buff_current = pca_subcarrier_buff[s_pca_subcarrier_buff_num % move_buffer_size];

        static int pr_count = 0;
        if (csi_detection_compute_pca(cols, &buff_index, pca_subcarrier_buff_current) != ESP_OK) {
            ESP_LOGW(TAG_DETECTION, "PCA calculation failed");
            continue;
        }
        s_pca_subcarrier_buff_num++;

        // 打印当前 buff_index 对应的所有 seq
        // printf("pca - buff_index: begin=%d, end=%d, window=%d, seq: [",
        //        buff_index.begin, buff_index.end, buff_index.window);
        // for (int i = 0; i < buff_index.window; i++) {
        //     uint32_t idx = (buff_index.begin + i) % s_ctx.window_ctx.buff_size;
        //     printf("%ld", s_ctx.csi_data_buff->seq_id[idx]);
        //     if (i < buff_index.window - 1) {
        //         printf(", ");
        //     }
        // }
        // printf("]\n");

        // static uint32_t lasttimestamp = 0;
        // printf("%d time :%lu , pca_subcarrier_buff_current: ", pr_count++,esp_log_timestamp() - lasttimestamp);
        // lasttimestamp = esp_log_timestamp();
        // for (int i = 0; i < cols; i++) {
        //     printf("%f ", pca_subcarrier_buff_current[i]);

        // }
        // printf("\n");

        csi_detection_update_waveform_metrics(&radar_info, pca_subcarrier_buff_current, pca_subcarrier_buff, cols);

        // 检测过程：计算波形漂移
        if (s_radar_calibrate) {
            // 初始化或验证子载波长度
            if (s_radar_calibrate->subcarrier_len == 0) {
                s_radar_calibrate->subcarrier_len = cols;
            } else if (s_radar_calibrate->subcarrier_len != cols) {
                // 子载波长度变化（如WiFi模式切换），需要重置训练数据
                ESP_LOGW(TAG_TRAIN, "Subcarrier length changed from %u to %u, reset training data",
                         s_radar_calibrate->subcarrier_len, cols);
                esp_radar_train_remove();
                s_radar_calibrate->subcarrier_len = cols;
            }

            csi_detection_compute_wander(&radar_info, pca_subcarrier_buff_current, cols);

            // 训练过程：采集训练样本（仅在训练模式下执行）
            if (s_radar_calibrate->calibrate_status == RADAR_CALIBRATE_PROGRESS && s_pca_subcarrier_buff_num >= 2) {
                csi_training_collect_sample(&radar_info, pca_subcarrier_buff, cols);
            }
        }

        radar_info.waveform_wander = 1.0f - radar_info.waveform_wander;
        radar_info.waveform_jitter = 1.0f - radar_info.waveform_jitter;

        int32_t time_spent  = s_ctx.csi_data_buff->timestamp[buff_index.end] - s_ctx.csi_data_buff->timestamp[buff_index.begin];
        if (time_spent > 0) {
            ESP_LOGI(TAG_DETECTION, "det_time: %u/%u, free_heap: %d, wander: %f, jitter: %f, window: %d, begin: %d, end: %d, freq: %dHz",
                     time_spent, esp_log_timestamp() - timestamp_start, esp_get_free_heap_size(),
                     radar_info.waveform_wander, radar_info.waveform_jitter, buff_index.window,
                     buff_index.begin, buff_index.end, buff_index.window * 1000 / time_spent);
        }

        if (s_ctx.radar_config.wifi_radar_cb) {
            s_ctx.radar_config.wifi_radar_cb(s_ctx.radar_config.wifi_radar_cb_ctx, &radar_info);
        }
    }

    for (int i = 0; i < move_buffer_size; ++i) {
        if (pca_subcarrier_buff[i]) {
            RADAR_FREE(pca_subcarrier_buff[i]);
        }
    }
    RADAR_FREE(pca_subcarrier_buff);
    s_pca_subcarrier_buff_num = 0;
    ESP_LOGW(TAG_DETECTION, "csi_detection_task  exit");
    xEventGroupSetBits(s_ctx.task_exit_group, CSI_DETECTION_EXIT_BIT);
    vTaskDelete(NULL);
}
esp_err_t esp_radar_start()
{
    if (s_ctx.run_flag) {
        return ESP_OK;
    }

    s_ctx.run_flag = true;
    s_csi_seq = 0;

    s_ctx.csi_data_buff = RADAR_MALLOC_RETRY(sizeof(csi_data_buff_t));
    memset(s_ctx.csi_data_buff, 0, sizeof(csi_data_buff_t));

    s_ctx.csi_info_queue = xQueueCreate(5, sizeof(void *));
    s_ctx.csi_data_queue = xQueueCreate(1, sizeof(csi_data_buff_index_t));
    s_ctx.task_exit_group  = xEventGroupCreate();

    s_ctx.window_ctx.handle_window     = (s_ctx.radar_config.csi_handle_time / s_ctx.csi_config.csi_recv_interval) * 2;
    s_ctx.window_ctx.buff_size         = (s_ctx.radar_config.csi_handle_time / s_ctx.csi_config.csi_recv_interval) * 2 + 20;
    s_ctx.window_ctx.window_start_seq  = 0;
    s_ctx.window_ctx.next_seq          = 0;
    s_ctx.window_ctx.last_timestamp    = 0;

    ESP_LOGI(TAG, "[%s, %d] csi_recv_interval: %d, csi_handle_time: %d, csi_handle_window: %d, csi_handle_buffer: %d",
             __func__, __LINE__,
             s_ctx.csi_config.csi_recv_interval, s_ctx.radar_config.csi_handle_time,
             s_ctx.window_ctx.handle_window, s_ctx.window_ctx.buff_size);

    // amplitude 将在第一次接收到数据时动态分配
    s_ctx.csi_data_buff->amplitude = NULL;
    s_ctx.csi_data_buff->timestamp = RADAR_MALLOC_RETRY(s_ctx.window_ctx.buff_size * sizeof(uint32_t));
    memset(s_ctx.csi_data_buff->timestamp, 0, s_ctx.window_ctx.buff_size * sizeof(uint32_t));
    s_ctx.csi_data_buff->seq_id = RADAR_MALLOC_RETRY(s_ctx.window_ctx.buff_size * sizeof(uint32_t));
    memset(s_ctx.csi_data_buff->seq_id, 0, s_ctx.window_ctx.buff_size * sizeof(uint32_t));

    xTaskCreate(csi_detection_task, "csi_handle", 3 * 1024, NULL, s_ctx.radar_config.csi_handle_priority, NULL);
    xTaskCreate(csi_preprocessing_task, "csi_combine", 3 * 1024, NULL, s_ctx.radar_config.csi_combine_priority, NULL);

    return ESP_OK;
}

esp_err_t esp_radar_stop()
{
    csi_data_buff_index_t buff_index = {0};
    wifi_csi_filtered_info_t *filtered_info = NULL;
    s_ctx.run_flag = false;
    xQueueSend(s_ctx.csi_info_queue, &filtered_info, 0);
    xQueueSend(s_ctx.csi_data_queue, &buff_index, 0);

    xEventGroupWaitBits(s_ctx.task_exit_group, CSI_DETECTION_EXIT_BIT | CSI_PREPROCESSING_EXIT_BIT,
                        pdTRUE, pdTRUE, portMAX_DELAY);

    if (s_ctx.csi_info_queue) {
        while (xQueueReceive(s_ctx.csi_info_queue, &filtered_info, 0)) {
            if (filtered_info) {  // 只释放非 NULL 的
                RADAR_FREE(filtered_info);
            }
        }
    }

    if (s_ctx.csi_data_queue) {
        while (xQueueReceive(s_ctx.csi_data_queue, &buff_index, 0)) ;

        if (s_ctx.csi_data_buff) {
            RADAR_FREE(s_ctx.csi_data_buff->amplitude);
            RADAR_FREE(s_ctx.csi_data_buff->timestamp);
            RADAR_FREE(s_ctx.csi_data_buff->seq_id);
            RADAR_FREE(s_ctx.csi_data_buff);
            s_ctx.csi_data_buff = NULL;
            s_ctx.subcarrier_len = 0;
        }
        vQueueDelete(s_ctx.csi_data_queue);
        s_ctx.csi_data_queue = NULL;
    }

    if (s_ctx.csi_info_queue) {
        vQueueDelete(s_ctx.csi_info_queue);
        s_ctx.csi_info_queue = NULL;
    }

    vEventGroupDelete(s_ctx.task_exit_group);
    s_ctx.task_exit_group = NULL;

    return ESP_OK;
}

esp_err_t esp_radar_set_csi_config(esp_radar_csi_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    wifi_csi_config_t wifi_csi_config;
    s_ctx.lltf_bit_mode = false;
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
    wifi_csi_config = (wifi_csi_config_t) {
        .enable                   = true,
        .acquire_csi_legacy       = config->acquire_csi_lltf,
        .acquire_csi_force_lltf   = config->acquire_csi_lltf,
        .acquire_csi_ht20         = config->acquire_csi_ht20,
        .acquire_csi_ht40         = config->acquire_csi_ht40,
        .acquire_csi_vht          = config->acquire_csi_vht,
        .acquire_csi_su           = config->acquire_csi_su,
        .acquire_csi_mu           = config->acquire_csi_mu,
        .acquire_csi_dcm          = config->acquire_csi_dcm,
        .acquire_csi_beamformed   = config->acquire_csi_beamformed,
        .acquire_csi_he_stbc_mode = config->acquire_csi_he_stbc_mode,
        .val_scale_cfg            = config->val_scale_cfg,
        .dump_ack_en              = config->dump_ack_en,
        .reserved                 = 0
    };
    if (config->acquire_csi_lltf) {
        ESP_LOGW(TAG, "%s: LLTF collection is enabled, other collection settings will be ignored", CONFIG_IDF_TARGET);
    }
    s_ctx.lltf_bit_mode = config->acquire_csi_lltf;
#elif CONFIG_IDF_TARGET_ESP32C6
    wifi_csi_config = (wifi_csi_config_t) {
        .enable                 = true,
        .acquire_csi_legacy     = config->acquire_csi_lltf,
        .acquire_csi_ht20       = config->acquire_csi_ht20,
        .acquire_csi_ht40       = config->acquire_csi_ht40,
        .acquire_csi_su         = config->acquire_csi_su,
        .acquire_csi_mu         = config->acquire_csi_mu,
        .acquire_csi_dcm        = config->acquire_csi_dcm,
        .acquire_csi_beamformed = config->acquire_csi_beamformed,
        .acquire_csi_he_stbc    = config->acquire_csi_he_stbc_mode,
        .val_scale_cfg          = config->val_scale_cfg,
        .dump_ack_en            = config->dump_ack_en,
        .reserved               = 0
    };
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32
    wifi_csi_config = (wifi_csi_config_t) {
        .acquire_csi_lltf       = config->acquire_csi_lltf,
        .acquire_csi_htltf      = config->acquire_csi_htltf,
        .acquire_csi_stbc_htltf2 = config->acquire_csi_stbc_htltf2,
        .ltf_merge_en           = config->ltf_merge_en,
        .channel_filter_en      = config->channel_filter_en,
        .manu_scale             = config->manu_scale,
        .shift                  = config->shift,
        .dump_ack_en            = config->dump_ack_en,
    };
#else
    ESP_LOGE(TAG, "CSI functionality is not supported for %s.", CONFIG_IDF_TARGET);
    return ESP_ERR_NOT_SUPPORTED;
#endif
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&wifi_csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(esp_radar_csi_rx_cb, config->csi_filtered_cb_ctx));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    memcpy(&s_ctx.csi_config, config, sizeof(esp_radar_csi_config_t));

    return ESP_OK;
}

esp_err_t esp_radar_wifi_init(esp_radar_wifi_config_t *wifi_cfg, esp_radar_espnow_config_t *espnow_cfg)
{
    if (!wifi_cfg || !espnow_cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, wifi_cfg->mac_address));

#if !CONFIG_IDF_TARGET_ESP32C5
    if (wifi_cfg->band_mode == WIFI_BAND_MODE_5G_ONLY) {
        ESP_LOGW(TAG, "%s does not support 5GHz band_mode, forcing to WIFI_BAND_MODE_2G_ONLY", CONFIG_IDF_TARGET);
        wifi_cfg->band_mode = WIFI_BAND_MODE_2G_ONLY;
    }
    if (wifi_cfg->protocols.ghz_5g != 0) {
        ESP_LOGW(TAG, "%s does not support 5GHz protocols, clearing 5GHz protocol configuration", CONFIG_IDF_TARGET);
        wifi_cfg->protocols.ghz_5g = 0;
    }
    if (wifi_cfg->bandwidths.ghz_5g != 0) {
        ESP_LOGW(TAG, "%s does not support 5GHz bandwidths, clearing 5GHz bandwidth configuration", CONFIG_IDF_TARGET);
        wifi_cfg->bandwidths.ghz_5g = 0;
    }
#endif

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_band_mode(wifi_cfg->band_mode));
    ESP_ERROR_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &wifi_cfg->protocols));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &wifi_cfg->bandwidths));
    bool is_20mhz = (wifi_cfg->band_mode == WIFI_BAND_MODE_2G_ONLY && wifi_cfg->bandwidths.ghz_2g == WIFI_BW_HT20) ||
                    (wifi_cfg->band_mode == WIFI_BAND_MODE_5G_ONLY && wifi_cfg->bandwidths.ghz_5g == WIFI_BW_HT20);
#else
    ESP_ERROR_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &wifi_cfg->protocols));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, wifi_cfg->bandwidths.ghz_2g));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(ESP_IF_WIFI_STA, espnow_cfg->rate));
    bool is_20mhz = (wifi_cfg->bandwidths.ghz_2g == WIFI_BW_HT20);
#endif

    if (is_20mhz) {
        if (wifi_cfg->second_chan != WIFI_SECOND_CHAN_NONE) {
            ESP_LOGW(TAG, "20MHz bandwidth configured but secondary channel is set to %d, forcing to NONE (channel: %d)",
                     wifi_cfg->second_chan, wifi_cfg->channel);
        }
        ESP_ERROR_CHECK(esp_wifi_set_channel(wifi_cfg->channel, WIFI_SECOND_CHAN_NONE));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_channel(wifi_cfg->channel, wifi_cfg->second_chan));
    }
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    memcpy(&s_ctx.wifi_config, wifi_cfg, sizeof(esp_radar_wifi_config_t));
    memcpy(&s_ctx.espnow_config, espnow_cfg, sizeof(esp_radar_espnow_config_t));
    return ESP_OK;
}

esp_err_t esp_radar_init(esp_radar_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx.init_flag) {
        ESP_LOGW(TAG, "esp_radar already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    s_ctx.init_flag = true;
    memcpy(&s_ctx.radar_config, config, sizeof(esp_radar_config_t));
    if (s_ctx.radar_config.pca_window_size < RADAR_PCA_WINDOW_DEFAULT) {
        ESP_LOGW(TAG, "pca_window_size < 2, fallback to default: %d", RADAR_PCA_WINDOW_DEFAULT);
        s_ctx.radar_config.pca_window_size = RADAR_PCA_WINDOW_DEFAULT;
    }

#if WIFI_CSI_PHY_GAIN_ENABLE
    esp_radar_reset_rx_gain_baseline();
#endif
    return ESP_OK;
}

esp_err_t esp_radar_deinit()
{
    ESP_ERROR_CHECK(esp_wifi_set_csi(false));
    if (s_radar_calibrate) {
        radar_calibrate_free_entries(s_radar_calibrate);
        RADAR_FREE(s_radar_calibrate);
    }
    s_waveform_wander_last = 0.0f;
    memset(&s_ctx, 0, sizeof(s_ctx));

    return ESP_OK;
}
