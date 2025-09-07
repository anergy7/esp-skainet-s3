/**
 * 
 * @copyright Copyright 2021 Espressif Systems (Shanghai) Co. Ltd.
 *
 *      Licensed under the Apache License, Version 2.0 (the "License");
 *      you may not use this file except in compliance with the License.
 *      You may obtain a copy of the License at
 *
 *               http://www.apache.org/licenses/LICENSE-2.0
 *
 *      Unless required by applicable law or agreed to in writing, software
 *      distributed under the License is distributed on an "AS IS" BASIS,
 *      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *      See the License for the specific language governing permissions and
 *      limitations under the License.
 *
 * 此文件定义了ESP32-S3-KORVO-1-V4.0开发板的硬件配置，包括各种接口的GPIO引脚定义
 */
#pragma once

#include <driver/gpio.h>
#include <esp_idf_version.h>
#include <esp_codec_dev.h>
#include <esp_codec_dev_defaults.h>
#include <esp_codec_dev_os.h>

/**
 * @brief ESP32-S3-KORVO-1-V4.0 I2C GPIO defineation
 * 
 * @note ESP32-S3-KORVO-1-V4.0开发板的I2C接口GPIO定义
 * @note I2C时钟频率为600kHz，SCL引脚为GPIO2，SDA引脚为GPIO1
 */
#define FUNC_I2C_EN     (1)
#define I2C_NUM         (0)
#define I2C_CLK         (600000)
#define GPIO_I2C_SCL    (GPIO_NUM_2)
#define GPIO_I2C_SDA    (GPIO_NUM_1)

/**
 * @brief ESP32-S3-KORVO-1-V4.0 SDMMC GPIO defination
 * 
 * @note Only avaliable when PMOD connected
 * @note ESP32-S3-KORVO-1-V4.0开发板的SD卡接口GPIO定义
 * @note 仅在连接PMOD扩展模块时可用
 * @note 使用1线模式，时钟引脚为GPIO18，命令引脚为GPIO17，数据引脚D0为GPIO16，D3为GPIO15
 */
#define FUNC_SDMMC_EN   (1)
#define SDMMC_BUS_WIDTH (1)
#define GPIO_SDMMC_CLK  (GPIO_NUM_18)
#define GPIO_SDMMC_CMD  (GPIO_NUM_17)
#define GPIO_SDMMC_D0   (GPIO_NUM_16)
#define GPIO_SDMMC_D1   (GPIO_NUM_NC)
#define GPIO_SDMMC_D2   (GPIO_NUM_NC)
#define GPIO_SDMMC_D3   (GPIO_NUM_15)
#define GPIO_SDMMC_DET  (GPIO_NUM_NC)

/**
 * @brief ESP32-S3-KORVO-1-V4.0 SDSPI GPIO definationv
 * 
 * @note ESP32-S3-KORVO-1-V4.0开发板的SPI接口SD卡GPIO定义
 * @note 此功能默认禁用(FUNC_SDSPI_EN=0)，所有引脚均设为NC(未连接)
 */
#define FUNC_SDSPI_EN       (0)
#define SDSPI_HOST          (SPI2_HOST)
#define GPIO_SDSPI_CS       (GPIO_NUM_NC)
#define GPIO_SDSPI_SCLK     (GPIO_NUM_NC)
#define GPIO_SDSPI_MISO     (GPIO_NUM_NC)
#define GPIO_SDSPI_MOSI     (GPIO_NUM_NC)

/**
 * @brief ESP32-S3-KORVO-1-V4.0 I2S GPIO defination
 * 
 * @note ESP32-S3-KORVO-1-V4.0开发板的I2S接口GPIO定义（用于麦克风输入）
 * @note LRCK(帧同步/字时钟)引脚为GPIO9
 * @note MCLK(主时钟)引脚为GPIO20
 * @note SCLK(串行时钟)引脚为GPIO10
 * @note SDIN(数据输入)引脚为GPIO11，用于从麦克风接收音频数据
 * @note DOUT(数据输出)未连接(NC)
 */
#define FUNC_I2S_EN         (1)
#define GPIO_I2S_LRCK       (GPIO_NUM_9)
#define GPIO_I2S_MCLK       (GPIO_NUM_20)
#define GPIO_I2S_SCLK       (GPIO_NUM_10)
#define GPIO_I2S_SDIN       (GPIO_NUM_11)
#define GPIO_I2S_DOUT       (GPIO_NUM_NC)

/**
 * @brief ESP32-S3-KORVO-1-V4.0 I2S GPIO defination
 * 
 * @note ESP32-S3-KORVO-1-V4.0开发板的I2S0接口GPIO定义（用于扬声器输出）
 * @note LRCK(帧同步/字时钟)引脚为GPIO41
 * @note MCLK(主时钟)引脚为GPIO42
 * @note SCLK(串行时钟)引脚为GPIO40
 * @note SDIN(数据输入)未连接(NC)
 * @note DOUT(数据输出)引脚为GPIO39，用于向扬声器发送音频数据
 */
#define FUNC_I2S0_EN         (1)
#define GPIO_I2S0_LRCK       (GPIO_NUM_41)
#define GPIO_I2S0_MCLK       (GPIO_NUM_42)
#define GPIO_I2S0_SCLK       (GPIO_NUM_40)
#define GPIO_I2S0_SDIN       (GPIO_NUM_NC)
#define GPIO_I2S0_DOUT       (GPIO_NUM_39)

#define RECORD_VOLUME   (30.0)  // 录音音量设置，默认为30.0
/**
 * @brief player configurations
 *
 * @note 播放器配置参数
 */
#define PLAYER_VOLUME   (50)  // 播放音量设置，默认为50

/**
 * @brief ESP32-S3-HMI-DevKit power control IO
 * 
 * @note Some power control pins might not be listed yet
 * 
 * @note ESP32-S3-KORVO-1-V4.0开发板的电源控制IO
 * @note 部分电源控制引脚可能尚未列出
 * @note 电源控制引脚为GPIO38，高电平(1)开启电源
 */
#define FUNC_PWR_CTRL       (1)
#define GPIO_PWR_CTRL       (GPIO_NUM_38)
#define GPIO_PWR_ON_LEVEL   (1)

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)

// ESP-IDF 5.0及以上版本的I2S0默认配置（用于扬声器输出）
#define I2S0_CONFIG_DEFAULT(sample_rate, channel_fmt, bits_per_chan) { \
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate), \
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bits_per_chan, channel_fmt), \
        .gpio_cfg = { \
            .mclk = GPIO_I2S0_MCLK, \
            .bclk = GPIO_I2S0_SCLK, \
            .ws   = GPIO_I2S0_LRCK, \
            .dout = GPIO_I2S0_DOUT, \
            .din  = GPIO_I2S0_SDIN, \
            .invert_flags = { \
                .mclk_inv = false, \
                .bclk_inv = false, \
                .ws_inv   = false, \
            }, \
        }, \
    }

// ESP-IDF 5.0及以上版本的I2S默认配置（用于麦克风输入）
// ESP-IDF 5.0以下版本的I2S默认配置（用于麦克风输入）
#define I2S_CONFIG_DEFAULT(sample_rate, channel_fmt, bits_per_chan) { \
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000), \
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(32, I2S_SLOT_MODE_STEREO), \
        .gpio_cfg = { \
            .mclk = GPIO_I2S_MCLK, \
            .bclk = GPIO_I2S_SCLK, \
            .ws   = GPIO_I2S_LRCK, \
            .dout = GPIO_I2S_DOUT, \
            .din  = GPIO_I2S_SDIN, \
        }, \
    }

#else

// ESP-IDF 5.0以下版本的I2S0默认配置（用于扬声器输出）
#define I2S0_CONFIG_DEFAULT(sample_rate, channel_fmt, bits_per_chan) { \
    .mode                   = I2S_MODE_MASTER | I2S_MODE_TX, \
    .sample_rate            = sample_rate, \
    .bits_per_sample        = I2S_BITS_PER_SAMPLE_16BIT, \
    .channel_format         = channel_fmt, \
    .communication_format   = I2S_COMM_FORMAT_STAND_I2S, \
    .intr_alloc_flags       = ESP_INTR_FLAG_LEVEL1, \
    .dma_buf_count          = 6, \
    .dma_buf_len            = 160, \
    .use_apll               = false, \
    .tx_desc_auto_clear     = true, \
    .fixed_mclk             = 0, \
    .mclk_multiple          = I2S_MCLK_MULTIPLE_DEFAULT, \
    .bits_per_chan          = bits_per_chan, \
}

#define I2S_CONFIG_DEFAULT(sample_rate, channel_fmt, bits_per_chan) { \
    .mode                   = I2S_MODE_MASTER | I2S_MODE_RX, \
    .sample_rate            = 16000, \
    .bits_per_sample        = I2S_BITS_PER_SAMPLE_32BIT, \
    .channel_format         = I2S_CHANNEL_FMT_RIGHT_LEFT, \
    .communication_format   = I2S_COMM_FORMAT_STAND_I2S, \
    .intr_alloc_flags       = ESP_INTR_FLAG_LEVEL1, \
    .dma_buf_count          = 6, \
    .dma_buf_len            = 160, \
    .use_apll               = false, \
    .tx_desc_auto_clear     = true, \
    .fixed_mclk             = 0, \
    .mclk_multiple          = I2S_MCLK_MULTIPLE_DEFAULT, \
    .bits_per_chan          = I2S_BITS_PER_CHAN_32BIT, \
}

#endif