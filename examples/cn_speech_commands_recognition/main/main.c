/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_process_sdkconfig.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "esp_board_init.h"
#include "speech_commands_action.h"
#include "model_path.h"
#include "ws2812_control.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include <inttypes.h>
#ifdef CONFIG_BT_ENABLED
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "nvs_flash.h"
#endif
#include "esp_timer.h"

static const char *TAG = "main";

int wakeup_flag = 0;
// int detect_flag = 0;  // Add detect_flag declaration
static esp_afe_sr_iface_t *afe_handle = NULL;
static volatile int task_flag = 0;
srmodel_list_t *models = NULL;

#ifdef CONFIG_BT_ENABLED
// ==================== BLE配置 ====================
#define SCAN_TIME 5          // 扫描时间(秒)
#define SCAN_INTERVAL 1349   // 扫描间隔
#define SCAN_WINDOW 449      // 扫描窗口

// ==================== 协议定义 ====================
// OPPO手表BLE广播协议
#define COMPANY_ID 0xFFFF    // 自定义厂商ID
#define PROTOCOL_VERSION 0x01 // 协议版本

// 命令定义 (与OPPO手表应用保持一致)
#define CMD_ON 0x01          // 开启LED
#define CMD_OFF 0x02         // 关闭LED
#define CMD_TOGGLE 0x03      // 切换LED状态

// ==================== BLE全局变量 ====================
static bool ble_scanning = false;
static unsigned long last_command_time = 0;  // 上次命令时间
static const unsigned long COMMAND_COOLDOWN = 500;  // 命令冷却时间(ms)
#endif

// LED状态定义
typedef enum {
    LED_STATE_WAITING_WAKEUP,  // 等待唤醒词 - 白色
    LED_STATE_WAITING_COMMAND, // 等待指令 - 绿色
    LED_STATE_PROCESSING       // 处理指令 - 根据指令变色
} led_state_t;

static led_state_t current_led_state = LED_STATE_WAITING_WAKEUP;
static led_strip_handle_t global_strip = NULL;

typedef struct {
    esp_afe_sr_data_t *afe_data;
    led_strip_handle_t strip;
} detect_task_args_t;


void feed_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_feed_channel_num(afe_data);
    int feed_channel = esp_get_feed_channel();
    assert(nch == feed_channel);
    int16_t *i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    assert(i2s_buff);

    while (task_flag) {
        esp_get_feed_data(true, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);

        afe_handle->feed(afe_data, i2s_buff);
        
        // Add small delay to prevent AFE ringbuffer overflow
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (i2s_buff) {
        free(i2s_buff);
        i2s_buff = NULL;
    }
    vTaskDelete(NULL);
}

// ==================== 蓝牙命令处理函数 ====================
#ifdef CONFIG_BT_ENABLED
// 函数声明
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void process_ble_advertisement(uint8_t *adv_data, uint8_t adv_data_len);
bool init_bluetooth();

// 蓝牙初始化函数
bool init_bluetooth() {
    ESP_LOGI(TAG, "Initializing Bluetooth...");
    
    // 初始化NVS - 增强错误处理
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS需要擦除，正在擦除...");
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "NVS擦除失败: %s", esp_err_to_name(ret));
            return false;
        }
        ret = nvs_flash_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "NVS重新初始化失败: %s", esp_err_to_name(ret));
            return false;
        }
        ESP_LOGI(TAG, "NVS擦除并重新初始化成功");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS初始化失败: %s", esp_err_to_name(ret));
        // 尝试擦除并重新初始化
        ESP_LOGW(TAG, "尝试擦除NVS并重新初始化...");
        ret = nvs_flash_erase();
        if (ret == ESP_OK) {
            ret = nvs_flash_init();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "NVS擦除重建成功");
            } else {
                ESP_LOGE(TAG, "NVS擦除重建失败: %s", esp_err_to_name(ret));
                return false;
            }
        } else {
            ESP_LOGE(TAG, "NVS擦除失败: %s", esp_err_to_name(ret));
            return false;
        }
    } else {
        ESP_LOGI(TAG, "NVS初始化成功");
    }
    
    // 释放经典蓝牙内存
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to release classic BT memory: %s", esp_err_to_name(ret));
    }
    
    // 初始化蓝牙控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BT controller: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable BT controller: %s", esp_err_to_name(ret));
        esp_bt_controller_deinit();
        return false;
    }
    
    // 初始化蓝牙主机
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bluedroid: %s", esp_err_to_name(ret));
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return false;
    }
    
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable bluedroid: %s", esp_err_to_name(ret));
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return false;
    }
    
    // 注册GAP回调
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GAP callback: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "Bluetooth initialized successfully");
     return true;
 }
 
 void process_ble_command(uint8_t command) {
    switch (command) {
        case CMD_ON:
            ESP_LOGI(TAG, "BLE命令: 开启LED");
            if (global_strip != NULL) {
                set_led_color(global_strip, 0x00FF00);  // 绿色 (GRB格式)
            }
            break;
            
        case CMD_OFF:
            ESP_LOGI(TAG, "BLE命令: 关闭LED");
            if (global_strip != NULL) {
                set_led_color(global_strip, 0x000000);  // 关闭
            }
            break;
            
        case CMD_TOGGLE:
            ESP_LOGI(TAG, "BLE命令: 切换LED状态");
            if (global_strip != NULL) {
                // 简单的切换逻辑，可以根据需要改进
                static bool toggle_state = false;
                toggle_state = !toggle_state;
                set_led_color(global_strip, toggle_state ? 0x0000FF : 0x000000);  // 蓝色或关闭
            }
            break;
            
        default:
            ESP_LOGW(TAG, "未知BLE命令: 0x%02X", command);
            break;
    }
    
    ESP_LOGI(TAG, "BLE命令执行完成");
}

// 串口监视器任务，用于通过串口控制LED颜色
void serial_monitor_task(void *pvParameters) {
    char input_buffer[32];
    int buffer_index = 0;
    
    ESP_LOGI(TAG, "串口监视器已启动，输入格式: led RRGGBB (例如: led FF0000)");
    
    while (1) {
        int c = getchar();
        if (c != EOF) {
            if (c == '\n' || c == '\r') {
                if (buffer_index > 0) {
                    input_buffer[buffer_index] = '\0';
                    
                    // 解析LED命令
                    if (strncmp(input_buffer, "led ", 4) == 0 && strlen(input_buffer) == 10) {
                        char *color_str = input_buffer + 4;
                        uint32_t color = 0;
                        
                        // 解析十六进制颜色值 (RGB格式)
                        if (sscanf(color_str, "%06" PRIX32, &color) == 1) {
                            // 转换RGB到GRB格式
                            uint8_t r = (color >> 16) & 0xFF;
                            uint8_t g = (color >> 8) & 0xFF;
                            uint8_t b = color & 0xFF;
                            uint32_t grb_color = (g << 16) | (r << 8) | b;
                            
                            if (global_strip != NULL) {
                                set_led_color(global_strip, grb_color);
                                ESP_LOGI(TAG, "LED颜色已设置为: R=%02X G=%02X B=%02X (GRB=0x%06" PRIX32 ")", r, g, b, grb_color);
                             } else {
                                 ESP_LOGE(TAG, "LED strip未初始化");
                             }
                         } else {
                             ESP_LOGE(TAG, "无效的颜色格式，请使用: led RRGGBB");
                         }
                     } else {
                         ESP_LOGI(TAG, "可用命令: led RRGGBB (例如: led FF0000 设置红色)");
                     }
                    
                    buffer_index = 0;
                }
            } else if (buffer_index < sizeof(input_buffer) - 1) {
                input_buffer[buffer_index++] = c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== BLE扫描和广播接收功能 ====================
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            ESP_LOGI(TAG, "BLE扫描参数设置完成");
            esp_ble_gap_start_scanning(SCAN_TIME);
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "BLE扫描启动失败");
            } else {
                ESP_LOGI(TAG, "BLE扫描已启动");
                ble_scanning = true;
            }
            break;
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
            switch (scan_result->scan_rst.search_evt) {
                case ESP_GAP_SEARCH_INQ_RES_EVT:
                    // 处理扫描到的设备
                    if (scan_result->scan_rst.adv_data_len > 0) {
                        process_ble_advertisement(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len);
                    }
                    break;
                case ESP_GAP_SEARCH_INQ_CMPL_EVT:
                    ESP_LOGI(TAG, "BLE扫描完成，重新开始扫描");
                    esp_ble_gap_start_scanning(SCAN_TIME);
                    break;
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
                ESP_LOGE(TAG, "BLE扫描停止失败");
            } else {
                ESP_LOGI(TAG, "BLE扫描已停止");
                ble_scanning = false;
            }
            break;
        default:
            break;
    }
}

// 处理BLE广播数据
void process_ble_advertisement(uint8_t *adv_data, uint8_t adv_data_len) {
    if (adv_data_len < 5) {
        return;  // 数据长度不足
    }
    
    // 查找厂商数据
    for (int i = 0; i < adv_data_len - 1; i++) {
        uint8_t length = adv_data[i];
        uint8_t type = adv_data[i + 1];
        
        // 检查是否为厂商数据 (0xFF)
        if (type == 0xFF && length >= 5 && i + length < adv_data_len) {
            uint8_t *manufacturer_data = &adv_data[i + 2];
            
            // 解析厂商ID (小端序)
            uint16_t company_id = manufacturer_data[0] | (manufacturer_data[1] << 8);
            
            // 检查是否为目标厂商ID
            if (company_id == COMPANY_ID && length >= 5) {
                uint8_t version = manufacturer_data[2];
                uint8_t command = manufacturer_data[3];
                uint8_t sequence = manufacturer_data[4];
                
                // 检查协议版本
                if (version == PROTOCOL_VERSION) {
                    // 防止重复处理命令
                    unsigned long current_time = esp_timer_get_time() / 1000;  // 转换为毫秒
                    if (current_time - last_command_time < COMMAND_COOLDOWN) {
                        return;
                    }
                    last_command_time = current_time;
                    
                    ESP_LOGI(TAG, "接收到OPPO手表命令: 厂商ID=0x%04X, 版本=0x%02X, 命令=0x%02X, 序列号=%d", 
                             company_id, version, command, sequence);
                    
                    // 执行命令
                    process_ble_command(command);
                }
            }
            break;
        }
        i += length;  // 跳到下一个AD结构
    }
}
#endif

void detect_Task(void *arg)
{
    detect_task_args_t *task_args = (detect_task_args_t *)arg;
    esp_afe_sr_data_t *afe_data = task_args->afe_data;
    led_strip_handle_t strip = task_args->strip;
    
    // 添加任务到看门狗
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "detect_Task已添加到看门狗");
    
    // 检查可用内存
    size_t free_heap = esp_get_free_heap_size();
    size_t min_free_heap = esp_get_minimum_free_heap_size();
    ESP_LOGI(TAG, "detect_Task启动 - 可用堆内存: %zu bytes, 最小可用: %zu bytes", free_heap, min_free_heap);
    
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    ESP_LOGI(TAG, "初始化语音识别模型: %s", mn_name);
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *model_data = multinet->create(mn_name, 6000);
    
    esp_mn_error_t *update_error = esp_mn_commands_update_from_sdkconfig(multinet, model_data);
    if (update_error == NULL) {
        ESP_LOGI(TAG, "手动添加语音命令");
        
        // Manually add custom commands since MULTINET7_QUANT doesn't support sdkconfig loading
        esp_mn_commands_alloc(multinet, model_data);
        esp_mn_commands_add(0, "bang wo guan deng");
        esp_mn_commands_add(1, "bang wo kai deng");
        esp_mn_commands_add(2, "da kai dian deng");
        esp_mn_commands_add(3, "kai xiang");
        esp_mn_commands_add(4, "guan bi dian deng");
        
        esp_mn_commands_update();
        ESP_LOGI(TAG, "语音命令添加完成");
    }
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    assert(mu_chunksize == afe_chunksize);

    // 打印活跃的语音命令
    multinet->print_active_speech_commands(model_data);
    ESP_LOGI(TAG, "语音检测任务启动");
    
    uint32_t loop_count = 0;
    while (task_flag) {
        // 每次循环都重置看门狗，防止afe_fetch阻塞导致看门狗超时
        esp_task_wdt_reset();
        
        // 每1000次循环检查内存
        if (++loop_count % 1000 == 0) {
            size_t current_free = esp_get_free_heap_size();
            if (current_free < 50000) {  // 如果可用内存少于50KB，记录警告
                ESP_LOGW(TAG, "内存不足警告: %zu bytes", current_free);
            }
        }
        
        afe_fetch_result_t* res = afe_handle->fetch(afe_data); 
        if (!res || res->ret_value == ESP_FAIL) {
            ESP_LOGE(TAG, "音频数据获取失败");
            esp_task_wdt_reset();  // 重置看门狗
            vTaskDelay(pdMS_TO_TICKS(10));  // 短暂延迟后重试
            continue;  // 继续循环而不是退出
        }

        if (res->wakeup_state == WAKENET_DETECTED) {
            ESP_LOGI(TAG, "检测到唤醒词");
	        multinet->clean(model_data);
        }

        if (res->raw_data_channels == 1 && res->wakeup_state == WAKENET_DETECTED) {
            wakeup_flag = 1;
            
            // 检测到唤醒词，切换到等待指令状态，显示绿色
            current_led_state = LED_STATE_WAITING_COMMAND;
            set_led_color(strip, 0xFF0000);  // 绿色 (GRB格式)
            ESP_LOGI(TAG, "等待语音指令");
        } else if (res->raw_data_channels > 1 && res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
            // For a multi-channel AFE, it is necessary to wait for the channel to be verified.
            ESP_LOGI(TAG, "多通道验证完成，通道: %d", res->trigger_channel_id);
            wakeup_flag = 1;
            
            // 检测到唤醒词，切换到等待指令状态，显示绿色
            current_led_state = LED_STATE_WAITING_COMMAND;
            set_led_color(strip, 0xFF0000);  // 绿色 (GRB格式)
            ESP_LOGI(TAG, "等待语音指令");
        }

        if (wakeup_flag == 1) {
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

            if (mn_state == ESP_MN_STATE_DETECTING) {
                continue;
            }

            if (mn_state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                for (int i = 0; i < mn_result->num; i++) {
                    ESP_LOGI(TAG, "TOP %d, command_id: %d, phrase_id: %d, string:%s prob: %f", 
                    i+1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->string, mn_result->prob[i]);
                }
                
                ESP_LOGI(TAG, "检测到语音命令 ID: %d, 置信度: %.2f", mn_result->command_id[0], mn_result->prob[0]);
                
                // 检测到语音命令，切换到执行状态，显示蓝色
                current_led_state = LED_STATE_PROCESSING;
                set_led_color(strip, 0x0000FF);  // 蓝色 (GRB格式)
                
                // 处理语音指令
                speech_commands_action(mn_result->command_id[0], strip);
                
                // 指令处理完成，返回等待唤醒词状态
                vTaskDelay(pdMS_TO_TICKS(2000));  // 保持指令颜色2秒
                current_led_state = LED_STATE_WAITING_WAKEUP;
                set_led_color(strip, 0xFFFFFF);  // 返回白色
                ESP_LOGI(TAG, "指令处理完成，LED返回白色，等待唤醒词");
                wakeup_flag = 0;

                ESP_LOGI(TAG, "-----------listening-----------");
            }

            if (mn_state == ESP_MN_STATE_TIMEOUT) {
                afe_handle->enable_wakenet(afe_data);
                wakeup_flag = 0;
                
                // 超时，返回等待唤醒词状态
                current_led_state = LED_STATE_WAITING_WAKEUP;
                set_led_color(strip, 0xFFFFFF);  // 返回白色
                ESP_LOGD(TAG, "语音识别超时，等待唤醒词");
                continue;
            }
        }
    }
    
    // 清理资源
    if (model_data) {
        multinet->destroy(model_data);
        model_data = NULL;
    }
    
    // 从看门狗中移除任务
    esp_task_wdt_delete(NULL);
    ESP_LOGI(TAG, "语音检测任务退出，已从看门狗移除");
    vTaskDelete(NULL);
}

void app_main()
{
    ESP_LOGI(TAG, "应用程序启动");
    
    // 设置日志级别，减少I2C错误输出
    esp_log_level_set("I2C_If", ESP_LOG_WARN);
    esp_log_level_set("ES7210", ESP_LOG_WARN);
    esp_log_level_set("ES8311", ESP_LOG_WARN);
    
    models = esp_srmodel_init("model");
    
    // 尝试初始化音频板，如果失败则使用默认配置
    esp_err_t board_init_ret = esp_board_init(16000, 1, 16);
    if (board_init_ret != ESP_OK) {
        ESP_LOGW(TAG, "音频板初始化失败，可能缺少硬件支持，但程序继续运行");
        ESP_LOGW(TAG, "某些音频功能可能不可用");
    } else {
        ESP_LOGI(TAG, "音频板初始化成功");
    }

    afe_config_t *afe_config = afe_config_init(esp_get_input_format(), models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    afe_handle = esp_afe_handle_from_config(afe_config);
    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(afe_config);
    afe_config_free(afe_config);

    task_flag = 1;
    led_strip_handle_t strip = init_ws2812(GPIO_NUM_48);
    global_strip = strip;  // 保存全局引用
    
    // 初始状态：等待唤醒词，显示白色
    current_led_state = LED_STATE_WAITING_WAKEUP;
    set_led_color(strip, 0xFFFFFF); // 初始状态：白色，等待唤醒词 (GRB格式)
    ESP_LOGI(TAG, "Ready - LED显示白色，等待唤醒词");

    detect_task_args_t *task_args = (detect_task_args_t *)malloc(sizeof(detect_task_args_t));
    task_args->afe_data = afe_data;
    task_args->strip = strip;

#ifdef CONFIG_BT_ENABLED
    // ==================== 蓝牙初始化 ====================
    if (!init_bluetooth()) {
        ESP_LOGE(TAG, "蓝牙初始化失败，但程序继续运行");
    } else {
        // 设置简化的扫描参数
        esp_ble_scan_params_t ble_scan_params = {
            .scan_type              = BLE_SCAN_TYPE_PASSIVE,  // 使用被动扫描减少功耗
            .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
            .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
            .scan_interval          = 0x80,  // 增加扫描间隔
            .scan_window            = 0x50,    // 增加扫描窗口
            .scan_duplicate         = BLE_SCAN_DUPLICATE_ENABLE  // 启用重复过滤
        };
        
        esp_err_t ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "设置扫描参数失败: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "蓝牙扫描功能不可用，但程序继续运行");
        } else {
            ESP_LOGI(TAG, "蓝牙初始化完成，开始扫描...");
            
            // 开始扫描
            ret = esp_ble_gap_start_scanning(SCAN_TIME);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "开始扫描失败: %s", esp_err_to_name(ret));
                ESP_LOGW(TAG, "蓝牙扫描启动失败，但程序继续运行");
            }
        }
    }
#else
    ESP_LOGI(TAG, "蓝牙功能已禁用，仅启用语音识别功能");
#endif
    
    // 增加任务栈大小以防止栈溢出导致的系统重启
    xTaskCreatePinnedToCore(&detect_Task, "detect", 12 * 1024, (void*)task_args, 5, NULL, 1);
    xTaskCreatePinnedToCore(&feed_Task, "feed", 10 * 1024, (void*)afe_data, 5, NULL, 0);
    xTaskCreatePinnedToCore(&serial_monitor_task, "serial_monitor", 6 * 1024, NULL, 3, NULL, 0);
    
    ESP_LOGI(TAG, "所有任务已创建，系统运行中...");
    
    // Keep main task alive
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
