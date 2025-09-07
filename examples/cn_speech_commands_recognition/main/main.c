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
#include <inttypes.h>

static const char *TAG = "main";

int wakeup_flag = 0;
// int detect_flag = 0;  // Add detect_flag declaration
static esp_afe_sr_iface_t *afe_handle = NULL;
static volatile int task_flag = 0;
srmodel_list_t *models = NULL;

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

void detect_Task(void *arg)
{
    detect_task_args_t *task_args = (detect_task_args_t *)arg;
    esp_afe_sr_data_t *afe_data = task_args->afe_data;
    led_strip_handle_t strip = task_args->strip;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    printf("multinet:%s\n", mn_name);
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *model_data = multinet->create(mn_name, 6000);
    printf("Before esp_mn_commands_update_from_sdkconfig\n");
    esp_mn_error_t *update_error = esp_mn_commands_update_from_sdkconfig(multinet, model_data); // Add speech commands from sdkconfig
    if (update_error == NULL) {
        printf("esp_mn_commands_update_from_sdkconfig returned NULL - MULTINET7_QUANT does not support sdkconfig commands\n");
        printf("Manually adding custom commands...\n");
        
        // Manually add custom commands since MULTINET7_QUANT doesn't support sdkconfig loading
        esp_mn_commands_alloc(multinet, model_data);
        esp_mn_commands_add(0, "bang wo guan deng");
        esp_mn_commands_add(1, "bang wo kai deng");
        esp_mn_commands_add(2, "da kai dian deng");
        esp_mn_commands_add(3, "kai xiang");  // Add the custom command
        esp_mn_commands_add(4, "guan bi dian deng");
        
        printf("Custom commands added manually\n");
        esp_mn_commands_update();
    } else {
        printf("esp_mn_commands_update_from_sdkconfig completed with errors\n");
    }
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    assert(mu_chunksize == afe_chunksize);

    //print active speech commands
    multinet->print_active_speech_commands(model_data);
    printf("------------detect start------------\n");
    while (task_flag) {
        afe_fetch_result_t* res = afe_handle->fetch(afe_data); 
        if (!res || res->ret_value == ESP_FAIL) {
            printf("fetch error!\n");
            break;
        }

        if (res->wakeup_state == WAKENET_DETECTED) {
            printf("WAKEWORD DETECTED\n");
	        multinet->clean(model_data);
        }

        if (res->raw_data_channels == 1 && res->wakeup_state == WAKENET_DETECTED) {
            wakeup_flag = 1;
            // detect_flag = 0; // Reset detect_flag on wake word
            printf("Wakeup flag set to 1 (WAKENET_DETECTED)\n");
            
            // 检测到唤醒词，切换到等待指令状态，显示绿色
            current_led_state = LED_STATE_WAITING_COMMAND;
            set_led_color(strip, 0xFF0000);  // 绿色 (GRB格式)
            printf("LED显示绿色，等待语音指令\n");
        } else if (res->raw_data_channels > 1 && res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
            // For a multi-channel AFE, it is necessary to wait for the channel to be verified.
            printf("AFE_FETCH_CHANNEL_VERIFIED, channel index: %d\n", res->trigger_channel_id);
            wakeup_flag = 1;
            // detect_flag = 0; // Reset detect_flag on wake word
            printf("Wakeup flag set to 1 (WAKENET_CHANNEL_VERIFIED)\n");
            
            // 检测到唤醒词，切换到等待指令状态，显示绿色
            current_led_state = LED_STATE_WAITING_COMMAND;
            set_led_color(strip, 0xFF0000);  // 绿色 (GRB格式)
            printf("LED显示绿色，等待语音指令\n");
        }

        if (wakeup_flag == 1) {
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

            if (mn_state == ESP_MN_STATE_DETECTING) {
                continue;
            }

            if (mn_state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                for (int i = 0; i < mn_result->num; i++) {
                    printf("TOP %d, command_id: %d, phrase_id: %d, string:%s prob: %f\n", 
                    i+1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->string, mn_result->prob[i]);
                }
                // 处理语音指令
                current_led_state = LED_STATE_PROCESSING;
                speech_commands_action(mn_result->command_id[0], strip);
                
                // 指令处理完成，返回等待唤醒词状态
                vTaskDelay(pdMS_TO_TICKS(2000));  // 保持指令颜色2秒
                current_led_state = LED_STATE_WAITING_WAKEUP;
                set_led_color(strip, 0xFFFFFF);  // 返回白色
                printf("指令处理完成，LED返回白色，等待唤醒词\n");
                wakeup_flag = 0;

                printf("\n-----------listening-----------\n");
            }

            if (mn_state == ESP_MN_STATE_TIMEOUT) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                printf("timeout, string:%s\n", mn_result->string);
                afe_handle->enable_wakenet(afe_data);
                wakeup_flag = 0;
                // detect_flag = 0; // Reset detect_flag on timeout
                
                // 超时，返回等待唤醒词状态
                current_led_state = LED_STATE_WAITING_WAKEUP;
                set_led_color(strip, 0xFFFFFF);  // 返回白色
                printf("\n-----------awaits to be waken up-----------\n");
                continue;
            }
        }
    }
    if (model_data) {
        multinet->destroy(model_data);
        model_data = NULL;
    }
    printf("detect exit\n");
    vTaskDelete(NULL);
}

void app_main()
{
    models = esp_srmodel_init("model");
    ESP_ERROR_CHECK(esp_board_init(16000, 1, 16));

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

    xTaskCreatePinnedToCore(&detect_Task, "detect", 8 * 1024, (void*)task_args, 5, NULL, 1);
    xTaskCreatePinnedToCore(&feed_Task, "feed", 8 * 1024, (void*)afe_data, 5, NULL, 0);
    xTaskCreatePinnedToCore(&serial_monitor_task, "serial_monitor", 4 * 1024, NULL, 3, NULL, 0);
    
    // Keep main task alive
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
