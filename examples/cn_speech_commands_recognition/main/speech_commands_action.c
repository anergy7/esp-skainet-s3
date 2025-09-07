/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// #include "ie_kaiji.h"
#include "m_0.h"
#include "m_1.h"
#include "m_2.h"
#include "m_3.h"
#include "m_4.h"
#include "m_5.h"
#include "m_6.h"
#include "m_7.h"
#include "m_8.h"
#include "m_9.h"
#include "m_10.h"
#include "m_11.h"
#include "m_12.h"
#include "m_13.h"
#include "m_14.h"
#include "m_15.h"
#include "m_16.h"
#include "m_17.h"
#include "esp_board_init.h"
#include "wake_up_prompt_tone.h"
#include "speech_commands_action.h"
#include "led_strip.h"

extern int detect_flag;
led_strip_handle_t strip = NULL;

typedef struct {
    char* name;
    const uint16_t* data;
    int length;
} dac_audio_item_t;

// LED control for ESP32-S3 and other boards
#define EXAMPLE_CHASE_SPEED_MS (10)

// LED_BUILTIN equivalent for ESP32-S3-N16R8 (try both GPIO38 and GPIO48)
#define LED_GPIO 2  // GPIO for external HW-826 LED

void led_Task(void *arg)
{
#if defined CONFIG_ESP32_S3_KORVO_1_V4_0_BOARD
    // ESP32-S3 Korvo board with WS2812 LED strip
    const led_strip_config_t led_config = {
        .strip_gpio_num = 19,
        .max_leds = 12,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
    };
    const led_strip_rmt_config_t rmt_config = {}; // default
    led_strip_new_rmt_device(&led_config, &rmt_config, &strip);
    if (!strip) {
        printf("install WS2812 driver failed\n");
        vTaskDelete(NULL);
        return;
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(led_strip_clear(strip));
    
    while (1) {
        if (detect_flag) {
            // Light up all LEDs when speech command detected
            for (int j = 0; j < 12; j++) {
                ESP_ERROR_CHECK(led_strip_set_pixel(strip, j, 255, 0, 0)); // Red color
            }
            ESP_ERROR_CHECK(led_strip_refresh(strip));
        } else {
            // Turn off all LEDs
            ESP_ERROR_CHECK(led_strip_clear(strip));
            ESP_ERROR_CHECK(led_strip_refresh(strip));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
#else
    // ESP32-S3-N16R8 and other generic ESP32-S3 boards with single LED
    // Configure both GPIO38 and GPIO48 as outputs (ESP32-S3-N16R8 LED_BUILTIN equivalent)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    
    printf("[LED] Configured GPIO%d as LED output for external HW-826\n", LED_GPIO);
    
    while (1) {
        if (detect_flag) {
            // Turn on LED
            gpio_set_level(LED_GPIO, 1);
            printf("[LED] External LED turned ON (GPIO%d)\n", LED_GPIO);
        } else {
            // Turn off LED
            gpio_set_level(LED_GPIO, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
#endif
}

dac_audio_item_t playlist[] = {
    // {"ie_kaiji.h", ie_kaiji, sizeof(ie_kaiji)},
    {"wake_up_prompt_tone.h", (uint16_t*)wake_up_prompt_tone, sizeof(wake_up_prompt_tone)},
    {"m_1.h", (uint16_t*)m_1, sizeof(m_1)},
    {"m_2.h", (uint16_t*)m_2, sizeof(m_2)},
    {"m_3.h", (uint16_t*)m_3, sizeof(m_3)},
    {"m_4.h", (uint16_t*)m_4, sizeof(m_4)},
    {"m_5.h", (uint16_t*)m_5, sizeof(m_5)},
    {"m_6.h", (uint16_t*)m_6, sizeof(m_6)},
    {"m_7.h", (uint16_t*)m_7, sizeof(m_7)},
    {"m_8.h", (uint16_t*)m_8, sizeof(m_8)},
    {"m_9.h", (uint16_t*)m_9, sizeof(m_9)},
    {"m_10.h", (uint16_t*)m_10, sizeof(m_10)},
    {"m_11.h", (uint16_t*)m_11, sizeof(m_11)},
    {"m_12.h", (uint16_t*)m_12, sizeof(m_12)},
    {"m_13.h", (uint16_t*)m_13, sizeof(m_13)},
    {"m_14.h", (uint16_t*)m_14, sizeof(m_14)},
    {"m_15.h", (uint16_t*)m_15, sizeof(m_15)},
    {"m_16.h", (uint16_t*)m_16, sizeof(m_16)},
    {"m_17.h", (uint16_t*)m_17, sizeof(m_17)},
};

void wake_up_action(void)
{
    esp_audio_play((int16_t *)(playlist[0].data), playlist[0].length, portMAX_DELAY);
}

void speech_commands_action(int command_id)
{
    printf("[DEBUG] Speech command detected, command_id: %d\n", command_id);
    
    // Set detect_flag to trigger LED lighting when any speech command is recognized
    detect_flag = 1;
    printf("[DEBUG] LED detect_flag set to 1 - LED should turn ON now\n");
    
    // Handle specific commands
    switch (command_id) {
        case 0:
            printf("[ACTION] Command: bang wo guan deng (Turn off light)\n");
            break;
        case 1:
            printf("[ACTION] Command: bang wo kai deng (Turn on light)\n");
            break;
        case 2:
            printf("[ACTION] Command: da kai dian deng (Open electric light)\n");
            break;
        case 3:
            printf("[ACTION] Command: kai xiang (Custom command - Open box/Start)\n");
            break;
        case 4:
            printf("[ACTION] Command: guan bi dian deng (Close electric light)\n");
            break;
        default:
            printf("[ACTION] Unknown command_id: %d\n", command_id);
            break;
    }
    
    // Play audio if command_id is within valid range
    int playlist_size = sizeof(playlist) / sizeof(playlist[0]);
    if (command_id + 1 < playlist_size) {
        esp_audio_play((int16_t *)(playlist[command_id + 1].data), playlist[command_id + 1].length, portMAX_DELAY);
    } else {
        printf("[WARNING] No audio file for command_id %d (playlist size: %d)\n", command_id, playlist_size);
        // Play a default sound for custom commands
        esp_audio_play((int16_t *)(playlist[1].data), playlist[1].length, portMAX_DELAY);
    }
    
    // Keep LED on for 3 seconds after command recognition
    printf("[DEBUG] Keeping LED on for 3 seconds...\n");
    vTaskDelay(pdMS_TO_TICKS(3000));
    detect_flag = 0;
    printf("[DEBUG] LED detect_flag set to 0 - LED should turn OFF now\n");
    printf("[INFO] ESP32-S3-N16R8 LED control: GPIO%d=%s\n", 
           LED_GPIO, gpio_get_level(LED_GPIO) ? "HIGH" : "LOW");
}