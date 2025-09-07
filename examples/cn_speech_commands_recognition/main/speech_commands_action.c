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
#include "ws2812_control.h"


typedef struct {
    char* name;
    const uint16_t* data;
    int length;
} dac_audio_item_t;

// LED control for ESP32-S3 and other boards
#define EXAMPLE_CHASE_SPEED_MS (10)

// LED_BUILTIN equivalent for ESP32-S3-N16R8 (try both GPIO38 and GPIO48)
#define LED_GPIO 2  // GPIO for external HW-826 LED

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

// New task to restore LED color after a delay
void restore_led_color_task(void *pvParameters) {
    led_strip_handle_t strip = (led_strip_handle_t)pvParameters;
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 2 seconds
    set_led_color(strip, 0xFF0000); // Restore to green (GRB format)
    vTaskDelete(NULL); // Delete the task after execution
}

void wake_up_action(void)
{
    esp_audio_play((int16_t *)(playlist[0].data), playlist[0].length, portMAX_DELAY);
}

void speech_commands_action(int command_id, led_strip_handle_t strip)
{
    printf("[DEBUG] Speech command detected, command_id: %d\n", command_id);
    
    // Set detect_flag to trigger LED lighting when any speech command is recognized
    // int detect_flag = 1; // Moved to local if needed, but not used for direct LED control here
    printf("[DEBUG] Speech command detected, command_id: %d\n", command_id);
    
    // Handle specific commands
    switch (command_id) {
        case 0:
            printf("[ACTION] Command: bang wo guan deng (Turn off light)\n");
            set_led_color(strip, 0x000000); // Turn off LED
            break;
        case 1:
            printf("[ACTION] Command: bang wo kai deng (Turn on light)\n");
            set_led_color(strip, 0xFF0000); // Green (GRB format)
            break;
        case 2:
            printf("[ACTION] Command: da kai dian deng (Open electric light)\n");
            set_led_color(strip, 0xFF0000); // Green (GRB format)
            break;
        case 3:
            printf("[ACTION] Command: kai xiang (Custom command - Open box/Start)\n");
            set_led_color(strip, 0x0000FF); // Red (GRB format)
            // Create a task to restore LED color after 2 seconds
            xTaskCreate(restore_led_color_task, "restore_led_color_task", 2048, (void *)strip, 5, NULL);
            break;
        case 4:
            printf("[ACTION] Command: guan bi dian deng (Close electric light)\n");
            set_led_color(strip, 0x000000); // Turn off LED
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
    
    // Removed delay and detect_flag reset to allow immediate color changes
    // printf("[INFO] ESP32-S3-N16R8 LED control: GPIO%d=%s\n", 
    //        LED_GPIO, gpio_get_level(LED_GPIO) ? "HIGH" : "LOW");
}