#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ws2812_control.h"






void app_main() {
    // 初始化WS2812 LED
    led_strip_handle_t strip = init_ws2812(GPIO_NUM_48);
    
    // 定义颜色数组
    uint32_t colors[] = {
        0xFF0000, // 红色
        0x00FF00, // 绿色
        0x0000FF, // 蓝色
        0xFFFF00, // 黄色
        0xFF00FF, // 紫色
        0x00FFFF  // 青色
    };
    int color_count = sizeof(colors)/sizeof(colors[0]);
    
    while(1) {
        // 循环显示所有颜色
        for(int i = 0; i < color_count; i++) {
            set_led_color(strip, colors[i]);
            vTaskDelay(pdMS_TO_TICKS(500)); // 每个颜色显示0.5秒
        }
    }
}