#ifndef WS2812_CONTROL_H
#define WS2812_CONTROL_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"

#ifdef __cplusplus
extern "C" {
#endif

led_strip_handle_t init_ws2812(gpio_num_t gpio_num);
void set_led_color(led_strip_handle_t strip, uint32_t color);
void turn_off_led(led_strip_handle_t strip);

#ifdef __cplusplus
}
#endif

#endif // WS2812_CONTROL_H