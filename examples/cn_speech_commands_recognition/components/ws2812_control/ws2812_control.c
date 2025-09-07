#include "ws2812_control.h"
#include "led_strip.h"

static const char *TAG = "WS2812_CONTROL";

led_strip_handle_t init_ws2812(gpio_num_t gpio_num) {
    led_strip_handle_t led_strip;
    led_strip_config_t strip_config = {
        .strip_gpio_num = gpio_num,
        .max_leds = 1, // 只有一个LED
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz

    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "WS2812 LED initialized on GPIO%d", gpio_num);
    return led_strip;
}

void set_led_color(led_strip_handle_t strip, uint32_t color) {
    uint32_t red = (color >> 16) & 0xFF;
    uint32_t green = (color >> 8) & 0xFF;
    uint32_t blue = color & 0xFF;
    // Note: LED_PIXEL_FORMAT_GRB means the order is Green-Red-Blue
    // So we need to pass green, red, blue in that order
    ESP_ERROR_CHECK(led_strip_set_pixel(strip, 0, green, red, blue));
    ESP_ERROR_CHECK(led_strip_refresh(strip));
    ESP_LOGI(TAG, "LED color set to R:%d G:%d B:%d", red, green, blue);
}

void turn_off_led(led_strip_handle_t strip) {
    ESP_ERROR_CHECK(led_strip_clear(strip));
    ESP_LOGI(TAG, "LED turned off");
}