#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_PORT      UART_NUM_1
#define UART_TX_PIN    17
#define UART_RX_PIN    18
#define UART_BAUD      9600
#define UART_BUF_SIZE  256
#define BUTTON_PIN     GPIO_NUM_15
#define LED_PIN        GPIO_NUM_6
#define DEBOUNCE_MS    30
#define POLL_MS        10

#define ESP_BUTTON_COMMAND 'A'
#define STM_BUTTON_COMMAND 'B'

static const char *TAG = "UART_LINK";

static void uart_link_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,     // 8 біт даних
        .parity     = UART_PARITY_DISABLE,  // N — без парності
        .stop_bits  = UART_STOP_BITS_1,     // 1 стоп-біт
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
    ESP_LOGI(TAG, "UART1 готовий: %d 8N1 на GPIO%d(TX)/GPIO%d(RX)",
             UART_BAUD, UART_TX_PIN, UART_RX_PIN);
}

void app_main(void)
{
    const gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    const gpio_config_t led_config = {
        .pin_bit_mask = 1ULL << LED_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&button_config));
    ESP_ERROR_CHECK(gpio_config(&led_config));
    ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 0));
    uart_link_init();
    ESP_LOGI(TAG, "Module 4.1: button GPIO%d, LED GPIO%d", BUTTON_PIN, LED_PIN);

    bool led_on = false;
    int stable_button = gpio_get_level(BUTTON_PIN);
    int last_sample = stable_button;
    TickType_t changed_at = xTaskGetTickCount();
    const TickType_t debounce_ticks = pdMS_TO_TICKS(DEBOUNCE_MS) + 1;
    const TickType_t poll_ticks = pdMS_TO_TICKS(POLL_MS) + 1;

    while (1) {
        uint8_t rx_byte;
        if (uart_read_bytes(UART_PORT, &rx_byte, 1, poll_ticks) == 1 &&
            rx_byte == STM_BUTTON_COMMAND) {
            led_on = !led_on;
            ESP_ERROR_CHECK(gpio_set_level(LED_PIN, led_on));
            ESP_LOGI(TAG, "RX B: ESP32 LED %s", led_on ? "ON" : "OFF");
        }

        int sample = gpio_get_level(BUTTON_PIN);
        TickType_t now = xTaskGetTickCount();
        if (sample != last_sample) {
            last_sample = sample;
            changed_at = now;
        }
        if (sample != stable_button && now - changed_at >= debounce_ticks) {
            stable_button = sample;
            if (stable_button == 0) {
                const uint8_t command = ESP_BUTTON_COMMAND;
                if (uart_write_bytes(UART_PORT, &command, 1) == 1) {
                    ESP_LOGI(TAG, "TX A: ESP32 button pressed");
                } else {
                    ESP_LOGE(TAG, "UART write failed");
                }
            }
        }
    }
}
