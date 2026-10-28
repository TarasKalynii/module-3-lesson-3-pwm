#include <stdint.h>

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Потенціометр: від 60% PWM на мінімумі до 100% на максимумі.
#define MOTOR_MIN_PERCENT    60U
#define MOTOR_MAX_PERCENT    100U

#define MOTOR_PIN           7
#define LED_PIN             6
#define POT_CHANNEL         ADC_CHANNEL_3  // ADC1, GPIO4
#define LDR_CHANNEL         ADC_CHANNEL_4  // ADC1, GPIO5
#define PWM_MODE            LEDC_LOW_SPEED_MODE
#define MOTOR_TIMER         LEDC_TIMER_1
#define LED_TIMER           LEDC_TIMER_0
#define MOTOR_CHANNEL       LEDC_CHANNEL_1
#define LED_CHANNEL         LEDC_CHANNEL_0
#define MOTOR_FREQUENCY_HZ  20000
#define LED_FREQUENCY_HZ    1000
#define PWM_RESOLUTION      LEDC_TIMER_10_BIT
#define PWM_PERIOD_TICKS    1024U
#define PWM_MAX_DUTY        (PWM_PERIOD_TICKS - 1U)
#define MOTOR_MIN_TICKS     ((PWM_PERIOD_TICKS * MOTOR_MIN_PERCENT + 50U) / 100U)
#define MOTOR_MAX_TICKS     ((PWM_PERIOD_TICKS * MOTOR_MAX_PERCENT + 50U) / 100U)
#define ADC_MAX_VALUE       4095
#define UPDATE_PERIOD_MS    20
#define LOG_PERIOD_MS       500

_Static_assert(MOTOR_MIN_PERCENT <= MOTOR_MAX_PERCENT && MOTOR_MAX_PERCENT <= 100U,
               "Invalid motor PWM range");

static const char *TAG = "hw_3_3";
static adc_oneshot_unit_handle_t adc_handle;

static void pwm_output_init(int pin, ledc_channel_t channel,
                            ledc_timer_t timer, uint32_t frequency_hz,
                            uint32_t initial_duty)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = timer,
        .freq_hz = frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    const ledc_channel_config_t channel_config = {
        .gpio_num = pin,
        .speed_mode = PWM_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = initial_duty,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

static void adc_inputs_init(void)
{
    const adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_handle));

    const adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, POT_CHANNEL,
                                             &channel_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LDR_CHANNEL,
                                             &channel_config));
}

static uint32_t ldr_to_led_duty(int value)
{
    if (value < 0) {
        value = 0;
    } else if (value > ADC_MAX_VALUE) {
        value = ADC_MAX_VALUE;
    }
    return PWM_MAX_DUTY - (uint32_t)value * PWM_MAX_DUTY / ADC_MAX_VALUE;
}

// Коротка діагностична вибірка самого GPIO, не вимірювач обертів.
static unsigned motor_high_samples(void)
{
    unsigned high = 0;
    for (unsigned i = 0; i < 256; ++i) {
        high += gpio_get_level(MOTOR_PIN) != 0;
        esp_rom_delay_us(1 + i % 13);
    }
    return high;
}

static uint32_t pot_to_motor_duty(int value)
{
    if (value < 0) {
        value = 0;
    } else if (value > ADC_MAX_VALUE) {
        value = ADC_MAX_VALUE;
    }
    const uint32_t span = MOTOR_MAX_TICKS - MOTOR_MIN_TICKS;
    return MOTOR_MIN_TICKS + ((uint32_t)value * span + ADC_MAX_VALUE / 2U) / ADC_MAX_VALUE;
}

static void motor_set_duty(uint32_t duty)
{
    if (duty >= PWM_PERIOD_TICKS) {
        ESP_ERROR_CHECK(ledc_stop(PWM_MODE, MOTOR_CHANNEL, 1));
    } else {
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, MOTOR_CHANNEL, duty));
        ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, MOTOR_CHANNEL));
    }
}

void app_main(void)
{
    pwm_output_init(MOTOR_PIN, MOTOR_CHANNEL, MOTOR_TIMER,
                    MOTOR_FREQUENCY_HZ, MOTOR_MIN_TICKS);
    ESP_ERROR_CHECK(gpio_input_enable(MOTOR_PIN));
    pwm_output_init(LED_PIN, LED_CHANNEL, LED_TIMER, LED_FREQUENCY_HZ, 0);
    adc_inputs_init();

    ESP_LOGI(TAG, "POT GPIO4 -> MOTOR GPIO7: %u..%u%% PWM; LDR GPIO5 -> LED GPIO6",
             (unsigned)MOTOR_MIN_PERCENT, (unsigned)MOTOR_MAX_PERCENT);
    ESP_LOGI(TAG, "Motor %u Hz, LED %u Hz",
             (unsigned)ledc_get_freq(PWM_MODE, MOTOR_TIMER),
             (unsigned)ledc_get_freq(PWM_MODE, LED_TIMER));

    uint32_t motor_duty = MOTOR_MIN_TICKS;
    TickType_t last_log = xTaskGetTickCount();
    while (1) {
        int pot_value = 0;
        int ldr_value = 0;
        esp_err_t pot_result = adc_oneshot_read(adc_handle, POT_CHANNEL, &pot_value);
        esp_err_t ldr_result = adc_oneshot_read(adc_handle, LDR_CHANNEL, &ldr_value);

        // Потенціометр впливає лише на двигун; помилка його ADC вимикає двигун.
        uint32_t target = pot_result == ESP_OK ? pot_to_motor_duty(pot_value) : 0;
        if (target != motor_duty) {
            motor_set_duty(target);
            motor_duty = target;
        }
        // Фоторезистор впливає лише на LED: темніше -> яскравіше.
        uint32_t led_duty = ldr_result == ESP_OK ? ldr_to_led_duty(ldr_value) : 0;
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, LED_CHANNEL, led_duty));
        ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, LED_CHANNEL));

        TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - last_log) >= pdMS_TO_TICKS(LOG_PERIOD_MS)) {
            ESP_LOGI(TAG, "POT_GPIO4=%d MOTOR_GPIO7=%u/1024 (%u%%) PIN7_HIGH=%u/256 "
                     "LDR_GPIO5=%d LED_GPIO6=%u/1023",
                     pot_value, (unsigned)motor_duty,
                     (unsigned)((motor_duty * 100U + PWM_PERIOD_TICKS / 2U) / PWM_PERIOD_TICKS),
                     motor_high_samples(), ldr_value, (unsigned)led_duty);
            if (pot_result != ESP_OK) {
                ESP_LOGW(TAG, "POT ADC read failed: %s", esp_err_to_name(pot_result));
            }
            if (ldr_result != ESP_OK) {
                ESP_LOGW(TAG, "LDR ADC read failed: %s", esp_err_to_name(ldr_result));
            }
            last_log = now;
        }
        vTaskDelay(pdMS_TO_TICKS(UPDATE_PERIOD_MS));
    }
}
