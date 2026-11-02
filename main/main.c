#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "SERVO";

/* Потенціометр: крайні контакти — 3V3 і GND, середній — GPIO1. */
#define POT_GPIO            1
#define POT_ADC_MAX         4095
#define POT_SAMPLES         16
#define UPDATE_MS           20
#define DUTY_DEADBAND       4

/* Сигнал сервоприводу залишається на GPIO4. */
#define SERVO_GPIO          4
#define SERVO_TIMER         LEDC_TIMER_0
#define SERVO_CHANNEL       LEDC_CHANNEL_0
#define SERVO_MODE          LEDC_LOW_SPEED_MODE
#define SERVO_FREQ_HZ       50
#define SERVO_RES           LEDC_TIMER_14_BIT
#define SERVO_DUTY_MAX      ((1 << 14) - 1)
#define SERVO_PERIOD_US     20000
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2400
#define DUTY_MIN            ((SERVO_MIN_US * SERVO_DUTY_MAX) / SERVO_PERIOD_US)
#define DUTY_MAX            ((SERVO_MAX_US * SERVO_DUTY_MAX) / SERVO_PERIOD_US)

static adc_oneshot_unit_handle_t pot_adc;
static adc_channel_t pot_channel;

static void pot_init(void)
{
    adc_unit_t unit;
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(POT_GPIO, &unit, &pot_channel));

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = unit,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &pot_adc));

    adc_oneshot_chan_cfg_t channel_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(pot_adc, pot_channel, &channel_cfg));
}

static int pot_read(void)
{
    int sum = 0;
    for (int i = 0; i < POT_SAMPLES; ++i) {
        int raw;
        ESP_ERROR_CHECK(adc_oneshot_read(pot_adc, pot_channel, &raw));
        sum += raw;
    }
    return sum / POT_SAMPLES;
}

static int pot_to_duty(int raw)
{
    return DUTY_MIN + (raw * (DUTY_MAX - DUTY_MIN)) / POT_ADC_MAX;
}

static void servo_init(int initial_duty)
{
    ledc_timer_config_t timer = {
        .speed_mode = SERVO_MODE,
        .timer_num = SERVO_TIMER,
        .duty_resolution = SERVO_RES,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel = SERVO_CHANNEL,
        .timer_sel = SERVO_TIMER,
        .duty = (uint32_t)initial_duty,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

void app_main(void)
{
    pot_init();
    int filtered = pot_read();
    int duty = pot_to_duty(filtered);
    servo_init(duty);
    ESP_LOGI(TAG, "Потенціометр GPIO%d -> серво GPIO%d, duty %d..%d",
             POT_GPIO, SERVO_GPIO, DUTY_MIN, DUTY_MAX);

    TickType_t last_log = xTaskGetTickCount();
    while (1) {
        /* Усереднення та фільтр зменшують тремтіння від шуму АЦП. */
        filtered += (pot_read() - filtered) / 4;
        int target = pot_to_duty(filtered);
        int delta = target - duty;
        if (delta >= DUTY_DEADBAND || delta <= -DUTY_DEADBAND) {
            duty = target;
            ESP_ERROR_CHECK(ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, (uint32_t)duty));
            ESP_ERROR_CHECK(ledc_update_duty(SERVO_MODE, SERVO_CHANNEL));
        }

        TickType_t now = xTaskGetTickCount();
        if (now - last_log >= pdMS_TO_TICKS(500)) {
            ESP_LOGI(TAG, "ADC: %d / %d, duty: %d", filtered, POT_ADC_MAX, duty);
            last_log = now;
        }
        vTaskDelay(pdMS_TO_TICKS(UPDATE_MS));
    }
}
