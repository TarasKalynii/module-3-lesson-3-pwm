#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/ledc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LIGHT_ADC_UNIT ADC_UNIT_1
#define LIGHT_ADC_CHANNEL ADC_CHANNEL_8  // GPIO9 on ESP32-S3
#define LIGHT_ADC_ATTENUATION ADC_ATTEN_DB_12
#define LIGHT_ADC_BITWIDTH ADC_BITWIDTH_12

#define SERVO_GPIO 14
#define SERVO_MODE LEDC_LOW_SPEED_MODE
#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_CHANNEL LEDC_CHANNEL_0
#define SERVO_RESOLUTION LEDC_TIMER_12_BIT
#define SERVO_FREQUENCY_HZ 50U
#define SERVO_PERIOD_US (1000000U / SERVO_FREQUENCY_HZ)
#define SERVO_DUTY_STEPS (1U << SERVO_RESOLUTION)

enum {
    LIGHT_MIN_MV = 50,
    LIGHT_MAX_MV = 3000,
    SERVO_MIN_DEG = 10,
    SERVO_MAX_DEG = 170,
    // Match the range used with the servo in the previous exercises.
    SERVO_MIN_PULSE_US = 1000,
    SERVO_MAX_PULSE_US = 2000,
    SAMPLE_INTERVAL_MS = 100,
    LOG_INTERVAL_SAMPLES = 5,
};

static void init_light_sensor(adc_oneshot_unit_handle_t *adc, adc_cali_handle_t *calibration)
{
    const adc_oneshot_unit_init_cfg_t unit = {
        .unit_id = LIGHT_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, adc));

    const adc_oneshot_chan_cfg_t channel = {
        .atten = LIGHT_ADC_ATTENUATION,
        .bitwidth = LIGHT_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc, LIGHT_ADC_CHANNEL, &channel));

    const adc_cali_curve_fitting_config_t config = {
        .unit_id = LIGHT_ADC_UNIT,
        .chan = LIGHT_ADC_CHANNEL,
        .atten = LIGHT_ADC_ATTENUATION,
        .bitwidth = LIGHT_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&config, calibration));
}

static void init_servo(void)
{
    const ledc_timer_config_t timer = {
        .speed_mode = SERVO_MODE,
        .timer_num = SERVO_TIMER,
        .duty_resolution = SERVO_RESOLUTION,
        .freq_hz = SERVO_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    const ledc_channel_config_t channel = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel = SERVO_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = SERVO_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

static esp_err_t read_light(adc_oneshot_unit_handle_t adc, adc_cali_handle_t calibration,
                            int *raw, int *millivolts)
{
    esp_err_t error = adc_oneshot_read(adc, LIGHT_ADC_CHANNEL, raw);
    if (error != ESP_OK) {
        return error;
    }
    return adc_cali_raw_to_voltage(calibration, *raw, millivolts);
}

static int light_to_angle(int millivolts)
{
    if (millivolts < LIGHT_MIN_MV) {
        millivolts = LIGHT_MIN_MV;
    } else if (millivolts > LIGHT_MAX_MV) {
        millivolts = LIGHT_MAX_MV;
    }

    return SERVO_MIN_DEG + (millivolts - LIGHT_MIN_MV) *
           (SERVO_MAX_DEG - SERVO_MIN_DEG) / (LIGHT_MAX_MV - LIGHT_MIN_MV);
}

static uint32_t angle_to_pulse_us(int angle)
{
    return SERVO_MIN_PULSE_US + (angle - SERVO_MIN_DEG) *
           (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
}

static uint32_t pulse_to_duty(uint32_t pulse_us)
{
    // Round once, after multiplication, so we retain the timer resolution.
    return (pulse_us * SERVO_DUTY_STEPS + SERVO_PERIOD_US / 2U) / SERVO_PERIOD_US;
}

void app_main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    puts("\n[LDR_SERVO] START: ESP32-S3 LDR and servo demo");

    adc_oneshot_unit_handle_t adc;
    adc_cali_handle_t calibration;
    puts("[LDR_SERVO] Initializing ADC on GPIO9...");
    init_light_sensor(&adc, &calibration);
    puts("[LDR_SERVO] ADC ready; initializing PWM on GPIO14...");
    init_servo();
    puts("[LDR_SERVO] READY: reading the light sensor and driving the servo");

    uint32_t sample_number = 0;
    while (true) {
        int raw;
        int millivolts;
        const bool log_sample = sample_number++ % LOG_INTERVAL_SAMPLES == 0;
        esp_err_t error = read_light(adc, calibration, &raw, &millivolts);
        if (error == ESP_OK) {
            const int angle = light_to_angle(millivolts);
            const uint32_t pulse_us = angle_to_pulse_us(angle);
            const uint32_t duty = pulse_to_duty(pulse_us);

            ESP_ERROR_CHECK(ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty));
            ESP_ERROR_CHECK(ledc_update_duty(SERVO_MODE, SERVO_CHANNEL));
            if (log_sample) {
                printf("[LDR_SERVO] sample=%" PRIu32 " raw=%d voltage=%d mV angle=%d deg "
                       "pulse=%" PRIu32 " us duty=%" PRIu32 "\n",
                       sample_number, raw, millivolts, angle, pulse_us, duty);
            }
        } else if (log_sample) {
            printf("[LDR_SERVO] sample=%" PRIu32 " ADC error: %s\n",
                   sample_number, esp_err_to_name(error));
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}
