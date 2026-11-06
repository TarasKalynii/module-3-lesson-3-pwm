#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define ENC_A_PIN       GPIO_NUM_5
#define ENC_B_PIN       GPIO_NUM_6
#define ENC_SW_PIN      GPIO_NUM_7
#define SERVO_PIN       GPIO_NUM_1
#define BUZZER_PIN      GPIO_NUM_2

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_CENTER    90
#define STEP_NORMAL     10
#define STEP_FINE       (STEP_NORMAL / 2)
#define DEBOUNCE_US     30000LL
#define LONG_PRESS_US   1000000LL
#define LIMIT_BEEP_US   120000LL
#define HEARTBEAT_US    5000000LL

static const char *TAG = "SERVO_CTRL";
static QueueHandle_t encoder_queue;
static uint8_t previous_ab;
static int8_t quarter_steps;
static int servo_angle = SERVO_CENTER;
static int step_degrees = STEP_NORMAL;
static int64_t buzzer_until;

static int last_raw_button = 1;
static int stable_button = 1;
static int64_t button_changed_at;
static int64_t button_pressed_at;
static bool long_press_handled;

static void servo_set_angle(int angle)
{
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    // Same pulse range as the working safe: 1000..2000 us, period 20 ms.
    const uint32_t pulse_us = 1000U + (uint32_t)angle * 1000U / 180U;
    const uint32_t duty = pulse_us * (1U << 14) / 20000U;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    servo_angle = angle;
    ESP_LOGI(TAG, "ANGLE=%d deg | STEP=%d deg", servo_angle, step_degrees);
}

static void buzzer_stop(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
    buzzer_until = 0;
}

static void signal_limit(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 512));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
    buzzer_until = esp_timer_get_time() + LIMIT_BEEP_US;
    ESP_LOGW(TAG, "LIMIT: cannot move beyond %d deg", servo_angle);
}

static void process_encoder(int direction)
{
    const int requested_angle = servo_angle + direction * step_degrees;
    // Reaching a limit is allowed; trying to turn past it makes a sound.
    if ((direction > 0 && servo_angle == SERVO_MAX_ANGLE) ||
        (direction < 0 && servo_angle == SERVO_MIN_ANGLE)) {
        signal_limit();
        return;
    }
    servo_set_angle(requested_angle);
}

static void process_button(int raw_button, int64_t now)
{
    if (raw_button != last_raw_button) {
        last_raw_button = raw_button;
        button_changed_at = now;
    }
    if (raw_button != stable_button && now - button_changed_at >= DEBOUNCE_US) {
        stable_button = raw_button;
        if (stable_button == 0) {
            button_pressed_at = now;
            long_press_handled = false;
        } else if (!long_press_handled) {
            // Also handle a long press released just before the next poll.
            if (now - button_pressed_at >= LONG_PRESS_US) {
                ESP_LOGI(TAG, "LONG PRESS: center servo");
                servo_set_angle(SERVO_CENTER);
            } else {
                step_degrees = step_degrees == STEP_NORMAL ? STEP_FINE : STEP_NORMAL;
                ESP_LOGI(TAG, "MODE=%s | STEP=%d deg/tick",
                         step_degrees == STEP_FINE ? "FINE" : "NORMAL", step_degrees);
            }
        }
    }
    if (stable_button == 0 && raw_button == 0 && !long_press_handled &&
        now - button_pressed_at >= LONG_PRESS_US) {
        long_press_handled = true;
        ESP_LOGI(TAG, "LONG PRESS: center servo");
        servo_set_angle(SERVO_CENTER);
    }
}

static void encoder_isr(void *arg)
{
    (void)arg;
    // Keep the direction mapping already verified with this encoder.
    static const int8_t transitions[16] = {
         0,  1, -1,  0,
        -1,  0,  0,  1,
         1,  0,  0, -1,
         0, -1,  1,  0
    };
    const uint8_t ab = (gpio_get_level(ENC_A_PIN) << 1) | gpio_get_level(ENC_B_PIN);
    if ((previous_ab ^ ab) == 3) {
        // Both bits changed: discard an incomplete/invalid quadrature cycle.
        quarter_steps = 0;
        previous_ab = ab;
        return;
    }
    quarter_steps += transitions[(previous_ab << 2) | ab];
    previous_ab = ab;
    int8_t direction = 0;
    if (quarter_steps >= 4) {
        direction = 1;
        quarter_steps = 0;
    } else if (quarter_steps <= -4) {
        direction = -1;
        quarter_steps = 0;
    }
    if (direction != 0) {
        BaseType_t task_woken = pdFALSE;
        xQueueSendFromISR(encoder_queue, &direction, &task_woken);
        if (task_woken == pdTRUE) portYIELD_FROM_ISR();
    }
}

static void init_gpio(void)
{
    const gpio_config_t encoder = {
        .pin_bit_mask = (1ULL << ENC_A_PIN) | (1ULL << ENC_B_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&encoder));
    const gpio_config_t button = {
        .pin_bit_mask = 1ULL << ENC_SW_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button));
    previous_ab = (gpio_get_level(ENC_A_PIN) << 1) | gpio_get_level(ENC_B_PIN);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC_A_PIN, encoder_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC_B_PIN, encoder_isr, NULL));
}

static void init_pwm(void)
{
    const ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_USE_XTAL_CLK,
    };
    const ledc_timer_config_t buzzer_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 2000,
        // All LEDC timers share the source clock on ESP32-S3.
        .clk_cfg = LEDC_USE_XTAL_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&servo_timer));
    ESP_ERROR_CHECK(ledc_timer_config(&buzzer_timer));
    const ledc_channel_config_t servo_channel = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    const ledc_channel_config_t buzzer_channel = {
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&servo_channel));
    ESP_ERROR_CHECK(ledc_channel_config(&buzzer_channel));
    servo_set_angle(SERVO_CENTER);
}

void app_main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    encoder_queue = xQueueCreate(64, sizeof(int8_t));
    configASSERT(encoder_queue != NULL);
    init_pwm();
    init_gpio();
    ESP_LOGI(TAG, "ESP32-S3 ENCODER SERVO STARTED");
    ESP_LOGI(TAG, "Encoder A=GPIO%d B=GPIO%d SW=GPIO%d | Servo=GPIO%d Buzzer=GPIO%d",
             ENC_A_PIN, ENC_B_PIN, ENC_SW_PIN, SERVO_PIN, BUZZER_PIN);
    ESP_LOGI(TAG, "RIGHT: +angle | LEFT: -angle | Click: 10/5 deg | Hold 1s: 90 deg");
    int64_t last_heartbeat = esp_timer_get_time();
    while (true) {
        int8_t direction;
        if (xQueueReceive(encoder_queue, &direction, pdMS_TO_TICKS(10)) == pdTRUE) {
            process_encoder(direction);
        }
        const int64_t now = esp_timer_get_time();
        process_button(gpio_get_level(ENC_SW_PIN), now);
        if (buzzer_until != 0 && now >= buzzer_until) buzzer_stop();
        if (now - last_heartbeat >= HEARTBEAT_US) {
            ESP_LOGI(TAG, "HEARTBEAT | ANGLE=%d STEP=%d | A=%d B=%d SW=%d",
                     servo_angle, step_degrees, gpio_get_level(ENC_A_PIN),
                     gpio_get_level(ENC_B_PIN), gpio_get_level(ENC_SW_PIN));
            last_heartbeat = now;
        }
    }
}
