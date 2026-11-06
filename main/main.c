
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// GPIO
#define ENC_A_PIN       5
#define ENC_B_PIN       6
#define ENC_SW_PIN      7
#define SERVO_PIN       1
#define BUZZER_PIN      2

// Safe settings
#define PIN_LENGTH      4
#define MAX_ATTEMPTS    3
#define DEBOUNCE_US     30000
#define LOCKOUT_SECONDS 30

static const uint8_t SECRET_PIN[PIN_LENGTH] = {1, 2, 3, 4};

static const char *TAG = "SAFE";

static QueueHandle_t encoder_queue;

static volatile uint8_t previous_ab = 0;
static volatile int8_t quarter_steps = 0;

static uint8_t entered_pin[PIN_LENGTH] = {0};

static int digit_index = 0;
static int last_direction = 0;
static int attempts_used = 0;

typedef enum {
    SAFE_ENTERING_PIN,
    SAFE_OPEN,
    SAFE_LOCKOUT
} safe_state_t;

static safe_state_t safe_state = SAFE_ENTERING_PIN;
static int64_t lockout_until = 0;

static const char *safe_state_name(void)
{
    switch (safe_state) {
    case SAFE_ENTERING_PIN: return "ENTERING_PIN";
    case SAFE_OPEN: return "OPEN";
    case SAFE_LOCKOUT: return "LOCKOUT";
    default: return "UNKNOWN";
    }
}

// ==================================================
// SERVO
// ==================================================

static void servo_set_angle(int angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    uint32_t pulse_us = 1000 + angle * 1000 / 180;
    uint32_t duty = pulse_us * (1U << 14) / 20000;

    ESP_ERROR_CHECK(ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_0,
        duty
    ));

    ESP_ERROR_CHECK(ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_0
    ));

    ESP_LOGI(TAG, "SERVO: angle = %d", angle);
}

// ==================================================
// BUZZER
// ==================================================

static void play_tone(uint32_t frequency, int duration_ms)
{
    ESP_ERROR_CHECK(ledc_set_freq(
        LEDC_LOW_SPEED_MODE,
        LEDC_TIMER_1,
        frequency
    ));

    ESP_ERROR_CHECK(ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_1,
        512
    ));

    ESP_ERROR_CHECK(ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_1
    ));

    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    ESP_ERROR_CHECK(ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_1,
        0
    ));

    ESP_ERROR_CHECK(ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_1
    ));

    vTaskDelay(pdMS_TO_TICKS(50));
}

static void play_success(void)
{
    ESP_LOGI(TAG, "Playing success melody");

    play_tone(523, 150);
    play_tone(659, 150);
    play_tone(784, 150);
    play_tone(1047, 400);
}

static void play_alarm(void)
{
    ESP_LOGE(TAG, "ALARM ACTIVATED");

    for (int i = 0; i < 6; i++) {
        play_tone(880, 180);
        play_tone(440, 180);
    }
}

// ==================================================
// PIN DISPLAY
// ==================================================

static void print_pin(void)
{
    printf("PIN: ");

    for (int i = 0; i <= digit_index; i++) {
        printf("%d", entered_pin[i]);
    }

    printf("\n");
    fflush(stdout);
}

// ==================================================
// SAFE LOGIC
// ==================================================

static void lock_safe(void)
{
    ESP_LOGE(TAG, "================================");
    ESP_LOGE(TAG, "SAFE LOCKED! NO ATTEMPTS LEFT");
    ESP_LOGW(TAG, "Try again automatically in %d seconds", LOCKOUT_SECONDS);
    ESP_LOGE(TAG, "================================");

    safe_state = SAFE_LOCKOUT;
    lockout_until = esp_timer_get_time() + LOCKOUT_SECONDS * 1000000LL;
    xQueueReset(encoder_queue);
    servo_set_angle(0);
    play_alarm();
}

static void start_new_attempt(void)
{
    safe_state = SAFE_ENTERING_PIN;
    memset(entered_pin, 0, sizeof(entered_pin));

    digit_index = 0;
    last_direction = 0;

    xQueueReset(encoder_queue);

    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "NEW ATTEMPT: %d/%d",
             attempts_used + 1, MAX_ATTEMPTS);
    ESP_LOGI(TAG, "Rotate encoder to enter PIN");
    ESP_LOGI(TAG, "Change direction to confirm digit");
    ESP_LOGI(TAG, "================================");

    print_pin();
}

static void fail_attempt(void)
{
    attempts_used++;

    ESP_LOGW(TAG, "WRONG PIN OR RESET");
    ESP_LOGW(TAG, "Attempts used: %d/%d",
             attempts_used, MAX_ATTEMPTS);

    if (attempts_used >= MAX_ATTEMPTS) {
        lock_safe();
        return;
    }

    start_new_attempt();
}

static void check_pin(void)
{
    ESP_LOGI(TAG, "Checking PIN...");

    printf("ENTERED PIN: ");

    for (int i = 0; i < PIN_LENGTH; i++) {
        printf("%d", entered_pin[i]);
    }

    printf("\n");
    fflush(stdout);

    if (memcmp(entered_pin, SECRET_PIN, PIN_LENGTH) == 0) {
        ESP_LOGI(TAG, "==========================");
        ESP_LOGI(TAG, "ACCESS GRANTED!");
        ESP_LOGI(TAG, "==========================");

        play_success();
        servo_set_angle(90);

        safe_state = SAFE_OPEN;
        xQueueReset(encoder_queue);
        ESP_LOGI(TAG, "SAFE OPENED | Press encoder button to close and try again");
        return;
    }

    ESP_LOGE(TAG, "ACCESS DENIED!");
    fail_attempt();
}

// ==================================================
// ENCODER PROCESSING
// ==================================================

static void process_encoder(int direction)
{
    const char *direction_name =
        direction > 0 ? "RIGHT" : "LEFT";

    ESP_LOGI(TAG, "Encoder step: %s", direction_name);

    // Changed direction
    if (last_direction != 0 &&
        direction != last_direction) {

        ESP_LOGI(TAG,
                 "Digit %d confirmed: %d",
                 digit_index + 1,
                 entered_pin[digit_index]);

        // Last digit confirmed -> check PIN
        if (digit_index == PIN_LENGTH - 1) {
            check_pin();
            return;
        }

        digit_index++;
        entered_pin[digit_index] = 0;

        ESP_LOGI(TAG,
                 "Moving to digit %d, starting at 0",
                 digit_index + 1);
    }
    else {
        // Increment current digit, wrap from 9 to 0
        entered_pin[digit_index] =
            (entered_pin[digit_index] + 1) % 10;

        ESP_LOGI(TAG,
                 "Digit %d changed to %d",
                 digit_index + 1,
                 entered_pin[digit_index]);
    }

    last_direction = direction;

    // \b moves back one character in the terminal.
    // Full PIN is also logged separately for debugging.
    printf("\b");
    print_pin();
}

// ==================================================
// ENCODER INTERRUPT
// ==================================================

static void encoder_isr(void *arg)
{
    (void)arg;

    static const int8_t transitions[16] = {
         0,  1, -1,  0,
        -1,  0,  0,  1,
         1,  0,  0, -1,
         0, -1,  1,  0
    };

    uint8_t a = gpio_get_level(ENC_A_PIN);
    uint8_t b = gpio_get_level(ENC_B_PIN);

    uint8_t ab = (a << 1) | b;

    int8_t change =
        transitions[(previous_ab << 2) | ab];

    previous_ab = ab;
    quarter_steps += change;

    int direction = 0;

    if (quarter_steps >= 4) {
        direction = 1;
        quarter_steps = 0;
    }
    else if (quarter_steps <= -4) {
        direction = -1;
        quarter_steps = 0;
    }

    if (direction != 0) {
        BaseType_t task_woken = pdFALSE;

        xQueueSendFromISR(
            encoder_queue,
            &direction,
            &task_woken
        );

        if (task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

// ==================================================
// GPIO INITIALIZATION
// ==================================================

static void init_gpio(void)
{
    gpio_config_t encoder_config = {
        .pin_bit_mask =
            (1ULL << ENC_A_PIN) |
            (1ULL << ENC_B_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };

    ESP_ERROR_CHECK(gpio_config(&encoder_config));

    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << ENC_SW_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&button_config));

    previous_ab =
        (gpio_get_level(ENC_A_PIN) << 1) |
         gpio_get_level(ENC_B_PIN);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    ESP_ERROR_CHECK(gpio_isr_handler_add(
        ENC_A_PIN, encoder_isr, NULL
    ));

    ESP_ERROR_CHECK(gpio_isr_handler_add(
        ENC_B_PIN, encoder_isr, NULL
    ));

    ESP_LOGI(TAG, "GPIO initialized");
    ESP_LOGI(TAG, "Encoder A: GPIO %d", ENC_A_PIN);
    ESP_LOGI(TAG, "Encoder B: GPIO %d", ENC_B_PIN);
    ESP_LOGI(TAG, "Encoder SW: GPIO %d", ENC_SW_PIN);
}

// ==================================================
// PWM INITIALIZATION
// ==================================================

static void init_pwm(void)
{
    ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz = 50,
        // ESP32-S3 shares the LEDC source clock across all timers.
        .clk_cfg = LEDC_USE_XTAL_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&servo_timer));

    ledc_timer_config_t buzzer_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 1000,
        .clk_cfg = LEDC_USE_XTAL_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&buzzer_timer));

    ledc_channel_config_t servo_channel = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&servo_channel));

    ledc_channel_config_t buzzer_channel = {
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&buzzer_channel));

    servo_set_angle(0);

    ESP_LOGI(TAG, "PWM initialized");
}

// ==================================================
// MAIN
// ==================================================

void app_main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "ESP32-S3 SAFE STARTED");
    ESP_LOGI(TAG, "PIN length: %d", PIN_LENGTH);
    ESP_LOGI(TAG, "Max attempts: %d", MAX_ATTEMPTS);
    ESP_LOGI(TAG, "================================");

    encoder_queue = xQueueCreate(32, sizeof(int));
    configASSERT(encoder_queue != NULL);

    init_pwm();
    init_gpio();

    start_new_attempt();

    int last_button = gpio_get_level(ENC_SW_PIN);
    int stable_button = last_button;

    int64_t changed_at = esp_timer_get_time();
    int64_t last_heartbeat = esp_timer_get_time();

    while (true) {
        int direction;

        if (xQueueReceive(
                encoder_queue,
                &direction,
                pdMS_TO_TICKS(10)) == pdTRUE) {

            if (safe_state == SAFE_ENTERING_PIN) {
                process_encoder(direction);
            }
        }

        // Button debounce
        int button = gpio_get_level(ENC_SW_PIN);
        int64_t now = esp_timer_get_time();

        if (button != last_button) {
            last_button = button;
            changed_at = now;
        }

        if (button != stable_button &&
            now - changed_at >= DEBOUNCE_US) {

            stable_button = button;

            if (stable_button == 0) {
                if (safe_state == SAFE_OPEN) {
                    servo_set_angle(0);
                    attempts_used = 0;
                    ESP_LOGI(TAG, "SAFE CLOSED | Attempts reset");
                    start_new_attempt();
                } else if (safe_state == SAFE_ENTERING_PIN) {
                    ESP_LOGW(TAG, "BUTTON PRESSED: RESET ATTEMPT");
                    fail_attempt();
                } else {
                    ESP_LOGW(TAG, "LOCKOUT active | Wait for automatic retry");
                }
            }
        }

        // Return to PIN entry without restarting the microcontroller.
        now = esp_timer_get_time();
        if (safe_state == SAFE_LOCKOUT && now >= lockout_until) {
            attempts_used = 0;
            ESP_LOGI(TAG, "LOCKOUT FINISHED | Attempts reset");
            start_new_attempt();
        }

        // Diagnostic message every 5 seconds
        if (now - last_heartbeat >= 5000000) {
            ESP_LOGI(
                TAG,
                "HEARTBEAT | State=%s | A=%d B=%d SW=%d | Attempts used=%d/%d",
                safe_state_name(),
                gpio_get_level(ENC_A_PIN),
                gpio_get_level(ENC_B_PIN),
                gpio_get_level(ENC_SW_PIN),
                attempts_used,
                MAX_ATTEMPTS
            );

            last_heartbeat = now;
        }
    }
}
