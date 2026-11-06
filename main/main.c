#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
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
#define OPERAND_LIMIT   9999
#define DEBOUNCE_US     30000LL
#define HEARTBEAT_US    5000000LL

static const char *TAG = "CALC";
static QueueHandle_t encoder_queue;
static uint8_t previous_ab;
static int8_t quarter_steps;

typedef enum {
    INPUT_FIRST,
    SELECT_OPERATION,
    INPUT_SECOND,
    SHOW_RESULT
} calc_state_t;

typedef enum {
    OP_ADD,
    OP_SUBTRACT,
    OP_MULTIPLY,
    OP_DIVIDE,
    OP_COUNT
} calc_operation_t;

static calc_state_t state = INPUT_FIRST;
static calc_operation_t operation = OP_ADD;
static const char operation_symbols[OP_COUNT] = {'+', '-', '*', '/'};
static int32_t first_operand;
static int32_t second_operand;
static char result_text[48];
static size_t displayed_length;
static int last_raw_button = 1;
static int stable_button = 1;
static int64_t button_changed_at;

static const char *state_name(void)
{
    switch (state) {
    case INPUT_FIRST: return "FIRST_OPERAND";
    case SELECT_OPERATION: return "OPERATION";
    case INPUT_SECOND: return "SECOND_OPERAND";
    case SHOW_RESULT: return "RESULT";
    default: return "UNKNOWN";
    }
}

// Clear the previous expression using backspace, including leftover digits.
static void replace_line(const char *line)
{
    for (size_t i = 0; i < displayed_length; i++) putchar('\b');
    for (size_t i = 0; i < displayed_length; i++) putchar(' ');
    for (size_t i = 0; i < displayed_length; i++) putchar('\b');
    fputs(line, stdout);
    fflush(stdout);
    displayed_length = strlen(line);
}

static void finish_line(void)
{
    if (displayed_length > 0) {
        putchar('\n');
        fflush(stdout);
        displayed_length = 0;
    }
}

static void render_expression(void)
{
    char line[128];
    char second[24];
    if (second_operand < 0) {
        snprintf(second, sizeof(second), "(%" PRId32 ")", second_operand);
    } else {
        snprintf(second, sizeof(second), "%" PRId32, second_operand);
    }
    switch (state) {
    case INPUT_FIRST:
        snprintf(line, sizeof(line), "%" PRId32, first_operand);
        break;
    case SELECT_OPERATION:
        snprintf(line, sizeof(line), "%" PRId32 " %c ",
                 first_operand, operation_symbols[operation]);
        break;
    case INPUT_SECOND:
        snprintf(line, sizeof(line), "%" PRId32 " %c %s",
                 first_operand, operation_symbols[operation], second);
        break;
    case SHOW_RESULT:
        snprintf(line, sizeof(line), "%" PRId32 " %c %s = %s",
                 first_operand, operation_symbols[operation], second, result_text);
        break;
    default:
        return;
    }
    replace_line(line);
}

static void calculate_result(void)
{
    // Bound operands to +/-9999; multiplication is evaluated in int64_t.
    const int64_t left = first_operand;
    const int64_t right = second_operand;
    switch (operation) {
    case OP_ADD:
        snprintf(result_text, sizeof(result_text), "%" PRId64, left + right);
        break;
    case OP_SUBTRACT:
        snprintf(result_text, sizeof(result_text), "%" PRId64, left - right);
        break;
    case OP_MULTIPLY:
        snprintf(result_text, sizeof(result_text), "%" PRId64, left * right);
        break;
    case OP_DIVIDE:
        if (right == 0) {
            snprintf(result_text, sizeof(result_text), "ERROR: division by zero");
        } else {
            snprintf(result_text, sizeof(result_text), "%.6f", (double)left / (double)right);
            size_t length = strlen(result_text);
            while (length > 0 && result_text[length - 1] == '0') result_text[--length] = '\0';
            if (length > 0 && result_text[length - 1] == '.') result_text[--length] = '\0';
            if (strcmp(result_text, "-0") == 0) strcpy(result_text, "0");
        }
        break;
    default:
        snprintf(result_text, sizeof(result_text), "ERROR: invalid operation");
        break;
    }
}

static void start_calculation(void)
{
    finish_line();
    first_operand = 0;
    second_operand = 0;
    operation = OP_ADD;
    state = INPUT_FIRST;
    result_text[0] = '\0';
    ESP_LOGI(TAG, "First operand: rotate RIGHT +1 / LEFT -1, press to confirm");
    render_expression();
}

static void process_encoder(int direction)
{
    if (state == SHOW_RESULT) return;
    if (state == SELECT_OPERATION) {
        operation = (calc_operation_t)((operation + OP_COUNT + direction) % OP_COUNT);
    } else {
        int32_t *operand = state == INPUT_FIRST ? &first_operand : &second_operand;
        const int32_t next = *operand + direction;
        if (next < -OPERAND_LIMIT || next > OPERAND_LIMIT) return;
        *operand = next;
    }
    render_expression();
}

static void confirm_selection(void)
{
    // Discard rotation events queued before confirming the current stage.
    xQueueReset(encoder_queue);
    if (state == SHOW_RESULT) {
        start_calculation();
        return;
    }
    if (state == INPUT_FIRST) {
        finish_line();
        state = SELECT_OPERATION;
        ESP_LOGI(TAG, "Select operation: + - * /, press to confirm");
    } else if (state == SELECT_OPERATION) {
        finish_line();
        state = INPUT_SECOND;
        ESP_LOGI(TAG, "Second operand: rotate RIGHT +1 / LEFT -1, press to calculate");
    } else {
        calculate_result();
        state = SHOW_RESULT;
        render_expression();
        finish_line();
        ESP_LOGI(TAG, "Press button to start a new calculation");
        return;
    }
    render_expression();
}

static bool button_pressed(int raw, int64_t now)
{
    if (raw != last_raw_button) {
        last_raw_button = raw;
        button_changed_at = now;
    }
    if (raw != stable_button && now - button_changed_at >= DEBOUNCE_US) {
        stable_button = raw;
        return stable_button == 0;
    }
    return false;
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

static void init_unused_outputs(void)
{
    // Keep existing servo/buzzer wiring; this exercise does not drive them.
    const gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << SERVO_PIN) | (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&outputs));
    ESP_ERROR_CHECK(gpio_set_level(SERVO_PIN, 0));
    ESP_ERROR_CHECK(gpio_set_level(BUZZER_PIN, 0));
}

void app_main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    encoder_queue = xQueueCreate(64, sizeof(int8_t));
    configASSERT(encoder_queue != NULL);
    init_unused_outputs();
    init_gpio();
    last_raw_button = gpio_get_level(ENC_SW_PIN);
    stable_button = last_raw_button;
    button_changed_at = esp_timer_get_time();
    ESP_LOGI(TAG, "ESP32-S3 ENCODER CALCULATOR STARTED");
    ESP_LOGI(TAG, "Encoder A=GPIO%d B=GPIO%d SW=GPIO%d | Range: -%d..%d",
             ENC_A_PIN, ENC_B_PIN, ENC_SW_PIN, OPERAND_LIMIT, OPERAND_LIMIT);
    start_calculation();
    int64_t last_heartbeat = esp_timer_get_time();
    while (true) {
        int8_t direction;
        if (xQueueReceive(encoder_queue, &direction, pdMS_TO_TICKS(10)) == pdTRUE) {
            process_encoder(direction);
        }
        const int64_t now = esp_timer_get_time();
        if (button_pressed(gpio_get_level(ENC_SW_PIN), now)) confirm_selection();
        if (now - last_heartbeat >= HEARTBEAT_US) {
            // Keep diagnostic logs separate from the editable expression.
            finish_line();
            ESP_LOGI(TAG, "HEARTBEAT | Stage=%s | A=%d B=%d SW=%d", state_name(),
                     gpio_get_level(ENC_A_PIN), gpio_get_level(ENC_B_PIN),
                     gpio_get_level(ENC_SW_PIN));
            if (state != SHOW_RESULT) render_expression();
            last_heartbeat = now;
        }
    }
}
