#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RTC_ADDRESS  0x68
#define OLED_ADDRESS 0x3C
#define SDA_PIN      GPIO_NUM_6
#define SCL_PIN      GPIO_NUM_7
#define I2C_SPEED_HZ 100000
#define I2C_TIMEOUT_MS 200

#define OLED_WIDTH  128
#define OLED_HEIGHT 64

static const char *TAG = "DESK_CLOCK";
static const char *const weekdays[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

typedef struct {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
} clock_time_t;

// Перший байт 0x40 вказує SSD1306, що далі йдуть пікселі.
static uint8_t oled_frame[1 + OLED_WIDTH * OLED_HEIGHT / 8] = {0x40};

static const uint8_t digit_pixels[10][7] = {
    {14, 17, 19, 21, 25, 17, 14}, {4, 12, 4, 4, 4, 4, 14},
    {14, 17, 1, 2, 4, 8, 31},    {30, 1, 1, 14, 1, 1, 30},
    {2, 6, 10, 18, 31, 2, 2},    {31, 16, 16, 30, 1, 1, 30},
    {14, 16, 16, 30, 17, 17, 14},{31, 1, 2, 4, 8, 8, 8},
    {14, 17, 17, 14, 17, 17, 14},{14, 17, 17, 15, 1, 1, 14},
};

static const uint8_t text_pixels[128][7] = {
    [':'] = {0, 4, 4, 0, 4, 4, 0}, ['.'] = {0, 0, 0, 0, 0, 0, 4},
    ['-'] = {0, 0, 0, 14, 0, 0, 0},
    ['F'] = {31, 16, 16, 30, 16, 16, 16}, ['M'] = {17, 27, 21, 17, 17, 17, 17},
    ['S'] = {15, 16, 16, 14, 1, 1, 30}, ['T'] = {31, 4, 4, 4, 4, 4, 4},
    ['W'] = {17, 17, 17, 17, 21, 27, 17},
    ['a'] = {0, 0, 14, 1, 15, 17, 15}, ['d'] = {1, 1, 15, 17, 17, 17, 15},
    ['e'] = {0, 0, 14, 17, 31, 16, 14}, ['h'] = {16, 16, 30, 17, 17, 17, 17},
    ['i'] = {4, 0, 12, 4, 4, 4, 14}, ['n'] = {0, 0, 30, 17, 17, 17, 17},
    ['o'] = {0, 0, 14, 17, 17, 17, 14}, ['r'] = {0, 0, 22, 25, 16, 16, 16},
    ['t'] = {8, 8, 30, 8, 8, 9, 6}, ['u'] = {0, 0, 17, 17, 17, 19, 13},
};

static int weekday_index(const clock_time_t *time)
{
    struct tm calendar = {
        .tm_year = time->year - 1900,
        .tm_mon = time->month - 1,
        .tm_mday = time->day,
    };
    mktime(&calendar);
    return calendar.tm_wday;
}

static int from_bcd(uint8_t value)
{
    const int tens = value >> 4;
    const int ones = value & 0x0f;
    return (tens <= 9 && ones <= 9) ? tens * 10 + ones : -1;
}

static uint8_t to_bcd(int value)
{
    return (uint8_t)(((value / 10) << 4) | (value % 10));
}

static esp_err_t rtc_read(i2c_master_dev_handle_t rtc, clock_time_t *time)
{
    static const int days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    const uint8_t start_register = 0x00;
    uint8_t registers[7];
    esp_err_t result = i2c_master_transmit_receive(rtc, &start_register, 1, registers,
                                                  sizeof(registers), I2C_TIMEOUT_MS);
    if (result != ESP_OK) {
        return result;
    }

    int hour = from_bcd(registers[2] & 0x3f);
    if (registers[2] & 0x40) {
        // У 12-годинному режимі біт 5 позначає PM.
        const int hour_12 = from_bcd(registers[2] & 0x1f);
        hour = (hour_12 >= 1 && hour_12 <= 12)
             ? hour_12 % 12 + ((registers[2] & 0x20) ? 12 : 0)
             : -1;
    }

    const clock_time_t value = {
        .year = 2000 + from_bcd(registers[6]),
        .month = from_bcd(registers[5] & 0x1f),
        .day = from_bcd(registers[4] & 0x3f),
        .hour = hour,
        .minute = from_bcd(registers[1] & 0x7f),
        .second = from_bcd(registers[0] & 0x7f),
    };

    // CH = 1 означає зупинений генератор DS1307. Рік 2000 вважаємо заводським.
    if ((registers[0] & 0x80) || value.year <= 2000 ||
        value.month < 1 || value.month > 12 || value.day < 1 ||
        value.hour < 0 || value.hour > 23 || value.minute < 0 || value.minute > 59 ||
        value.second < 0 || value.second > 59) {
        return ESP_ERR_INVALID_STATE;
    }

    const int leap_day = value.month == 2 && value.year % 4 == 0;
    if (value.day > days_in_month[value.month - 1] + leap_day) {
        return ESP_ERR_INVALID_STATE;
    }

    *time = value;
    return ESP_OK;
}

static esp_err_t rtc_set_build_time(i2c_master_dev_handle_t rtc)
{
    static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char month[4];
    clock_time_t time;
    if (sscanf(__DATE__, "%3s %d %d", month, &time.day, &time.year) != 3 ||
        sscanf(__TIME__, "%d:%d:%d", &time.hour, &time.minute, &time.second) != 3) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *month_position = strstr(months, month);
    if (month_position == NULL || time.year < 2001 || time.year > 2099) {
        return ESP_ERR_INVALID_STATE;
    }
    time.month = (int)(month_position - months) / 3 + 1;

    const uint8_t registers[] = {
        0x00, to_bcd(time.second), to_bcd(time.minute), to_bcd(time.hour),
        to_bcd(weekday_index(&time) + 1), to_bcd(time.day),
        to_bcd(time.month), to_bcd(time.year - 2000),
    };
    ESP_LOGW(TAG, "RTC мав збитий час. Встановлюю час збірки: %s %s", __DATE__, __TIME__);
    return i2c_master_transmit(rtc, registers, sizeof(registers), I2C_TIMEOUT_MS);
}

static esp_err_t oled_init(i2c_master_dev_handle_t oled)
{
    const uint8_t commands[] = {
        0x00,       // Далі йдуть команди SSD1306.
        0xAE,       // Вимкнути дисплей під час налаштування.
        0xD5, 0x80, 0xA8, OLED_HEIGHT - 1, 0xD3, 0x00, 0x40,
        0x8D, 0x14, // Увімкнути внутрішній перетворювач живлення OLED.
        0x20, 0x00, // Горизонтальна адресація буфера.
        0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x7F, 0xD9, 0xF1, 0xDB, 0x40,
        0xA4, 0xA6, 0x2E, 0xAF,
    };
    return i2c_master_transmit(oled, commands, sizeof(commands), I2C_TIMEOUT_MS);
}

static esp_err_t oled_show(i2c_master_dev_handle_t oled)
{
    const uint8_t address_window[] = {
        0x00, 0x21, 0, OLED_WIDTH - 1, 0x22, 0, OLED_HEIGHT / 8 - 1,
    };
    esp_err_t result = i2c_master_transmit(oled, address_window, sizeof(address_window), I2C_TIMEOUT_MS);
    if (result != ESP_OK) {
        return result;
    }
    return i2c_master_transmit(oled, oled_frame, sizeof(oled_frame), I2C_TIMEOUT_MS);
}

static void fill_rect(int x, int y, int width, int height)
{
    for (int row = y; row < y + height; row++) {
        for (int col = x; col < x + width; col++) {
            oled_frame[1 + (row / 8) * OLED_WIDTH + col] |= 1u << (row % 8);
        }
    }
}

static void draw_centered(const char *text, int y, int scale)
{
    int width = ((int)strlen(text) * 6 - 1) * scale;
    int x = (OLED_WIDTH - width) / 2;
    for (const char *character = text; *character != '\0'; character++, x += 6 * scale) {
        unsigned char symbol = (unsigned char)*character;
        const uint8_t *pixels = symbol >= '0' && symbol <= '9'
                              ? digit_pixels[symbol - '0'] : text_pixels[symbol];
        for (int row = 0; row < 7; row++) {
            for (int col = 0; col < 5; col++) {
                if (pixels[row] & (1u << (4 - col))) {
                    fill_rect(x + col * scale, y + row * scale, scale, scale);
                }
            }
        }
    }
}

static esp_err_t update_display(i2c_master_dev_handle_t oled, const clock_time_t *time, bool log_frame)
{
    char time_text[24];
    char date_text[32];
    snprintf(time_text, sizeof(time_text), "%02d:%02d:%02d", time->hour, time->minute, time->second);
    snprintf(date_text, sizeof(date_text), "%s %02d.%02d.%04d",
             weekdays[weekday_index(time)], time->day, time->month, time->year);
    memset(&oled_frame[1], 0, sizeof(oled_frame) - 1);
    draw_centered(time_text, 15, 2);
    draw_centered(date_text, 40, 1);

    esp_err_t result = oled_show(oled);
    if (result == ESP_OK && log_frame) {
        ESP_LOGI(TAG, "OLED: %s | %s", time_text, date_text);
    }
    return result;
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(5000));

    const i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));

    i2c_master_dev_handle_t rtc;
    i2c_master_dev_handle_t oled;
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = RTC_ADDRESS,
        .scl_speed_hz = I2C_SPEED_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &device_config, &rtc));
    device_config.device_address = OLED_ADDRESS;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &device_config, &oled));
    ESP_ERROR_CHECK(oled_init(oled));

    clock_time_t current_time;
    if (rtc_read(rtc, &current_time) == ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(rtc_set_build_time(rtc));
    }

    int last_second = -1;
    int logged_frames = 0;
    esp_err_t last_error = ESP_OK;
    while (true) {
        esp_err_t result = rtc_read(rtc, &current_time);
        if (result == ESP_OK && current_time.second != last_second) {
            result = update_display(oled, &current_time, logged_frames < 2);
            if (result == ESP_OK) {
                last_second = current_time.second;
                if (logged_frames < 2) {
                    logged_frames++;
                }
            }
        }
        if (result != ESP_OK && result != last_error) {
            ESP_LOGW(TAG, "Помилка RTC або OLED: %s", esp_err_to_name(result));
            memset(&oled_frame[1], 0, sizeof(oled_frame) - 1);
            draw_centered("--:--:--", 15, 2);
            oled_show(oled);
            last_second = -1;
        }
        last_error = result;
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}
