#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "bme280.h"

static const char *TAG = "APP";

/* ===================== Налаштування I2C та пристроїв ===================== */
#define I2C_PORT        I2C_NUM_0
#define PIN_SDA         GPIO_NUM_6
#define PIN_SCL         GPIO_NUM_7
#define I2C_FREQ_HZ     100000          /* DS1307 максимум 100 кГц */

#define OLED_ADDR       0x3C
#define OLED_W          128
#define OLED_H          64

#define BME_ADDR_LOW    0x76            /* SDO -> GND */
#define BME_ADDR_HIGH   0x77            /* SDO -> VDD */
#define I2C_TIMEOUT_MS  100

#define UPDATE_PERIOD_MS 1000           /* оновлення раз на секунду */
#define DEG             "\xC2\xB0"      /* знак ° у UTF-8 */
#define SNAPSHOT_INTERVAL_US INT64_C(3600000000)
#define SNAPSHOT_COUNT 8
#define SNAPSHOTS_PER_PAGE 4
#define HISTORY_PAGE_PERIOD_US INT64_C(5000000)

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_bme_i2c;
static struct bme280_dev       s_bme;
static bool                    s_bme_ok;

/* Погодинна історія в RAM. Усі звернення виконує задача LVGL. */
typedef struct {
    int64_t captured_us;
    struct bme280_data values;
    bool valid;
} hourly_snapshot_t;

static hourly_snapshot_t s_snapshots[SNAPSHOT_COUNT];
static unsigned s_snapshot_next;
static unsigned s_snapshot_count;
static int64_t s_history_start_us;
static int64_t s_last_snapshot_hour;

/* Рядки таблиці: поточні дані та чотири записи історії. */
static lv_obj_t *s_data_labels[1 + SNAPSHOTS_PER_PAGE][4];
static lv_obj_t *s_history_title;

static void history_record(int64_t now_us, const struct bme280_data *values, bool valid)
{
    int64_t elapsed_hour = (now_us - s_history_start_us) / SNAPSHOT_INTERVAL_US;
    if (elapsed_hour <= s_last_snapshot_hour) {
        return;
    }

    /* Перший знімок через годину. Пропущені години не заповнюємо поточними даними. */
    s_snapshots[s_snapshot_next] = (hourly_snapshot_t) {
        .captured_us = now_us,
        .values = *values,
        .valid = valid,
    };
    s_snapshot_next = (s_snapshot_next + 1) % SNAPSHOT_COUNT;
    if (s_snapshot_count < SNAPSHOT_COUNT) {
        s_snapshot_count++;
    }
    s_last_snapshot_hour = elapsed_hour;
}

static const hourly_snapshot_t *history_get(unsigned newest_index)
{
    if (newest_index >= s_snapshot_count) {
        return NULL;
    }
    unsigned index = (s_snapshot_next + SNAPSHOT_COUNT - 1 - newest_index) % SNAPSHOT_COUNT;
    return &s_snapshots[index];
}

/* ====================== Callback-и для драйвера Bosch ==================== */
/* Читання len байтів, починаючи з регістра reg_addr */
static BME280_INTF_RET_TYPE bme_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                         uint32_t len, void *intf_ptr)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)intf_ptr;
    esp_err_t err = i2c_master_transmit_receive(dev, &reg_addr, 1, reg_data, len,
                                                I2C_TIMEOUT_MS);
    return (err == ESP_OK) ? BME280_INTF_RET_SUCCESS : -1;
}

/* Запис: адреса регістра + дані одним I2C-пакетом */
static BME280_INTF_RET_TYPE bme_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                          uint32_t len, void *intf_ptr)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)intf_ptr;
    uint8_t buf[32];
    if (len > sizeof(buf) - 1) return -1;
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, len);
    esp_err_t err = i2c_master_transmit(dev, buf, len + 1, I2C_TIMEOUT_MS);
    return (err == ESP_OK) ? BME280_INTF_RET_SUCCESS : -1;
}

/* Затримка потрібна драйверу лише під час ініціалізації (кілька мс) */
static void bme_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period);
}

/* ============================== Ініціалізація ============================ */
/* Ініціалізація шини I2C, дисплея (esp_lcd) та LVGL */
static esp_err_t display_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,           /* внутрішні підтяжки */
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "bus");

    /* Канал вводу/виводу esp_lcd поверх I2C */
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr = OLED_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
        .control_phase_bytes = 1,       /* керуючий байт 0x00 / 0x40 */
        .dc_bit_offset = 6,             /* біт D/C у керуючому байті SSD1306 */
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(s_bus, &io_cfg, &io), TAG, "panel io");

    /* Драйвер контролера SSD1306 (штатний компонент esp_lcd) */
    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_ssd1306_config_t ssd_cfg = { .height = OLED_H };
    esp_lcd_panel_dev_config_t panel_cfg = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,           /* апаратного RESET немає */
        .vendor_config = &ssd_cfg,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ssd1306(io, &panel_cfg, &panel), TAG, "ssd1306");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel), TAG, "init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel, true), TAG, "on");

    /* LVGL: окрема задача, монохромний дисплей */
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "lvgl init");

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io,
        .panel_handle = panel,
        .buffer_size = OLED_W * OLED_H,
        .double_buffer = true,
        .hres = OLED_W,
        .vres = OLED_H,
        .monochrome = true,
        .rotation = { .swap_xy = false, .mirror_x = false, .mirror_y = false },
    };
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);
    ESP_RETURN_ON_FALSE(disp, ESP_FAIL, TAG, "add disp");

    /* Сім рядків по 9 пікселів уміщаються на дисплеї 128x64. */
    lvgl_port_lock(0);
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(scr, lv_color_white(), 0);
    lv_obj_set_style_text_font(scr, &lv_font_unscii_8, 0);
    lv_obj_set_scrollable(scr, false);

    const int column_x[] = {0, 32, 72, 96};
    const int column_width[] = {32, 40, 24, 32};
    const char *column_titles[] = {"AGE", "C", "%", "hPa"};
    for (int column = 0; column < 4; column++) {
        lv_obj_t *header = lv_label_create(scr);
        lv_obj_set_pos(header, column_x[column], 0);
        lv_obj_set_size(header, column_width[column], 9);
        lv_obj_set_style_text_align(header, LV_TEXT_ALIGN_RIGHT, 0);
        lv_label_set_text(header, column_titles[column]);

        for (int row = 0; row < 1 + SNAPSHOTS_PER_PAGE; row++) {
            lv_obj_t *label = lv_label_create(scr);
            s_data_labels[row][column] = label;
            int y = row == 0 ? 9 : (row + 2) * 9;
            lv_obj_set_pos(label, column_x[column], y);
            lv_obj_set_size(label, column_width[column], 9);
            lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_RIGHT, 0);
            lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
            lv_label_set_text(label, "--");
        }
    }
    s_history_title = lv_label_create(scr);
    lv_obj_set_pos(s_history_title, 0, 18);
    lv_obj_set_size(s_history_title, OLED_W, 9);
    lv_label_set_text(s_history_title, "HISTORY 1-4");
    lvgl_port_unlock();
    return ESP_OK;
}

/* Ініціалізація BME280 через офіційний драйвер Bosch */
static esp_err_t bme_init(void)
{
    /* Автовизначення адреси: 0x76 або 0x77 */
    esp_err_t low_result = i2c_master_probe(s_bus, BME_ADDR_LOW, I2C_TIMEOUT_MS);
    esp_err_t high_result = i2c_master_probe(s_bus, BME_ADDR_HIGH, I2C_TIMEOUT_MS);
    ESP_LOGI(TAG, "BME280 probe: 0x76=%s, 0x77=%s (SDA=GPIO6, SCL=GPIO7)",
             esp_err_to_name(low_result), esp_err_to_name(high_result));
    if (low_result != ESP_OK && high_result != ESP_OK) {
        ESP_LOGE(TAG, "BME280 не відповідає на обох адресах. Перевір підключення датчика.");
        return low_result == ESP_ERR_NOT_FOUND ? high_result : low_result;
    }
    uint8_t addr = low_result == ESP_OK ? BME_ADDR_LOW : BME_ADDR_HIGH;
    ESP_LOGI(TAG, "BME280 адреса: 0x%02X", addr);

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &cfg, &s_bme_i2c), TAG, "bme dev");

    uint8_t id_register = BME280_REG_CHIP_ID;
    uint8_t chip_id = 0;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(s_bme_i2c, &id_register, 1, &chip_id, 1,
                                                 I2C_TIMEOUT_MS), TAG, "читання chip ID");
    ESP_LOGI(TAG, "Датчик 0x%02X: chip ID=0x%02X, очікується BME280=0x60", addr, chip_id);
    if (chip_id == 0x58) {
        ESP_LOGE(TAG, "Виявлено BMP280. Потрібен драйвер BMP280; цей датчик не вимірює вологість.");
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* Підключаємо callback-и до драйвера Bosch */
    s_bme.intf     = BME280_I2C_INTF;
    s_bme.read     = bme_i2c_read;
    s_bme.write    = bme_i2c_write;
    s_bme.delay_us = bme_delay_us;
    s_bme.intf_ptr = s_bme_i2c;

    int8_t result = bme280_init(&s_bme);
    if (result != BME280_OK) {
        ESP_LOGE(TAG, "bme280_init: код Bosch=%d, I2C callback=%d", result, s_bme.intf_rslt);
        return ESP_FAIL;
    }

    /* Oversampling x1, нормальний режим, standby 0.5 мс, фільтр вимкнено */
    struct bme280_settings st = {0};
    st.osr_h = BME280_OVERSAMPLING_1X;
    st.osr_p = BME280_OVERSAMPLING_1X;
    st.osr_t = BME280_OVERSAMPLING_1X;
    st.filter = BME280_FILTER_COEFF_OFF;
    st.standby_time = BME280_STANDBY_TIME_0_5_MS;
    result = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &st, &s_bme);
    if (result != BME280_OK) {
        ESP_LOGE(TAG, "bme280_set_sensor_settings: код Bosch=%d", result);
        return ESP_FAIL;
    }
    result = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &s_bme);
    if (result != BME280_OK) {
        ESP_LOGE(TAG, "bme280_set_sensor_mode: код Bosch=%d", result);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BME280 готовий до вимірювання");
    return ESP_OK;
}

/* ================== Періодичне оновлення (таймер LVGL, 1 с) ============== */
static void display_values(int row, const char *age, const struct bme280_data *values, bool valid)
{
    lv_label_set_text(s_data_labels[row][0], age);
    if (!valid) {
        for (int column = 1; column < 4; column++) {
            lv_label_set_text(s_data_labels[row][column], "Err");
        }
        return;
    }
    /* У поточній конфігурації форматер LVGL не підтримує float. */
    char temperature[24], humidity[24], pressure[24];
    snprintf(temperature, sizeof(temperature), "%.1f", values->temperature);
    snprintf(humidity, sizeof(humidity), "%.0f", values->humidity);
    snprintf(pressure, sizeof(pressure), "%.0f", values->pressure / 100.0);
    lv_label_set_text(s_data_labels[row][1], temperature);
    lv_label_set_text(s_data_labels[row][2], humidity);
    lv_label_set_text(s_data_labels[row][3], pressure);
}

static void display_history(int64_t now_us)
{
    unsigned page = 0;
    if (s_snapshot_count > SNAPSHOTS_PER_PAGE) {
        page = (unsigned)((now_us - s_history_start_us) / HISTORY_PAGE_PERIOD_US % 2);
    }
    unsigned first = page * SNAPSHOTS_PER_PAGE;
    lv_label_set_text_fmt(s_history_title, "HIST %u-%u (%u/8)",
                          first + 1, first + SNAPSHOTS_PER_PAGE, s_snapshot_count);
    for (unsigned row = 0; row < SNAPSHOTS_PER_PAGE; row++) {
        const hourly_snapshot_t *snapshot = history_get(first + row);
        if (snapshot == NULL) {
            for (int column = 0; column < 4; column++) {
                lv_label_set_text(s_data_labels[row + 1][column], "--");
            }
            continue;
        }
        unsigned long age_hours = (unsigned long)((now_us - snapshot->captured_us) / SNAPSHOT_INTERVAL_US);
        char age[24];
        snprintf(age, sizeof(age), "%luh", age_hours);
        display_values(row + 1, age, &snapshot->values, snapshot->valid);
    }
}

/* Викликається з задачі LVGL; mutex уже взято, delay() не потрібен */
static void update_cb(lv_timer_t *timer)
{
    (void)timer;
    struct bme280_data d = {0};
    int8_t result = BME280_E_DEV_NOT_FOUND;
    if (s_bme_ok) {
        result = bme280_get_sensor_data(BME280_ALL, &d, &s_bme);
        static int8_t last_result = BME280_OK;
        if (result != BME280_OK && result != last_result) {
            ESP_LOGE(TAG, "bme280_get_sensor_data: код Bosch=%d, I2C callback=%d", result, s_bme.intf_rslt);
        }
        last_result = result;
    }
    int64_t now_us = esp_timer_get_time();
    bool valid = result == BME280_OK;
    history_record(now_us, &d, valid);
    display_values(0, "NOW", &d, valid);
    display_history(now_us);

    if (valid) {
        ESP_LOGI(TAG, "T: %.1f" DEG "C | RH: %.0f%% | P: %.0f hPa | Погодинних знімків: %u/8",
                 d.temperature, d.humidity, d.pressure / 100.0, s_snapshot_count);
    } else {
        ESP_LOGI(TAG, "T: Err | RH: Err | P: Err | Погодинних знімків: %u/8", s_snapshot_count);
    }
}

/* ================================ main =================================== */
void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_ERROR_CHECK(display_init());

    s_bme_ok = (bme_init() == ESP_OK);
    if (!s_bme_ok) {
        ESP_LOGE(TAG, "BME280 недоступний, на екрані буде Err");
    }

    /* Таймер LVGL замість delay(): перший виклик одразу, далі раз на секунду */
    lvgl_port_lock(0);
    s_history_start_us = esp_timer_get_time();
    lv_timer_t *t = lv_timer_create(update_cb, UPDATE_PERIOD_MS, NULL);
    lv_timer_ready(t);
    lvgl_port_unlock();
}
