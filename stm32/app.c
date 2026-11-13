#include "app.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define DEBOUNCE_MS 30U
#define UART_TX_TIMEOUT_MS 10U
#define ESP_BUTTON_COMMAND 'A'
#define STM_BUTTON_COMMAND 'B'

extern UART_HandleTypeDef huart1;
extern USBD_HandleTypeDef hUsbDeviceFS;

static GPIO_PinState stable_button;
static GPIO_PinState last_sample;
static uint32_t changed_at;

static void usb_log(const char *message, uint16_t length)
{
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
        hUsbDeviceFS.pClassData != NULL)
    {
        CDC_Transmit_FS((uint8_t *)message, length);
    }
}

void setup(void)
{
    // The PC13 user LED is active low.
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    stable_button = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
    last_sample = stable_button;
    changed_at = HAL_GetTick();
}

void loop(void)
{
    static const char received[] = "RX A: STM32 LED toggled\r\n";
    static const char sent[] = "TX B: STM32 button pressed\r\n";
    static const char tx_error[] = "UART write failed\r\n";
    uint8_t rx_byte;
    if (HAL_UART_Receive(&huart1, &rx_byte, 1, 0) == HAL_OK &&
        rx_byte == ESP_BUTTON_COMMAND)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        usb_log(received, sizeof(received) - 1);
    }

    GPIO_PinState sample = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
    uint32_t now = HAL_GetTick();
    if (sample != last_sample)
    {
        last_sample = sample;
        changed_at = now;
    }

    if (sample != stable_button && now - changed_at >= DEBOUNCE_MS)
    {
        stable_button = sample;
        if (stable_button == GPIO_PIN_RESET)
        {
            uint8_t command = STM_BUTTON_COMMAND;
            if (HAL_UART_Transmit(&huart1, &command, 1, UART_TX_TIMEOUT_MS) == HAL_OK)
            {
                usb_log(sent, sizeof(sent) - 1);
            }
            else
            {
                usb_log(tx_error, sizeof(tx_error) - 1);
            }
        }
    }
}
