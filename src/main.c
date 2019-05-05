#include "csro_common.h"

static const int RX_BUF_SIZE = 1024;

#define TXD0_PIN (GPIO_NUM_4)
#define RXD0_PIN (GPIO_NUM_5)

#define TXD1_PIN (GPIO_NUM_16)
#define RXD1_PIN (GPIO_NUM_17)

#define TXD2_PIN (GPIO_NUM_18)
#define RXD2_PIN (GPIO_NUM_19)

static void init()
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_0, &uart_config);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_param_config(UART_NUM_2, &uart_config);

    uart_set_pin(UART_NUM_0, TXD0_PIN, RXD0_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_pin(UART_NUM_1, TXD1_PIN, RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_pin(UART_NUM_2, TXD2_PIN, RXD2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

static void uart0_tx_task()
{
    while (true)
    {
        uart_write_bytes(UART_NUM_0, "Uart0 Hello world\r\n", strlen("Uart0 Hello world\r\n"));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void uart1_tx_task()
{
    while (true)
    {
        uart_write_bytes(UART_NUM_1, "Uart1 Hello world\r\n", strlen("Uart1 Hello world\r\n"));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void uart2_tx_task()
{
    while (true)
    {
        uart_write_bytes(UART_NUM_2, "Uart2 Hello world\r\n", strlen("Uart2 Hello world\r\n"));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    init();
    xTaskCreate(uart0_tx_task, "uart0_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 5, NULL);
    xTaskCreate(uart1_tx_task, "uart1_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 4, NULL);
    xTaskCreate(uart2_tx_task, "uart2_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 3, NULL);
}