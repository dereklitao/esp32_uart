#include "csro_common.h"

#define BUF_SIZE (1024)

#define TXD0_PIN (GPIO_NUM_4)
#define RXD0_PIN (GPIO_NUM_5)

#define TXD1_PIN (GPIO_NUM_16)
#define RXD1_PIN (GPIO_NUM_17)

#define TXD2_PIN (GPIO_NUM_18)
#define RXD2_PIN (GPIO_NUM_19)

static QueueHandle_t uart0_queue;

uint32_t byte_count0 = 0;
uint32_t byte_count1 = 0;
uint32_t byte_count2 = 0;

uint32_t byte_complete0 = 0;
uint32_t byte_complete1 = 0;
uint32_t byte_complete2 = 0;

void uart0_receive_one_byte(uint8_t data)
{
    byte_count0++;
}

void uart0_receive_complete(void)
{
    byte_complete0++;
    uart_flush_input(UART_NUM_0);
}

void uart1_receive_one_byte(uint8_t data)
{
    byte_count1++;
}

void uart1_receive_complete(void)
{
    byte_complete1++;
}

void uart2_receive_one_byte(uint8_t data)
{
    byte_count2++;
}

void uart2_receive_complete(void)
{
    byte_complete2++;
}

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

    uart0_handler.receive_one_byte = uart0_receive_one_byte;
    uart0_handler.receive_complete = uart0_receive_complete;
    uart1_handler.receive_one_byte = uart1_receive_one_byte;
    uart1_handler.receive_complete = uart1_receive_complete;
    uart2_handler.receive_one_byte = uart2_receive_one_byte;
    uart2_handler.receive_complete = uart2_receive_complete;

    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
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
        char message[200];
        memset(message, 0, 200);
        sprintf(message, "uart0: byte %d, complete %d || uart1: byte %d, complete %d || uart2: byte %d, complete %d\r\n", byte_count0, byte_complete0, byte_count1, byte_complete1, byte_count2, byte_complete2);
        uart_write_bytes(UART_NUM_2, message, strlen(message));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    init();
    xTaskCreate(uart1_tx_task, "uart1_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 4, NULL);
    xTaskCreate(uart2_tx_task, "uart2_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 3, NULL);
}