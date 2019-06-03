#include "csro_common.h"

#define BUF_SIZE (1024)

#define TXD0_PIN (GPIO_NUM_15)
#define RXD0_PIN (GPIO_NUM_2)
#define RTS0_PIN (GPIO_NUM_0)

#define TXD1_PIN (GPIO_NUM_4)
#define RXD1_PIN (GPIO_NUM_17)
#define RTS1_PIN (GPIO_NUM_16)

#define TXD2_PIN (GPIO_NUM_5)
#define RXD2_PIN (GPIO_NUM_19)
#define RTS2_PIN (GPIO_NUM_18)

#define LED_PIN (GPIO_NUM_21)
#define UART0_PIN (GPIO_NUM_0)
#define UART2_PIN (GPIO_NUM_14)

#define RELAY01_PIN (GPIO_NUM_23)
#define RELAY02_PIN (GPIO_NUM_22)

#define RELAY11_PIN (GPIO_NUM_12)
#define RELAY12_PIN (GPIO_NUM_13)

#define GPIO_SELECTED_PIN (1ULL << LED_PIN) | (1ULL << RELAY01_PIN) | (1ULL << RELAY02_PIN) | (1ULL << RELAY11_PIN) | (1ULL << RELAY12_PIN)

uint32_t byte_count[3];
uint32_t complete_count[3];

static void uart_receive_one_byte(uart_port_t uart_num, uint8_t data)
{
    byte_count[uart_num]++;
}

static void uart_receive_complete(uart_port_t uart_num)
{
    complete_count[uart_num]++;
}

static void init(void)
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

    uart_set_pin(UART_NUM_0, TXD0_PIN, RXD0_PIN, RTS0_PIN, UART_PIN_NO_CHANGE);
    uart_set_pin(UART_NUM_1, TXD1_PIN, RXD1_PIN, RTS1_PIN, UART_PIN_NO_CHANGE);
    uart_set_pin(UART_NUM_2, TXD2_PIN, RXD2_PIN, RTS2_PIN, UART_PIN_NO_CHANGE);

    uart_handler.receive_one_byte = uart_receive_one_byte;
    uart_handler.receive_complete = uart_receive_complete;

    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);

    uart_set_mode(UART_NUM_0, UART_MODE_RS485_HALF_DUPLEX);
    uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);
    uart_set_mode(UART_NUM_2, UART_MODE_RS485_HALF_DUPLEX);
}

static void uart0_tx_task(void *param)
{
    while (true)
    {
        uart_write_bytes(UART_NUM_0, "Uart0 Hello world\r\n", 19);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void uart1_tx_task(void *param)
{
    while (true)
    {
        uart_write_bytes(UART_NUM_1, "Uart1 Hello world\r\n", strlen("Uart1 Hello world\r\n"));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void uart2_tx_task(void *param)
{
    while (true)
    {
        char message[200];
        memset(message, 0, 200);
        sprintf(message, "uart0: byte %d, complete %d || uart1: byte %d, complete %d || uart2: byte %d, complete %d\r\n", byte_count[0], complete_count[0], byte_count[1], complete_count[1], byte_count[2], complete_count[2]);
        uart_write_bytes(UART_NUM_2, message, strlen(message));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void led_flash_task(void *param)
{
    static bool flag = false;
    while (true)
    {
        gpio_set_level(LED_PIN, flag);
        //gpio_set_level(RELAY12_PIN, flag);
        //gpio_set_level(RELAY01_PIN, !flag);
        flag = !flag;
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

static void led_gpio_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_SELECTED_PIN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void app_main()
{
    led_gpio_init();
    init();
    xTaskCreate(uart0_tx_task, "uart0_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 3, NULL);
    xTaskCreate(uart1_tx_task, "uart1_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 4, NULL);
    xTaskCreate(uart2_tx_task, "uart2_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 3, NULL);
    xTaskCreate(led_flash_task, "led_flash_task", 1024 * 2, NULL, configMAX_PRIORITIES - 6, NULL);
}