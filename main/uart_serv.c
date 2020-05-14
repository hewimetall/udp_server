#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
static const char *TAG = "UART";

/**
 * This is an example which echos any data it receives on UART0 back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 */

#define BUF_SIZE (1024)

void echo_task()
{
    // Configure parameters of an UART driver,
    // communication pins and install the driver
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

    ESP_ERROR_CHECK( uart_param_config(UART_NUM_0, &uart_config));
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        data[len]='\0';
        ESP_LOGI(TAG,"%s",data);
        // Write data back to the UART
        uart_write_bytes(UART_NUM_0, (const char *) data, len);
    }
}
