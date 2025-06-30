#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include "cloud_uart.h"

const struct device* uart = DEVICE_DT_GET(DT_NODELABEL(uart1));

const struct uart_config uart_cfg = {
    .baudrate = 115200,
    .parity = UART_CFG_PARITY_NONE,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .data_bits = UART_CFG_DATA_BITS_8,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};

void data_over_uart(){
    int ret = uart_tx(uart, sensor, sizeof(sensor), SYS_FOREVER_MS);
    if (ret != 0) {
        printk("Failed to send data over UART: %d\n", ret);
    } else {
        printk("Data Forwarded\n");
    }
}

int cloud_uart_init() {
    int err;

    if (!device_is_ready(uart)) {
        printk("UART device not ready\n");
        return -ENODEV;
    }

    k_msleep(1000); // Ensure any necessary startup delay

    printk("UART Initialized...\n");

    err = uart_configure(uart, &uart_cfg);
    if (err) {
        printk("Failed to configure UART: %d\n", err);
        return err;
    }

    printk("UART Configured...\n");
    return 0;
}