#ifndef CLOUD_UART_H
#define CLOUD_UART_H

#include <stdint.h>

extern uint8_t sensor[4];

void data_over_uart();
int cloud_uart_init();

#endif // CLOUD_UART_H