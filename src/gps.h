#ifndef GPS_H
#define GPS_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <time.h>
#include <date_time.h>

#define UART_BUF_SIZE 256
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(100)
#define UART_RX_TIMEOUT 100

typedef struct {
    char *Message_ID;
    char *Current_Time;
    char *Data_Valid;
    char *Latitude;
    char N_S;
    char *Longitude;
    char E_W;
    char *Speed;
    char *COG;
    char *Date;
    char *Position_Fix_Indicator;
    char *Satellites_Used;
    char *Altitude;
    char *A_Units;
    char *Fix_Age;
    int hour, minute, second, day, month, year;
} gps_data_t;

int UartInit(void);
void ProcessGpsData(void);

#endif // GPS_H
