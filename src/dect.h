#ifndef DECT_H
#define DECT_H

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_dect_phy.h>

#define CARRIER 1659
#define POWER  0x0f // 23 dBm
#define DATA_LEN 6
#define MCS 1

#define RANGE_SIZE 20

extern int RD_ID;
extern int INIT_DONE;
extern int EXIT;

// Handle values for API calls, separate tx and rx
extern int txHandle;
extern int rxHandle;

// Statistics collecting
extern int previous_received;
extern int missing_errors;
extern int crc_errors;
extern int received_ok;
extern float rssi_average;
extern uint8_t sensor[4];
extern uint8_t lat_buffer[3];
extern uint8_t lon_buffer[3];
extern char mode;
extern int range[RANGE_SIZE];
extern int16_t _rssi2;
extern int16_t snr;
extern int sinks[20];
extern int numSink;
extern bool sink_start;
extern int count10;
extern bool testRange;
extern int relay_select;
extern uint8_t _txData[DATA_LEN];
extern uint8_t _rxData[DATA_LEN];
extern int cluster;
extern uint16_t tx_id;

extern struct k_sem modem;

extern struct nrf_modem_dect_phy_callbacks dect_cb_config;
extern const struct nrf_modem_dect_phy_init_params init_params;

void modem_tx(uint16_t rx_id);
void modem_rx(uint32_t rxMode, int time_s);
void modem_tx_rx(uint8_t rx_id, uint32_t rxMode, int time_s);
void set_txData(uint8_t t, uint8_t p, uint8_t h, uint8_t t2, uint8_t id, uint8_t origin);

#endif // DECT_H
