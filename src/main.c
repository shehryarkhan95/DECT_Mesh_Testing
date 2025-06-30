#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>
#include <date_time.h>
#include "dect.h"
#include "gps.h"
#include "cloud_uart.h"

LOG_MODULE_REGISTER(app);

//Set Radio ID according to required mode and cluster setting
int RD_ID = 1;

uint8_t sensor[4];
uint32_t i;
char mode;
bool testRange = false;
int relay_select = 0;

// External GPS variables
int64_t current_time_ms;

void mapper(int value, uint8_t* buffer) {
    // Extract the three bytes from the value
    buffer[0] = (value >> 16) & 0xFF;  // Most significant byte
    buffer[1] = (value >> 8) & 0xFF;   // Middle byte
    buffer[2] = value & 0xFF;          // Least significant byte
}

void button_handler(uint32_t button_state, uint32_t has_changed) {
    // Check if the first button (usually DK_BTN1) was pressed
    if ((has_changed & button_state) && (button_state & DK_BTN1_MSK)) {
        _txData[4] = 41;
        testRange = false;
        modem_tx(relay_select);
        dk_set_led_off(DK_LED3);
        dk_set_led_off(DK_LED4);
    }
    // Check if the second button (usually DK_BTN2) was pressed
    if ((has_changed & button_state) && (button_state & DK_BTN2_MSK)) {
        _txData[4] = 61;
        testRange = false;
        modem_tx(relay_select);
        dk_set_led_off(DK_LED3);
        dk_set_led_off(DK_LED4);        
    }
    // Check if the third button (usually DK_BTN3) was pressed
    if ((has_changed & button_state) && (button_state & DK_BTN3_MSK)) {
        _txData[4] = 81;
        testRange = false;
        modem_tx(relay_select);
        dk_set_led_off(DK_LED3);
        dk_set_led_off(DK_LED4);        
    }
    if ((has_changed & button_state) && (button_state & DK_BTN4_MSK)) {
        _txData[4] = 81;
        testRange = true;
    }          
}

static const struct device* get_bme280_device(void) {
  const struct device* const dev = DEVICE_DT_GET_ANY(bosch_bme280);

  if (dev == NULL) {
    // No such node, or the node does not have status "okay".
    printk("\nError: no device found.\n");
    return NULL;
  }

  if (!device_is_ready(dev)) {
    printk("\nError: Device \"%s\" is not ready; check the driver initialization logs for errors.\n",
           dev->name);
    return NULL;
  }

  printk("Found device \"%s\", getting sensor data\n", dev->name);
  return dev;
}

void update_n_print_time() {
  struct tm timeinfo;

  // Get the current UTC timestamp in milliseconds
  date_time_now(&current_time_ms);

  // Extract the seconds part
  time_t seconds = (time_t)(current_time_ms / 1000);

  // Get the milliseconds part
  int milliseconds = (int)(current_time_ms % 1000);

  // Convert to structured time (UTC)
  gmtime_r(&seconds, &timeinfo);

  // Display the structured time with milliseconds
  printf("Date and Time (UTC): %02d/%02d/%04d - %2d:%02d:%02d.%03d\n",
         timeinfo.tm_mday,
         timeinfo.tm_mon,
         timeinfo.tm_year + 1900,
         timeinfo.tm_hour,
         timeinfo.tm_min,
         timeinfo.tm_sec,
         milliseconds);
}

void mode_define(){
  if (RD_ID >= 1 && RD_ID <= 10) {
    mode = 'T';
  } else if (RD_ID >= 11 && RD_ID <= 40) {
    mode = 'R';
  } else if (RD_ID >= 41 && RD_ID <= 100) {
    mode = 'S';
  }
}

int dect_init(){
  k_sem_take(&modem, K_FOREVER);
  int err = 0;
  // Define mode according to ID
  mode_define();
  err = nrf_modem_dect_phy_callback_set(&dect_cb_config);
  if (err != 0) {
    printk("ERROR settings callbacks %d\n", err);
  }
  err = nrf_modem_dect_phy_init(&init_params);
  if (err != 0) {
    printk("ERROR initializing modem PHY %d\n", err);
    return -1;
  }
  printk("DECT init started\n");
  nrf_modem_lib_init();
  return 0; 
}

int transmitter(){
    
  // Initialize GPS
  int err = UartInit();
  if (err != 0) {
    printk("uart_gps failed (err %d)\n", err);
    return 0;
  } else {
    printk("GPS Initialized...\n");
  }

  // Initialize DK buttons
  dk_buttons_init(button_handler);

  // Get BME280 device
  const struct device* dev = get_bme280_device(); 

  if (RD_ID == 1) relay_select = 11;
  else if (RD_ID == 2) relay_select = 21;
  else if (RD_ID == 3) relay_select = 31;
  
  // Wait for 1 second
  k_msleep(1000); 
  
  // Start transmission
  printk("Transmission Mode... RD ID: %d\n", RD_ID);
  printk("Data will be sent over relay %d\n", relay_select);
  
  while (1) {
    ProcessGpsData();
    // Sleep for 330 milliseconds
    k_msleep(330);

    // Toggle LEDs
    if (i % 2 == 0) {
      dk_set_led_on(DK_LED1);
      dk_set_led_on(DK_LED2);
    } else {
      dk_set_led_off(DK_LED1);
      dk_set_led_off(DK_LED2);
    }

    // Read sensor data
    struct sensor_value temp, press, humidity;
    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
    sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);

    // Store sensor data in array
    sensor[0] = temp.val1;
    sensor[1] = press.val1;
    sensor[2] = humidity.val1;
    sensor[3] = temp.val2 / 10000;

    // Set transmit data
    set_txData(sensor[0], sensor[1], sensor[2], sensor[3], 0, 0);

    if (testRange)
    {
      _txData[4] = 81;
      printk("Testing Range\n");
      modem_tx(81);
      dk_set_led_off(DK_LED3);
      dk_set_led_off(DK_LED4); 
    }
      
    // Increment counter
    i++;

    // Sleep for 670 milliseconds
    k_msleep(670);

    // Check for looped integer value
    if (i == INT32_MAX) {
      i = 0;
      LOG_INF("Sent integer value looped");
    }
  }
  return 0;
}

void range_set() {
  if (RD_ID >= 11 && RD_ID <= 20) {
    cluster = 1;
    int j = 41;
    for (int k = 0; k < RANGE_SIZE; k++) {
      range[k] = j;
      j++;
    }
  } else if (RD_ID >= 21 && RD_ID <= 30) {
    cluster = 2;
    int j = 61;
    for (int k = 0; k < RANGE_SIZE; k++) {
      range[k] = j;
      j++;
    }
  } else if (RD_ID >= 31 && RD_ID <= 40) {
    cluster = 3;
    int j = 81;
    for (int k = 0; k < RANGE_SIZE; k++) {
      range[k] = j;
      j++;
    }
  }
}

void init_sink(){
  if (41 <= RD_ID && RD_ID <= 60) {
    for(int i=11; i<=20; i++) {
      _txData[0] = 1;
      _txData[4] = 0;
      modem_tx(i);
      k_msleep(50);
      count10++;
    }
    sink_start = true;
  } else if (61 <= RD_ID && RD_ID <= 80) {
    for(int i=21; i<=30; i++) {
      _txData[0] = 1;
      _txData[4] = 0;
      modem_tx(i);
      k_msleep(50);
      count10++;
    }
    sink_start = true;
  } else if (81 <= RD_ID && RD_ID <= 100) {
    for(int i=31; i<=40; i++) {
      _txData[0] = 1;
      _txData[4] = 0;
      modem_tx(i);
      k_msleep(50);
      count10++;
    }
    sink_start = true;
  }
}

int main(void) {

  printk("START DECT MESH\n");
  dk_leds_init();
  printk("leds init\t");

  dect_init();

  if (mode == 'T') {

    transmitter();

  } 
  
  else if (mode == 'R') {
    // Set LEDs for relay mode
    dk_set_led_on(DK_LED2);
    dk_set_led_on(DK_LED3);
    printk("Relay Mode... RD ID: %d\n", RD_ID);
    // Set range
    range_set();

    while (1)
    {
      dk_set_led_off(DK_LED1);
      dk_set_led_on(DK_LED2);
      dk_set_led_on(DK_LED3);
      dk_set_led_off(DK_LED4);
      modem_rx(NRF_MODEM_DECT_PHY_RX_MODE_SEMICONTINUOUS, 1);
      k_msleep(100);
    }
  }

  else if (mode == 'S') {
    cloud_uart_init();
    // Set LEDs for sink mode
    dk_set_led_on(DK_LED3);
    dk_set_led_on(DK_LED4);
    printk("Sink Mode... RD ID: %d\n", RD_ID);
    init_sink();

    while (1)
    {
      dk_set_led_off(DK_LED1);
      dk_set_led_off(DK_LED2);
      dk_set_led_on(DK_LED3);
      dk_set_led_on(DK_LED4);
      modem_rx(NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS, 1);
      k_msleep(100);
      
    }
  }
  return 0;
}
