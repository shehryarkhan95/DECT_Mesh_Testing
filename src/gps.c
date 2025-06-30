#include <stdlib.h>
#include <zephyr/sys/ring_buffer.h>
#include "gps.h"

#define LOG_MODULE_NAME UART_GPS
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static const struct device* uart_gps = DEVICE_DT_GET(DT_NODELABEL(uart2));
static struct ring_buf rx_ringbuf;
static uint8_t rx_buffer[1024];  // Adjust the size as needed

const struct uart_config uart_gps_cfg = {
    .baudrate = 9600,
    .parity = UART_CFG_PARITY_NONE,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .data_bits = UART_CFG_DATA_BITS_8,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};

gps_data_t gps_data;

char* StrtokFr(char* s, char delim, char** save_ptr) {
  char* tail;
  char c;

  if (s == NULL) {
    s = *save_ptr;
  }
  tail = s;
  if ((c = *tail) == '\0') {
    s = NULL;
  } else {
    do {
      if (c == delim) {
        *tail++ = '\0';
        break;
      }
    } while ((c = *++tail) != '\0');
  }
  *save_ptr = tail;
  return s;
}

char* StrtokF(char* s, char delim) {
  static char* save_ptr;
  return StrtokFr(s, delim, &save_ptr);
}

int SetDateTime(struct tm* new_date_time) {
  int ret = date_time_set(new_date_time);
  if (ret == 0) {
    // printk("Date and time set successfully.\n");
  } else {
    printk("Failed to set date and time. Error code: %d\n", ret);
  }
  return ret;
}

double NmeaToDecimal(double nmea) {
  int degrees = (int)nmea / 100;  // Extract degrees
  double minutes = nmea - degrees * 100;  // Extract minutes
  return degrees + minutes / 60;  // Convert to decimal
}

void DMStoDecimal(gps_data_t* data) {
  double lat = NmeaToDecimal(strtod(data->Latitude, NULL));
  double lon = NmeaToDecimal(strtod(data->Longitude, NULL));
  int whole_lat = (int)lat;
  int decimal_lat = (int)((lat - whole_lat) * 1000000);
  int whole_lon = (int)lon;
  int decimal_lon = (int)((lon - whole_lon) * 1000000);
  printk("LAT: %d.%06d %c, LON: %d.%06d %c\n", 
         whole_lat, 
         decimal_lat, 
         data->N_S, 
         whole_lon, 
         decimal_lon, 
         data->E_W);
}

bool TimeSet = false;
bool GpsFix = false;
time_t LastPrintTime = 0;

void ParseGprmc(char* sentence, gps_data_t* data) {
  if (strncmp(sentence, "$GPRMC,", strlen("$GPRMC,")) != 0) {
    return;
  }

  char* tokens[15];
  int i = 0;
  char* token = StrtokF(sentence, ',');
  while (token != NULL && i < 15) {
    tokens[i++] = token;
    token = StrtokF(NULL, ',');
  }

  data->Message_ID = tokens[0];
  data->Current_Time = tokens[1];
  data->Data_Valid = tokens[2];
  data->Latitude = tokens[3];
  data->N_S = *tokens[4];  // Assigning individual character
  data->Longitude = tokens[5];
  data->E_W = *tokens[6];  // Assigning individual character
  data->Speed = tokens[7];
  data->COG = tokens[8];
  data->Date = tokens[9];
    
  sscanf(data->Current_Time, "%2d%2d%2d", &data->hour, &data->minute, &data->second);
  sscanf(data->Date, "%2d%2d%2d", &data->day, &data->month, &data->year);

  struct tm new_date_time = {
      .tm_sec = data->second,
      .tm_min = data->minute,
      .tm_hour = data->hour,
      .tm_mday = data->day,
      .tm_mon = data->month - 1,
      .tm_year = 100 + data->year,
      .tm_wday = -1,
      .tm_yday = -1,
      .tm_isdst = -1
  };

  if ((2000 + data->year) <= 2050 && (2000 + data->year) > 2020 && !TimeSet) {
    printk("Setting time to: %d-%d-%d %d:%d:%d\n",
           new_date_time.tm_year + 1900,
           new_date_time.tm_mon + 1,
           new_date_time.tm_mday,
           new_date_time.tm_hour,
           new_date_time.tm_min,
           new_date_time.tm_sec);

    SetDateTime(&new_date_time);
    TimeSet = true;
  }

  if (strcmp(data->Data_Valid, "A") == 0 && !GpsFix) {
    printk("GPS Fix Obtained\n");
    GpsFix = true;
  } 

  time_t current_time = time(NULL);
  if (current_time - LastPrintTime >= 60) {
    LastPrintTime = current_time;
    printk("*************************************\n");
    if (GpsFix && (strcmp(data->Data_Valid, "A") == 0)) {   
      DMStoDecimal(data);
    } else if (strcmp(data->Data_Valid, "A") != 0) {
      GpsFix = false;
      printk("Obtaining GPS Fix\n");
    }
    if ((2000 + data->year) <= 2050 && (2000 + data->year) > 2020) {
      printk("Time: %d-%d-%d %d:%d:%d\n",
             new_date_time.tm_year + 1900,
             new_date_time.tm_mon + 1,
             new_date_time.tm_mday,
             new_date_time.tm_hour,
             new_date_time.tm_min,
             new_date_time.tm_sec);
      SetDateTime(&new_date_time);
    }
    printk("*************************************\n");
  }
}

void UartGpsIsr(const struct device* dev, void* user_data) {
  uint8_t data;
  int recv_len;

  ARG_UNUSED(user_data);

  while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
    if (uart_irq_rx_ready(dev)) {
      recv_len = uart_fifo_read(dev, &data, 1);
      if (recv_len > 0) {
        ring_buf_put(&rx_ringbuf, &data, 1);
      }
    }
  }
}

void ProcessGpsData(void) {
  uint8_t buf[64];  // Adjust the size as needed
  size_t bytes_read;

  while ((bytes_read = ring_buf_get(&rx_ringbuf, buf, sizeof(buf))) > 0) {
    char* sentence = (char*)buf;
    // printk("%s", sentence);
    ParseGprmc(sentence, &gps_data);
  }
}

int UartInit(void) {
  int err;

  if (!device_is_ready(uart_gps)) {
    printk("UART GPS device not ready\n");
    return -ENODEV;
  } else {
    printk("GPS Initialized...\n");
  }

  printk("Attempting to configure UART GPS\n");
  err = uart_configure(uart_gps, &uart_gps_cfg);
  if (err) {
    LOG_ERR("GPS not Configured %d (0x%x)\n", err, err);
    return err;
  } else {
    printk("GPS Configured\n");
  }

  ring_buf_init(&rx_ringbuf, sizeof(rx_buffer), rx_buffer);
  printk("Ring buffer initialized with size: %d\n", sizeof(rx_buffer));

  uart_irq_callback_user_data_set(uart_gps, UartGpsIsr, NULL);
  uart_irq_rx_enable(uart_gps);

  return 0;
}
