#include "dect.h"
#include "cloud_uart.h"

int INIT_DONE = 0;
int EXIT = 0;

// Handle values for API calls, separate tx and rx
int txHandle = 1;
int rxHandle = 31400;

// Statistics collecting
int previous_received = -1;
int missing_errors = 0;
int crc_errors = 0;
int received_ok = 0;
float rssi_average = 0;
int n = 0;
uint8_t lat_buffer[3];
uint8_t lon_buffer[3];
int range[RANGE_SIZE];
int16_t _rssi2;
int16_t snr;
int sinks[20] = {0};
int numSink = 0;
bool sink_start = false;
int count10 = 1;

uint8_t _txData[DATA_LEN];
uint8_t _rxData[DATA_LEN];

int cluster;
uint16_t tx_id;

#define LOG_MODULE_NAME DECT
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

// Semaphore for API calls, only 1 async operation at a time in this sample 
K_SEM_DEFINE(modem, 1, 1);

// Setup the nrf_modem_dect_phy_operation_rx
struct nrf_modem_dect_phy_rx_params rxOpsParams = {0};

// TYPE 2 FORMAT 001 header for messages used
struct phy_ctrl_field_common {
    uint32_t header_format : 3;
    uint32_t packet_length_type : 1;
    uint32_t packet_length : 4;
    uint32_t short_network_id : 8;
    uint32_t transmitter_id_hi : 8;
    uint32_t transmitter_id_lo : 8;
    uint32_t transmit_power : 4;
    uint32_t df_mcs : 4;
    uint32_t receiver_id_hi : 8;
    uint32_t receiver_id_lo : 8;
    uint32_t spatial_stream : 2;
    uint32_t reserved : 6;
    uint32_t feedback_format : 4;
    uint32_t feedback_info_hi : 4;
    uint32_t feedback_info_lo : 8;
};
union nrf_modem_dect_phy_hdr phyHeader;

// New parameters for HARQ operation, not used in this sample
const struct nrf_modem_dect_phy_init_params init_params = {
    .harq_rx_expiry_time_us = 5000000,
    .harq_rx_process_count = 1
};

// ETSI TS 103 636-2 spec 8.3.3 RSSI is reported every 0.5dbm
// If successful reception, calculate the average 
int32_t calcRSSI(int16_t recrssi, int is_success) {
    float resp = -20 - ((-recrssi - 1) * 0.5);
    // avg_new = avg_old + (value - avg_old) / n
    if (is_success) {
        n++;
        float new_average = rssi_average + (resp - rssi_average) / n;
        rssi_average = new_average;
    }
    return (int32_t)resp;
}

// Callback functions from PHY API
void init(const uint64_t* time, int16_t temp, enum nrf_modem_dect_phy_err err, const struct nrf_modem_dect_phy_modem_cfg* cfg) {
    if (err == 0) {
        LOG_INF("DECT Init done, temperature %d", temp);
    } else {
        LOG_ERR("INIT FAILED");
        printk("Init failed, Exit\n");
        EXIT = 1;
    }
    k_sem_give(&modem);
}

void op_complete(const uint64_t* time, int16_t temperature, enum nrf_modem_dect_phy_err err, uint32_t handle) {
    LOG_DBG("operation_complete_cb Status %d, Temp %d, Handle %d\n", err, temperature, handle);
    k_sem_give(&modem);
}

void rssi(const uint64_t* time, const struct nrf_modem_dect_phy_rssi_meas* status) {
    k_sem_give(&modem);
}

void rx_stop(const uint64_t* time, enum nrf_modem_dect_phy_err err, uint32_t handle) {
    LOG_DBG("operation_stop_cb Status %d Handle %d", err, handle);
    k_sem_give(&modem);
}

// PHY header receive
void pcc(const uint64_t* time, const struct nrf_modem_dect_phy_rx_pcc_status* status, const union nrf_modem_dect_phy_hdr* hdr) {
    LOG_DBG("pcc_cb phy_header_valid %d rssi_2 %d", status->header_status, status->rssi_2);
    struct phy_ctrl_field_common* header = (struct phy_ctrl_field_common*)hdr->type_1;
    tx_id = header->transmitter_id_hi << 8 | header->transmitter_id_lo;
    // LOG_INF("Received header from device ID %d", tx_id);
}

void pcc_crc_err(const uint64_t* time, const struct nrf_modem_dect_phy_rx_pcc_crc_failure* crc_failure) {
    crc_errors++;
    int16_t resp = calcRSSI(crc_failure->rssi_2, 0);
    LOG_DBG("PCC CRC ERROR, rssi_2, %d, crc error count, %d, continuing", resp, crc_errors);
}

// Function to check if an ID is within the range array
bool isInRange(int id) {
    for (int i = 0; i < RANGE_SIZE; i++) {
        if (range[i] == id) {
            return true;
        }
    }
    return false;
}

// Function to relay data to other nodes
void relayData(int dest, int delay, bool toSink) {
    if(toSink) {
        bool present = false;
        for(int i = 0; i < 20; i++) {
            if (dest == sinks[i]) {
                present = true;
                break;
            }
        }
        if (present) {
            printk("Sink with RD ID %d found.... Sending Data.\n", _rxData[4]);
            set_txData(_rxData[0], _rxData[1], _rxData[2], _rxData[3], _rxData[4], _rxData[5]);
            modem_tx(dest);
            printk("Forwarded...\n");
            k_msleep(delay);
        } else
            printk("Sink with RD ID %d not found\n", _rxData[4]);
    } else {
        set_txData(_rxData[0], _rxData[1], _rxData[2], _rxData[3], _rxData[4], _rxData[5]);
        modem_tx(dest);
        k_msleep(delay);
    }
}

// Function to relay data between relays
void relaytorelay() {
    int delay = 50; // Adjust as needed
    if (_rxData[4] >= 41 && _rxData[4] <= 60) {
        if (cluster == 2) {
            relayData(11, delay, false); // Relay to final destination
        } else if (cluster == 3) {
            relayData(21, delay, false); // Relay to 2nd relay directly
        }
    } else if (_rxData[4] >= 61 && _rxData[4] <= 80) {
        relayData(21, delay, false); // Relay to final destination
    } else if (_rxData[4] >= 81 && _rxData[4] <= 100) {
        if (cluster == 2) {
            relayData(31, delay, false); // Relay to final destination
        } else if (cluster == 1) {
            relayData(21, delay, false); // Relay to 2nd relay directly
        }
    }
}

void processData() {
  printk("**********************************************************************************\n");
    if (!_rxData[5])
  {
    _rxData[5] = tx_id;
  }
  if (_rxData[4] != rxOpsParams.filter.receiver_identity && mode == 'R') {
    printk("Received data from device ID %d\n", tx_id);
    printk("RX ID does not match with receiver. Data to be forwarded...\n");
    
    if (isInRange(_rxData[4])) {
      relayData(_rxData[4], 0, true);  // Forward directly to the destination
    } else {
      printk("ID not in range, forwarding to next relay.\n");
      relaytorelay();  // Forward to the next relay
    }
  } else if (_rxData[4] == rxOpsParams.filter.receiver_identity) {
    printk(
      "Received data from device ID %d - Originated from device ID %d\nTemp = %d.%d, Pressure = %d, Humidity = %d, rssi_2 %d.%d, snr %d, missed/crc errors %d\r\n",
      tx_id, _rxData[5], _rxData[0], _rxData[3], _rxData[1], _rxData[2], (_rssi2 / 2), (_rssi2 & 0b1) * 5, snr, missing_errors
    );
    if (mode == 'S')
    {
      data_over_uart();
    }
  }
  dk_set_led_off(DK_LED4);
  printk("**********************************************************************************\n");
}

void addSink() {
  bool alreadyAdded = false;
  for (int i = 0; i < numSink; i++) {
    if (sinks[i] == tx_id) {
      alreadyAdded = true;
      break;
    }
  }

  if (!alreadyAdded) {
    sinks[numSink] = tx_id;
    printk("Sink with RD ID %d found\n", tx_id);
    numSink++;
  }
}

// Data payload receive, statistics calculation, and tracking previous received message number
void pdc(const uint64_t* time, const struct nrf_modem_dect_phy_rx_pdc_status* status, const void* data, uint32_t len) {
  if (mode == 'R')
  {
    dk_set_led_on(DK_LED1);
    dk_set_led_off(DK_LED2);
    dk_set_led_off(DK_LED3);
    dk_set_led_on(DK_LED4);
  }
    if (mode == 'S')
  {
    dk_set_led_on(DK_LED1);
    dk_set_led_on(DK_LED2);
    dk_set_led_off(DK_LED3);
    dk_set_led_off(DK_LED4);
  }
  memcpy(&_rxData, data, 6);
  _rssi2 = status->rssi_2;
  snr = status->snr;
  sensor[0] = _rxData[0];
  sensor[1] = _rxData[1];
  sensor[2] = _rxData[2];
  sensor[3] = _rxData[3];
  if (_rxData[0] == 1 && !_rxData[4] && mode == 'R') addSink();
  else processData();
}

void pdc_crc_err(const uint64_t* time, const struct nrf_modem_dect_phy_rx_pdc_crc_failure* crc_failure) {
  crc_errors++;
  int16_t resp = calcRSSI(crc_failure->rssi_2, 0);
  LOG_DBG("PDC CRC ERROR, rssi_2, %d, crc error count, %d, continuing", resp, crc_errors);
}

void link_config(const uint64_t* time, enum nrf_modem_dect_phy_err err) {}

void time_get(const uint64_t* time, enum nrf_modem_dect_phy_err err) {
  LOG_DBG("Time query response time %" PRIu64 " Status %d", *time, err);
}

void capability_get(const uint64_t* time, enum nrf_modem_dect_phy_err err, const struct nrf_modem_dect_phy_capability* capability) {
  LOG_DBG("Capability query response FIXME %" PRIu64 " Status %d", *time, err);
}

void deinit(const uint64_t* time, enum nrf_modem_dect_phy_err err) {
  LOG_DBG("DEINIT response time %" PRIu64 " Status %d", *time, err);
}

// Set function callbacks to structure, registered to modem API in main
struct nrf_modem_dect_phy_callbacks dect_cb_config = {
  .init = init,
  .op_complete = op_complete,
  .rssi = rssi,
  .rx_stop = rx_stop,
  .pcc = pcc,
  .pcc_crc_err = pcc_crc_err,
  .pdc = pdc,
  .pdc_crc_err = pdc_crc_err,
  .link_config = link_config,
  .time_get = time_get,
  .capability_get = capability_get,
  .deinit = deinit
};

// Listen, start immediately and listen for time_s duration
void modem_rx(uint32_t rxMode, int time_s) {
  // Setup the nrf_modem_dect_phy_operation_rx
  // struct nrf_modem_dect_phy_rx_params rxOpsParams = {0};
  rxOpsParams.start_time = 0; // Start immediately
  rxOpsParams.handle = rxHandle;
  rxOpsParams.network_id = 0x0a;
  rxOpsParams.mode = rxMode;
  rxOpsParams.link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED;
  rxOpsParams.rssi_level = 0;
  rxOpsParams.carrier = CARRIER;
  // Modem clock ticks NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ --> 69120*1000* TIME_S
  rxOpsParams.duration = time_s * 69120 * 1000; 
  // Filter on the short network id, last 8 bits of the network identifier in DECT NR
  rxOpsParams.filter.short_network_id = (uint8_t)(0x0a & 0xff);
  rxOpsParams.filter.is_short_network_id_used = 1;
  // Listen for specific transmissions (unicast mode used)
  rxOpsParams.filter.receiver_identity = RD_ID;
  k_sem_take(&modem, K_FOREVER);
  int err = nrf_modem_dect_phy_rx(&rxOpsParams);
  if (err != 0) LOG_ERR("RX FAIL %d", err);
  if (rxHandle == 65000) rxHandle = 31400;
  else rxHandle++;
}

// Send payload immediately, start_time = 0
void modem_tx(uint16_t rx_id) {
  // Define and set the feedback structure
  dect_phy_feedback_t feedback;
  feedback.format0.format = 0;  // Feedback format 1

  // Define a structure for header type 2 with format 1
  struct dect_phy_header_type2_format1_t header = {
      .packet_length = 1,
      .packet_length_type = 1,  // 1 indicates packet length in subslots
      .format = 1,              // Format 1, corresponding to header format 1
      .short_network_id = 0x0A, // Network ID
      .transmitter_identity_hi = (RD_ID >> 8),
      .transmitter_identity_lo = (RD_ID & 0xFF),
      .receiver_identity_hi = (rx_id >> 8),
      .receiver_identity_lo = (rx_id & 0xFF),
      .transmit_power = POWER,
      .df_mcs = MCS,
      .spatial_streams = 0x00,  // Single spatial stream
      .feedback = feedback};

  memcpy(&phyHeader.type_2, &header, 10);

  // Setup the nrf_modem_dect_phy_operation_tx
  struct nrf_modem_dect_phy_tx_params txOpsParams;
  // Immediate operation
  txOpsParams.start_time = 0;
  txOpsParams.handle = txHandle;
  // Network id value, used in rx filtering
  txOpsParams.network_id = 0x0a;
  txOpsParams.phy_type = 1;
  txOpsParams.lbt_rssi_threshold_max = 0;
  // EU carrier, see ETSI TS 103 636-2 5.4.2 for the calculation
  txOpsParams.carrier = CARRIER;
  // NOTE !!! No LBT done
  txOpsParams.lbt_period = 0;
  txOpsParams.phy_header = &phyHeader;
  txOpsParams.data = _txData;
  txOpsParams.data_size = DATA_LEN;
  // Call nrf_modem_dect_phy_schedule_tx_operation_add()
  k_sem_take(&modem, K_FOREVER);
  dk_set_led_on(DK_LED3);
  dk_set_led_on(DK_LED4);
  int err = nrf_modem_dect_phy_tx(&txOpsParams);
  if (err != 0)
    LOG_ERR("TX FAIL %d", err);
  else {
    if (mode == 'S' && !sink_start && count10 == 10)
      printk("Sent Signal to Relays\n");
    else if (mode != 'S')
      printk("Sent: Temp = %d.%d, Pressure = %d, Humidity = %d...... Sent For: %d\r\n", _txData[0], _txData[3], _txData[1], _txData[2], _txData[4]);
  }
  if (txHandle == 30000)
    txHandle = 1;
  else
    txHandle++;
  dk_set_led_on(DK_LED4);
}

void set_txData(uint8_t t, uint8_t p, uint8_t h, uint8_t t2, uint8_t id, uint8_t origin) {
  _txData[0] = t;
  _txData[1] = p;
  _txData[2] = h;
  _txData[3] = t2;
  _txData[4] = id;
  _txData[5] = origin;
}

void modem_tx_rx(uint8_t rx_id, uint32_t rxMode, int time_s) {
// Define and set the feedback structure
  dect_phy_feedback_t feedback;
  feedback.format0.format = 0;  // Feedback format 1

  // Define a structure for header type 2 with format 1
  struct dect_phy_header_type2_format1_t header2 = {
      .packet_length = 1,
      .packet_length_type = 1,  // 1 indicates packet length in subslots
      .format = 1,              // Format 1, corresponding to header format 1
      .short_network_id = 0x0A, // Network ID
      .transmitter_identity_hi = (RD_ID >> 8),
      .transmitter_identity_lo = (RD_ID & 0xFF),
      .receiver_identity_hi = (rx_id >> 8),
      .receiver_identity_lo = (rx_id & 0xFF),
      .transmit_power = POWER,
      .df_mcs = MCS,
      .spatial_streams = 0x00,  // Single spatial stream
      .feedback = feedback};

  memcpy(&phyHeader.type_2, &header2, 10);

  struct nrf_modem_dect_phy_tx_rx_params tx_rx = {
    // Immediate operation
    .tx.start_time = 0,
    .tx.handle = txHandle,
    // Network id value, used in rx filtering
    .tx.network_id = 0x0a,
    .tx.phy_type = 1,
    .tx.lbt_rssi_threshold_max = 0,
    // EU carrier, see ETSI TS 103 636-2 5.4.2 for the calculation
    .tx.carrier = CARRIER,
    // NOTE !!! No LBT done
    .tx.lbt_period = 0,
    .tx.phy_header = &phyHeader,
    .tx.data = _txData,
    .tx.data_size = DATA_LEN,
    .rx.start_time = 19,
    .rx.handle = rxHandle,
    .rx.network_id = 0x0a,
    .rx.mode = rxMode,
    .rx.link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED,
    .rx.rssi_level = 0,
    .rx.carrier = CARRIER,
    // Modem clock ticks NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ --> 69120*1000* TIME_S
    .rx.duration = time_s * 69120 * time_s,
    // Filter on the short network id, last 8 bits of the network identifier in DECT NR
    .rx.filter.short_network_id = (uint8_t)(0x0a & 0xff),
    .rx.filter.is_short_network_id_used = 1,
    // Listen for specific transmissions (unicast mode used)
    .rx.filter.receiver_identity = RD_ID 
  };
  
  int err = nrf_modem_dect_phy_tx_rx(&tx_rx);
  if (err != 0)
    LOG_ERR("TX FAIL %d", err);
}