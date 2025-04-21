/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This sketch demonstrates the central API() that allows you to connect
 * to multiple peripherals boards (Bluefruit nRF52 in peripheral mode, or
 * any Bluefruit nRF51 boards).
 *
 * One or more Bluefruit boards, configured as a peripheral with the
 * bleuart service running are required for this demo.
 *
 * This sketch will: 
 *  - Read data from the HW serial port (normally USB serial, accessible
 *    via the Serial Monitor for example), and send any incoming data to
 *    all other peripherals connected to the central device.
 *  - Forward any incoming bleuart messages from a peripheral to all of
 *    the other connected devices.
 * 
 * It is recommended to give each peripheral board a distinct name in order
 * to more easily distinguish the individual devices.
 * 
 * Connection Handle Explanation
 * -----------------------------
 * The total number of connections is BLE_MAX_CONNECTION (20)
 * 
 * The 'connection handle' is an integer number assigned by the SoftDevice
 * (Nordic's proprietary BLE stack). Each connection will receive it's own
 * numeric 'handle' starting from 0 to BLE_MAX_CONNECTION-1, depending on the order
 * of connection(s).
 *
 * - E.g If our Central board connects to a mobile phone first (running as a peripheral),
 * then afterwards connects to another Bluefruit board running in peripheral mode, then
 * the connection handle of mobile phone is 0, and the handle for the Bluefruit
 * board is 1, and so on.
 */

/* LED PATTERNS
 * ------------
 * LED_RED   - Blinks pattern changes based on the number of connections.
 * LED_BLUE  - Blinks constantly when scanning
 */

#include <bluefruit.h>
#include <string.h>

// Struct containing peripheral info
typedef struct
{
  char name[16+1];

  uint16_t conn_handle;

  // Each prph need its own bleuart client service
  BLEClientUart bleuart;
} prph_info_t;

/* Peripheral info array (one per peripheral device)
 * 
 * There are 'BLE_MAX_CONNECTION' central connections, but the
 * the connection handle can be numerically larger (for example if
 * the peripheral role is also used, such as connecting to a mobile
 * device). As such, we need to convert connection handles <-> the array
 * index where appropriate to prevent out of array accesses.
 * 
 * Note: One can simply declares the array with BLE_MAX_CONNECTION and use connection
 * handle as index directly with the expense of SRAM.
 */
prph_info_t prphs[BLE_MAX_CONNECTION];

// Software Timer for blinking the RED LED
SoftwareTimer blinkTimer;
uint8_t connection_num = 0; // for blink pattern
uint8_t STATE = 0;

volatile uint16_t FIFOcount = 0;
volatile uint32_t receivedData[256];

bool rx1 = false;
bool rx2 = false;
// bool SYNC_SENT = false;
// bool p1SYNCED = false;
// bool p2SYNCED = false;


String syncSTR = "SYNC";

uint8_t sync_payload[] = { 'S', 'Y', 'N', 'C' };
unsigned long last_sync = 0;
bool received_from_client[BLE_MAX_CONNECTION] = {false}; // Track each one
bool ready_to_sync = false;

uint8_t rx_buffer[2][256*3]; // Store raw data
uint16_t expected_bytes[2];
uint16_t received_bytes[2];
bool receiving[2];


void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  // Initialize blinkTimer for 100 ms and start it
  blinkTimer.begin(100, blink_timer_callback);
  blinkTimer.start();

  Serial.println("Bluefruit52 Central Multi BLEUART Example");
  Serial.println("-----------------------------------------\n");
  
  // Initialize Bluefruit with max concurrent connections as Peripheral = 0, Central = 4
  // SRAM usage required by SoftDevice will increase with number of connections
  Bluefruit.begin(0, 4);

  // Set Name
  Bluefruit.setName("Xiao Central");

///////////////////////////////////////////////////////////////////////////////////
  // Speed up connection (CRITICAL for fast FIFO data transmissions)
  Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);  // Maximize MTU
  Bluefruit.Central.setConnInterval(6, 12);  // Faster connection interval
  Bluefruit.setTxPower(8);  // Boost TX power
  ///////////////////////////////////////////////////////////////////////////////////
  
  // Init peripheral pool
  for (uint8_t idx=0; idx<BLE_MAX_CONNECTION; idx++)
  {
    // Invalid all connection handle
    prphs[idx].conn_handle = BLE_CONN_HANDLE_INVALID;
    
    // All of BLE Central Uart Serivce
    prphs[idx].bleuart.begin();
    prphs[idx].bleuart.setRxCallback(bleuart_rx_callback);
  }

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  // BLE Sync Advertising Setup
  startAdv();
  /////////////////////////////

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Filter only accept bleuart service in advertising
   * - Don't use active scan (used to retrieve the optional scan response adv packet)
   * - Start(0) = will scan forever since no timeout is given
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE);
  Bluefruit.Scanner.useActiveScan(false);       // Don't request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds
}

// Bluetooth Advertising Function
void startAdv(void){
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED);
  Bluefruit.Advertising.setInterval(160, 160); // 100 ms
  Bluefruit.Advertising.setFastTimeout(1);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
}

/**
 * Callback invoked when scanner picks up an advertising packet
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Since we configure the scanner with filterUuid()
  // Scan callback only invoked for device with bleuart service advertised  
  // Connect to the device with bleuart service in advertising packet
  Bluefruit.Central.connect(report);
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  // Find an available ID to use
  int id  = findConnHandle(BLE_CONN_HANDLE_INVALID);

  // Eeek: Exceeded the number of connections !!!
  if ( id < 0 ) return;
  
  prph_info_t* peer = &prphs[id];
  peer->conn_handle = conn_handle;
  
  Bluefruit.Connection(conn_handle)->getPeerName(peer->name, sizeof(peer->name)-1);

  Serial.print("Connected to ");
  Serial.println(peer->name);

  Serial.print("Discovering BLE UART service ... ");

  if ( peer->bleuart.discover(conn_handle) )
  {
    Serial.println("Found it");
    Serial.println("Enabling TXD characteristic's CCCD notify bit");
    peer->bleuart.enableTXD();

    Serial.println("Continue scanning for more peripherals"); //////////////////////////////////////////////////
    Bluefruit.Scanner.start(0);

    Serial.println("Enter some text in the Serial Monitor to send it to all connected peripherals:");
  } else
  {
    Serial.println("Found ... NOTHING!");

    // disconnect since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
  }  

  connection_num++;
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  connection_num--;

  // Mark the ID as invalid
  int id  = findConnHandle(conn_handle);

  // Non-existant connection, something went wrong, DBG !!!
  if ( id < 0 ) return;

  // Mark conn handle as invalid
  prphs[id].conn_handle = BLE_CONN_HANDLE_INVALID;

  Serial.print(prphs[id].name);
  Serial.println(" disconnected!");
}



int get_client_index(uint16_t conn_hdl){
  for(int i=0; i < 2; i++){
    if(prphs[i].conn_handle == conn_hdl) {
      return i;
    }
  }
  return -1; // Not found
}


/**
 * Callback invoked when BLE UART data is received
 * @param uart_svc Reference object to the service where the data 
 * arrived.
 */
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  // uart_svc is prphs[conn_handle].bleuart
  uint16_t conn_handle = uart_svc.connHandle();
  int client_index = get_client_index(conn_handle);
  int id = findConnHandle(conn_handle);
  prph_info_t* peer = &prphs[id];
  
  // Print sender's name
  // Serial.printf("[From %s]: ", peer->name);
  // Serial.printf("[From %d]: ", id);
    
       while (uart_svc.available()) {
        if(!receiving[client_index]){
          uint8_t header;
          if(!uart_svc.read(&header, 1)) return;  // Read the 1-byte header
          if(header != 0x02) return;  // Unexpected, ignore

          uint8_t count_bytes[2];
          if(uart_svc.read(count_bytes, 2) != 2) return;
          uint16_t count = count_bytes[0] | (count_bytes[1] << 8);

          expected_bytes[client_index] = count * 3;
          received_bytes[client_index] = 0;
          receiving[client_index] = true;
        }

      // Read remaining data (if any)
      while (uart_svc.available() && received_bytes[client_index] < expected_bytes[client_index]) {
        uint8_t b;
        uart_svc.read(&b, 1);
        rx_buffer[client_index][received_bytes[client_index]++] = b;
      }
      // Done receiving?
      if (received_bytes[client_index] >= expected_bytes[client_index]) {
        receiving[client_index] = false;
        received_from_client[client_index] = true;

        Serial.printf("[From %d]: ", id);

        // Decode and print
        int count = expected_bytes[client_index] / 3;
        // Serial.print("Data: ");
        for (int i = 0; i < count; i++) {
          uint32_t val = rx_buffer[client_index][i * 3 + 0]
                      | (rx_buffer[client_index][i * 3 + 1] << 8)
                      | (rx_buffer[client_index][i * 3 + 2] << 16);
          Serial.print(val, HEX);
          Serial.print(" ");
        }
        Serial.println();
      }
    }
  // Check if all peripherals have sent data
  if (all_clients_ready()) {
    ready_to_sync = true;
    // Serial.println("Clients Ready for Sync.");
  }
}



/**
 * Helper function to send a string to all connected peripherals
 */
void sendAll(const char* str)
{
  Serial.print("[Send to All]: ");
  Serial.println(str);
  
  for(uint8_t id=0; id < BLE_MAX_CONNECTION; id++)
  {
    prph_info_t* peer = &prphs[id];

    if ( peer->bleuart.discovered() )
    {
      peer->bleuart.print(str);
    }
  }
}

void sendNoSerial(const char* str)
{
  for(uint8_t id=0; id < BLE_MAX_CONNECTION; id++)
  {
    prph_info_t* peer = &prphs[id];

    if ( peer->bleuart.discovered() )
    {
      peer->bleuart.print(str);
    }
  }
}

bool all_clients_ready(){
  for(int i=0; i< 2; i++){
    if(!received_from_client[i]) return false;
  }
  return true;
}

void reset_rx_flags() {
  for (int i = 0; i < 2; i++){
    received_from_client[i] = false;
  }
}

// void send_sync_advertising(){
//   Bluefruit.Advertising.stop();  // Make sure it's stopped
//   Bluefruit.Advertising.clearData();  // Clear previous payload

//   // Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED);
//   // Bluefruit.Advertising.setInterval(160);  // 100ms (160 * 0.625 ms)
//   // Bluefruit.Advertising.setFastTimeout(1);  // Timeout after 1 second if not manually stopped

//   Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
//   Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, sync_payload, sizeof(sync_payload));

//   Bluefruit.Advertising.start(50); // Advertise for 100ms
// }

void send_sync_advertising(uint8_t bursts = 3, uint16_t burst_interval_ms = 10) {
  for (uint8_t i = 0; i < bursts; i++) {
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.clearData();
    // Manufacturer-specific sync payload
    const uint8_t sync_data[] = { 'S', 'Y', 'N', 'C' };
    Bluefruit.Advertising.addManufacturerData(sync_data, sizeof(sync_data));
    // Set advertising mode to non-connectable scannable
    Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED);
    Bluefruit.Advertising.setFastTimeout(1); // Not used, but required by Adafruit API
    Bluefruit.Advertising.start(0); // Start advertising indefinitely
    delay(5);  // Wait 5–10 ms for the radio to send one advertising packet
    Bluefruit.Advertising.stop();  // Stop immediately after one packet
    delay(burst_interval_ms);  // Wait before next SYNC burst
  }
}

void loop()
{
  // First check if we are connected to any peripherals
  if ( Bluefruit.Central.connected() )
  {
    // default MTU with an extra byte for string terminator
    char buf[20+1] = { 0 };
    int count;
    
    // Read from HW Serial (normally USB Serial) and send to all peripherals
    if ( Serial.readBytes(buf, sizeof(buf)-1) )
    {
      sendAll(buf);
      STATE = 1;
    }
    else if(STATE == 1){
      //delay(2);
      // count = bleuart.read(buf, sizeof(buf));
      // Serial.write(buf, count);
      while(Bluefruit.Central.connected()){
        if(ready_to_sync){
          // Serial.println("Syncing peripherals...");
          sendNoSerial("SYNC");
          // send_sync_advertising(); // This triggers the next round on all peripherals
          ready_to_sync = false;
          reset_rx_flags();
          // Serial.println("Peripherals Synced!");
        }

        delay(1);
      }
    }
  }
}

/**
 * Find the connection handle in the peripheral array
 * @param conn_handle Connection handle
 * @return array index if found, otherwise -1
 */
int findConnHandle(uint16_t conn_handle)
{
  for(int id=0; id<BLE_MAX_CONNECTION; id++)
  {
    if (conn_handle == prphs[id].conn_handle)
    {
      return id;
    }
  }

  return -1;  
}

/**
 * Software Timer callback is invoked via a built-in FreeRTOS thread with
 * minimal stack size. Therefore it should be as simple as possible. If
 * a periodically heavy task is needed, please use Scheduler.startLoop() to
 * create a dedicated task for it.
 * 
 * More information http://www.freertos.org/RTOS-software-timer.html
 */
void blink_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;

  // Period of sequence is 10 times (1 second). 
  // RED LED will toggle first 2*n times (on/off) and remain off for the rest of period
  // Where n = number of connection
  static uint8_t count = 0;

  if ( count < 2*connection_num ) digitalToggle(LED_RED);
  if ( count % 2 && digitalRead(LED_RED)) digitalWrite(LED_RED, LOW); // issue #98

  count++;
  if (count >= 10) count = 0;
}
