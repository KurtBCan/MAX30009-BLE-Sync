#include <bluefruit.h>
#include <set>
#include <RTClib.h>

RTC_DS3231 rtc;

#define SYNC_PULSE_PIN D2
#define MANUFACTURER_ID 0x0606

// UUIDs must match those on the peripheral
#define SERVICE_UUID   "12145678-1234-5678-1234-56789abcdef0"
#define NOTIFY_UUID    "87654321-4321-6789-4321-fedcba987654"
#define WRITE_UUID     "11112222-3333-4444-5555-666677778888"

// Sample service and characteristics UUIDs
BLEService sampleService = BLEService(SERVICE_UUID);
BLECharacteristic sampleChar = BLECharacteristic(NOTIFY_UUID);
BLECharacteristic commandChar = BLECharacteristic(WRITE_UUID);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

// Track conn_handles that sent data
int connection_num = 0;
std::set<uint16_t> dataReceivedHandles;  
bool allDataReceived = false;

// unsigned long  advStartTime = 0;  // Adv Start Time using millis()
bool advertising = false;
static bool sentFirst = false;

// Initialize hardware timer
NRF_TIMER_Type *timer = NRF_TIMER2;
int connectionCount = 0;
bool timerStarted = false;

void startTimer() {
  timer->TASKS_STOP = 1;
  timer->MODE = TIMER_MODE_MODE_Timer;
  timer->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  timer->PRESCALER = 0; // 16 MHz -> 1 tick = 62.5 ns
  timer->TASKS_CLEAR = 1;
  timer->TASKS_START = 1;
  timerStarted = true;
}

void stopTimer() {
  timer->TASKS_STOP = 1;
  timerStarted = false;
}

uint32_t getTimestamp() {
  timer->TASKS_CAPTURE[0] = 1;
  return timer->CC[0]; // time in µs
}

void setup() {
  // Serial Init
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  if (!rtc.begin()) {
    Serial.println("RTC not found!");
    while (1);
  }
  
  ////////////////////
  // BlueTooth Init //
  ////////////////////
  
  // // Setup the BLE LED to be enabled on CONNECT
  // // Note: This is actually the default behaviour, but provided
  // // here in case you want to control this LED manually via PIN 19
  // Bluefruit.autoConnLed(true);

  // Initialize Bluefruit with max concurrent connections as Peripheral = 2, Central = 0
  Bluefruit.begin(2, 0);
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.setTxPower(8);               // Max power
  Bluefruit.setName("MaxCtrl");


  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Seeed Studio Xiao");
  bledis.setModel("Xiao nRF52840");
  // Bluefruit.setName("Periph");
  // Bluefruit.setName("Periph2");
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

    // Setup the Heart Rate Monitor service using
  // BLEService and BLECharacteristic classes
  Serial.println("Configuring the BioZ Data Service");
  setupService();

  
  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  startAdv();
  startTimer(); // Start timer when advertising begins

  Serial.println("Ready Player One!!!");
  Serial.println("\nAdvertising");

  delay(2000);
}


////////////////////////////////////
// BlueTooth Advertising Function //
////////////////////////////////////

#define ADV_HANDLE 0

// Global advertising buffer
uint8_t adv_data[31];
ble_gap_adv_data_t raw_adv_data = {
  .adv_data = {
    .p_data = adv_data,
    .len = 0
  },
  .scan_rsp_data = {
    .p_data = NULL,
    .len = 0
  }
};

void startAdv(void)
{
  // Clear adv data (just in case)
  Bluefruit.Advertising.clearData();
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include HRM Service UUID
  Bluefruit.Advertising.addService(sampleService);

  // Include Name
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  // Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setInterval(32, 32);  // Fixed ~20ms interval
  // Bluefruit.Advertising.setInterval(20, 20);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  
  ble_gap_ch_mask_t     channel_mask;  

  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  

}



void setupService(void)
{
  sampleService.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!


  // Configure the BioZ Data characteristic
  sampleChar.setProperties(CHR_PROPS_NOTIFY);  // Set as Notify
  sampleChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  sampleChar.setFixedLen(20);
  sampleChar.begin();  // No need for CCCD callback here, we're just notifying

  // Configure the Command characteristic
  commandChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  commandChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  commandChar.setWriteCallback(write_callback);
  commandChar.setFixedLen(20);
  // commandChar.enableNotify();
  commandChar.begin();
  // commandChar.write8(2);    // Set the characteristic to 'Wrist' (2)
}


// ************************************************************
// MAIN
// ************************************************************
static uint32_t lastSendTime = 0;

void loop() {

  uint32_t now = millis();
  // Forward user commands over BLE to peripheral
  if (Serial.available() > 0) 
  {
    String msg = Serial.readStringUntil('\n');
    msg.trim();  // Remove newline, carriage return, etc.
    if ( (strcmp(msg.c_str(), "START") == 0) || (strcmp(msg.c_str(), "SYNC") == 0) )
    {
      Serial.println("START or SYNC received");
      startSyncAdv(rtc.now().unixtime());
      lastSendTime = now;
      advertising = true;
    }
    else if (msg.length()) 
      {
        // Ensure that we only send the relevant part of the message
        uint8_t msgData[msg.length() + 1];  // +1 for null-terminator
        msg.toCharArray((char*)msgData, msg.length() + 1);  // Copy string with null-terminator

        // Send Notify to every connected device
        for (uint8_t conn_idx = 0; conn_idx < connection_num; conn_idx++) 
        {
          uint16_t conn_handle = conn_idx;
          if (sampleChar.notify(conn_handle, msgData, msg.length() + 1)) // Message + null terminator
          {
            Serial.print("Sent notify to conn_handle ");
            Serial.print(conn_handle);
            Serial.print(": ");
            Serial.println(msg);
          } else {
            Serial.print("Failed to send notify to conn_handle ");
            Serial.println(conn_handle);
          }
        }
      }

  }
}

// Scan callback to handle peripherals when discovered
void scan_callback(ble_gap_evt_adv_report_t* report) {
  Serial.println("Peripheral found, connecting...");

  Bluefruit.Central.connect(report);  // Connect to the first available peripheral
}


void connect_callback(uint16_t conn_handle)
{
  double timestamp = getTimestamp()* 0.0625f; //ticks times 62.5 (ns) divided by 1000 µs
  connectionCount++;

  if (connectionCount == 1) {
    Serial.print("First connection at: ");
    Serial.print(timestamp, 3); // 3 Decimal place time µs
    Serial.println(" µs");
  }
  else if (connectionCount == 2) {
    Serial.print("Second connection at: ");
    Serial.print(timestamp, 3); // 3 Decimal place time µs
    Serial.println(" µs");
    stopTimer();
  }

  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

  connection_num++;

  // IMPORTANT: Start advertising again for more devices!
  if (  connection_num < 2) {
    Bluefruit.Advertising.start(0);  // Keep advertising for next device
    Serial.println("Advertising for more connections...");
  }
  else
  {
    Bluefruit.Advertising.stop();
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  Serial.println("Advertising!");

  conn_handle = BLE_CONN_HANDLE_INVALID;

  connection_num--;

  Bluefruit.Advertising.start(0);
}

void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value)
{
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    Serial.print(cccd_value);
    Serial.println("");

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == sampleChar.uuid) {
        if (chr->notifyEnabled(conn_hdl)) {
            Serial.println("BioZ Measurement 'Notify' enabled");
            // STATE = STATE_START;
        } else {
            Serial.println("BioZ Measurement 'Notify' disabled");
        }
    }
}

// Handle incoming WRITE data (previously notifyCallback)
void write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  Bluefruit.Advertising.stop();
  advertising = false;
  
  int total_bytes = 0;
  static uint8_t myData[200]; // Adjust size if necessary

  Serial.print("Write received from conn_handle ");
  Serial.print(conn_hdl);
  Serial.print(": ");
  Serial.print(len);
  Serial.print(" bytes: ");

  // Copy incoming data safely
  if (len > sizeof(myData)) len = sizeof(myData);  // Clamp
  memcpy(myData, data, len);

  // Process and display the received data
  Serial.print("Count: ");
  Serial.print(myData[0] >> 8 | myData[1] & 0xFF);
  Serial.println(" samples.");

  total_bytes = 2;
  for (uint16_t i = 0; i < 6; i++) {
    Serial.print(myData[total_bytes++] | (myData[total_bytes++] << 8) | (myData[total_bytes++] << 16), HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Track who sent data
  dataReceivedHandles.insert(conn_hdl);

  // Check if all connected devices have sent data
  if (dataReceivedHandles.size() == 2) {
    allDataReceived = true;
    dataReceivedHandles.clear();
    Serial.println("Data Received!");
  }
}
