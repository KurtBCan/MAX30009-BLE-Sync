#include <bluefruit.h>

// UUIDs must match those on the peripheral
#define SERVICE_UUID   "12145678-1234-5678-1234-56789abcdef0"
#define NOTIFY_UUID    "87654321-4321-6789-4321-fedcba987654"
#define WRITE_UUID     "11112222-3333-4444-5555-666677778888"

// Sample characteristics UUIDs
BLEService sampleService = BLEService(SERVICE_UUID);
BLECharacteristic sampleChar = BLECharacteristic(NOTIFY_UUID);
BLECharacteristic commandChar = BLECharacteristic(WRITE_UUID);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

int connection_num = 0;

// BLEAdvertising syncAdv;  // create a second advertiser

// void sendSyncPulse() {
//   // Stop previous sync adv if running
//   syncAdv.stop();

//   // Clear old payload
//   syncAdv.clearData();

//   // Add flags and a manufacturer-specific payload
//   syncAdv.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  
//   uint8_t syncPayload[] = {'S', 'Y', 'N', 'C', 0x01}; // or use a counter if needed
//   syncAdv.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, syncPayload, sizeof(syncPayload));

//   // Non-connectable, non-scannable (true, true)
//   syncAdv.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED);

//   // Use a short duration (e.g., 200ms)
//   syncAdv.setInterval(160);  // ~100ms
//   syncAdv.start(1);  // Advertise for 1 second max
// }


void setup() {
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  
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


  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Seeed Studio Xiao");
  bledis.setModel("Xiao nRF52840");
  Bluefruit.setName("Periph");
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

  Serial.println("Ready Player One!!!");
  Serial.println("\nAdvertising");
}


////////////////////////////////////
// BlueTooth Advertising Function //
////////////////////////////////////


void startAdv(void)
{
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
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
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

  // uint8_t sampleChardata[2] = {0b00000110, 0x40}; // Set the characteristic to use 8-bit values, with the sensor connected and detected
  // sampleChar.write(sampleChardata, 2);

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

void loop() {
  // Forward user commands over BLE to peripheral
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();  // Remove newline, carriage return, etc.
    if (msg.length()) 
    {
      // Ensure that we only send the relevant part of the message
      uint8_t msgData[msg.length() + 1];  // +1 for null-terminator
      msg.toCharArray((char*)msgData, msg.length() + 1);  // Copy string with null-terminator

      // uint8_t data[20];
      // size_t len = msg.length();
      // if (len > sizeof(data)) len = sizeof(data); // Clamp if longer than 20 bytes
      // memcpy(data, msg.c_str(), len);

      // Send Notify to every connected device
      for (uint8_t conn_idx = 0; conn_idx < connection_num; conn_idx++) {
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

  // conn_handle = BLE_CONN_HANDLE_INVALID;

  connection_num--;
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
  int total_bytes = 0;
  static uint8_t myData[20]; // Adjust size if necessary

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
}



// #include <bluefruit.h>
// #include <string.h>

// // UUIDs must match those on the peripheral
// #define SERVICE_UUID "12145678-1234-5678-1234-56789ABCDEF0"
// #define NOTIFY_UUID "87654321-4321-6789-4321-fedcba987654"
// #define WRITE_UUID "11112222-3333-4444-5555-666677778888"

// // Define BLE service and characteristics
// // BLEClientService        sampleService(SERVICE_UUID);
// // BLEClientCharacteristic sampleChar(NOTIFY_UUID);
// // BLEClientCharacteristic commandChar(WRITE_UUID);

// // Struct containing peripheral info
// typedef struct
// {
//   char name[16+1];

//   uint16_t conn_handle;

//   // Each prph need its own bleuart client service
//   BLEClientService sampleService;
//   BLEClientCharacteristic sampleChar;
//   BLEClientCharacteristic commandChar;
// } prph_info_t;


// prph_info_t prphs[2];

// void setup() {
//   Serial.begin(115200);
//   while ( !Serial ) delay(10);   // for nrf52840 with native usb

//   Serial.println("Starting Central");

//   // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
//   Bluefruit.begin();
//   Bluefruit.setName("Xiao Central");

//   // Init peripheral pool
//   for (uint8_t idx = 0; idx < 2; idx++) {
//     prphs[idx].conn_handle = BLE_CONN_HANDLE_INVALID;

//     prphs[idx].sampleService = BLEClientService(SERVICE_UUID);
//     prphs[idx].sampleChar = BLEClientCharacteristic(NOTIFY_UUID);
//     prphs[idx].commandChar = BLEClientCharacteristic(WRITE_UUID);

//     // Initialize client service first
//     prphs[idx].sampleService.begin();

//     // Then, initialize the client characteristics
//     prphs[idx].sampleChar.begin();
//     prphs[idx].commandChar.begin();

//     // Set up callback for receiving notification
//     prphs[idx].sampleChar.setNotifyCallback(notify_callback);
//   }

//   Bluefruit.setTxPower(4);               // Max power
//   Bluefruit.Central.setConnectCallback(connect_callback);
//   Bluefruit.Central.setDisconnectCallback(disconnect_callback);

//   // Start Central Scanning
//   Bluefruit.Scanner.setRxCallback(scan_callback);
//   Bluefruit.Scanner.restartOnDisconnect(true);
//   Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
//   Bluefruit.Scanner.filterUuid(SERVICE_UUID); // Ensure only the correct service UUID is filtered
//   Bluefruit.Scanner.useActiveScan(false);
//   Bluefruit.Scanner.start(0); // Scan forever

//   Serial.println("Central scanning for peripherals...");
// }


// void loop() {
//   // Forward user commands over BLE to peripheral
//   static String msg = "";
  
//   // Check for incoming serial data
//   if (Serial.available() > 0) {
//     char incomingByte = Serial.read();  // Read byte by byte
    
//     // Add byte to the message buffer
//     if (incomingByte == '\n') { // End of message (new line)
//       msg.trim();  // Clean up the message (remove spaces, newlines)
      
//       // Only proceed if message has content
//       if (msg.length()) {
//         for (int i = 0; i < 2; i++) {
//           // Make sure the connection is valid and characteristic is discovered
//           if (prphs[i].conn_handle != BLE_CONN_HANDLE_INVALID && prphs[i].commandChar.discovered()) {
//             prphs[i].commandChar.write((uint8_t*)msg.c_str(), msg.length());
//             Serial.print("Sent to peripheral: ");
//             Serial.println(msg);
//           }
//         }
//       }
      
//       // Clear message for next input
//       msg = "";
//     } else {
//       // Collect message bytes
//       msg += incomingByte;
//     }
//   }
// }

// // Scan callback
// void scan_callback(ble_gap_evt_adv_report_t* report) {
//   Serial.println("Peripheral found, connecting...");
  
//   // Make sure we're connecting to a valid peripheral (Optional: add more checks)
//   Bluefruit.Central.connect(report);
// }



// /**
//  * Callback invoked when an connection is established
//  * @param conn_handle
//  */
// void connect_callback(uint16_t conn_handle)
// {
//   prph_info_t* peer = &prphs[conn_handle];

//   Bluefruit.Connection(conn_handle)->getPeerName(peer->name, sizeof(peer->name)-1);

//   Serial.print("Connected to ");
//   Serial.println(peer->name);
//   Serial.print("Discovering BioZ Service ... ");


//   // If BioZ is not found, disconnect and return
//   if ( !peer->sampleService.discover(conn_handle) )
//   {
//     Serial.println("Found NONE");

//     // disconnect since we couldn't find HRM service
//     Bluefruit.disconnect(conn_handle);

//     return;
//   }

//   // Once BioZ service is found, we continue to discover its characteristic
//   Serial.println("Found it");
  
//   Serial.print("Discovering BioZ Data characteristic ... ");
//   if ( !peer->sampleChar.discover() )
//   {
//     // Measurement chr is mandatory, if it is not found (valid), then disconnect
//     Serial.println("not found !!!");  
//     Serial.println("Data characteristic is mandatory but not found");
//     Bluefruit.disconnect(conn_handle);
//     return;
//   }
//   Serial.println("Found it");

//   // Data is found, continue to look for option Command
//   // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.body_sensor_location.xml
//   // Body Sensor Location is optional, print out the location in text if present
//   Serial.print("Discovering Body Sensor Location characteristic ... ");
//   if ( peer->commandChar.discover() )
//   {
//     Serial.println("Found it");

//   }else
//   {
//     Serial.println("Found NONE");
//   }

//   // Reaching here means we are ready to go, let's enable notification on measurement chr
//   if ( peer->sampleChar.enableNotify() )
//   {
//     Serial.println("Ready to receive BioZ data");
//   }else
//   {
//     Serial.println("Couldn't enable notify for BioZ Data. Increase DEBUG LEVEL for troubleshooting");
//   }
// }

// /**
//  * Callback invoked when a connection is dropped
//  * @param conn_handle
//  * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
//  */
// void disconnect_callback(uint16_t conn_handle, uint8_t reason)
// {
//   (void) conn_handle;
//   (void) reason;

//   // Mark conn handle as invalid
//   prphs[conn_handle].conn_handle = BLE_CONN_HANDLE_INVALID;

//   Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
// }

// // Handle incoming notify data
// void notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
//   uint16_t conn_handle = chr->connHandle();  // <<< correct way
//   prph_info_t* peer = &prphs[conn_handle];
//   Serial.printf("[From %d]: ", conn_handle);

//   Serial.print("Notify received (");
//   // Serial.print(len);
//   Serial.print(" bytes): ");

//   int total_bytes = 0;
//   static uint8_t myData[20]; // or whatever size you expect

//   if (len > sizeof(myData)) len = sizeof(myData); // safety clamp
//   memcpy(myData, data, len); // copy incoming data into your own array

//   Serial.print("Count: ");
//   Serial.print(myData[0] >> 8 | myData[1] & 0xFF);
//   Serial.println(" samples.");
//   total_bytes = 2;

//   for (uint16_t i = 0; i < 6; i++) {
//     Serial.print( myData[total_bytes++] | (myData[total_bytes++] << 8) | (myData[total_bytes++] << 16), HEX);
//     Serial.print(" ");
//   }
//   Serial.println();

//   // Clear your local buffer
//   memset(myData, 0, sizeof(myData)); 
// }

