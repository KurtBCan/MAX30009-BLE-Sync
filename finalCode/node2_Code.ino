// External libraries
#include <arduino.h>
#include <SPI.h> // SPI communication to MAX
#include <string.h>
// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include <RTClib.h>
// Xiao Added Libraries
#include <bluefruit.h>
#include <Wire.h>
// #include <Adafruit_TinyUSB.h>
#include <ctype.h>  // Needed for `isspace()`

// Custom libraries
//#include "ALRI_DUE_PINS.h"          // DUE pins for ALRI PCB
#include "MAX_REGISTER_ADDRESSES.h" // Register addresses of MAX
#include "MAX_BIOZ_VALUES.h"        // Macros with defined bit patterns for BIOZ operation of MAX30001
#include "MAX_EGG_VALUES.h"         // Macros with defined bit patterns for EGG operation of MAX

#define MAX_PRPH_CONNECTION   2
uint8_t connection_count = 0;
uint8_t dataSendPriority = 1;


// ***********Global Variables*********
volatile unsigned long lastSyncTime = 0;  // Stores last received sync signal time
const int ledPin = LED_BUILTIN; // pin to use for the LED
DateTime startBurst;
DateTime endBurst;

#define BUFFER_SIZE 64  // Max expected command length
char receivedCommand[BUFFER_SIZE] = "";  // Stores incoming command
bool commandReceived = false;  // Flag to indicate a new command received

RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

int priority = 1;

// Define Xiao SPI pins
#define CS 7
#define SPI_SCK   PIN_SPI_SCK   // Standard SPI SCK pin
#define SPI_MISO  PIN_SPI_MISO  // Standard SPI MISO pin
#define SPI_MOSI  PIN_SPI_MOSI  // Standard SPI MOSI pin
#define SPI_SS    7    // Standard SPI Chip Select pin



// Define MAX30009/Xiao INT Pin Connection
#define INT_PIN 2 // Use an interrupt-capable pin (D2)
#define TRIG_PIN 1
#define extLED 0

#define AFE_FIFO_SIZE			256

#define NUM_BYTES_PER_SAMPLE	3
#define NUM_SAMPLES_PER_INT		129	/* number of samples in FIFO that generates a FIFO_A_FULL interrupt */

uint8_t STATE = 0;
#define STATE_WAIT  0
#define STATE_START 1
#define STATE_SYNC  2
#define STATE_DATA 3
#define STATE_WAITFORSYNC  4
#define STATE_STOP  5

volatile uint8_t TURN = 1;

// Global Variable
//uint8_t gReadBuf[NUM_SAMPLES_PER_INT*NUM_BYTES_PER_SAMPLE];	// array to store register reads
uint8_t errCnt;
uint8_t StatusReg1, StatusReg2;
int intPinState;
int iterationCnt;
int dataError = 0;

// MAX30009 Macros

// SCLK Frequency for MAX30001 can be 0 - 12 MHz
#define SCLK_1M 1000000  // 1 MHz

// Read/Write Bits for MAX30001
#define WRITE_BIT 0
#define READ_BIT 1

// Mapping for getMeasurement function
#define GET_RESISTANCE 0
#define GET_REACTANCE 1

// Create a setting with our SPI clock rate, data order, and spi mode.
// Commands are msb first and SPI_Mode0/3 are both compatible with the MAX30001
#define SPI_DEFINE SPISettings(SCLK_1M, MSBFIRST, SPI_MODE0)

// ************************************************************
// GLOBAL VARIABLES
// ************************************************************
volatile int measCount = 0;                 // Tracks number of completed measurements during BIOZ interrupt reads
volatile int32_t BiozFifoBurstValues[255] = {0};    // Array to store valid BIOZ reads during measurement
volatile int32_t savedBiozResistance[9][6]; // Array for storing BIOZ read data after measurement
                                            // First index tracks the frequency
                                            // Second index tracks the repeated measurement number
volatile int32_t savedBiozReactance[9][6];

volatile uint16_t FIFOcount = 0;

volatile uint8_t rxByte[3];	// array to store register reads

volatile bool interruptFlag = false; // Flag to indicate interrupt
volatile bool syncedFlag = true; // Flag to indicate synchronization

const uint8_t headerMarker[2] = { 0xAA, 0x55 }; // Unique header

#define SERVICE_UUID   "12145678-1234-5678-1234-56789abcdef0"
#define NOTIFY_UUID    "87654321-4321-6789-4321-fedcba987654"
#define WRITE_UUID     "11112222-3333-4444-5555-666677778888"

// Define BLE service and characteristics for multiple peripherals
#define MAX_PERIPHERALS 1  // Set to 1 for a single peripheral

// Define BLE service and characteristics
BLEClientService        sampleService(SERVICE_UUID);
BLEClientCharacteristic sampleChar(NOTIFY_UUID);
BLEClientCharacteristic commandChar(WRITE_UUID);

bool transmitting = true;

//////////////////////////////////////////////////////////////////////////////

static uint8_t notifyBuffer[244];


//////////  NEW SYNC METHOD INIT //////////
// #define SYNC_PULSE_PIN D1
#define MANUFACTURER_ID 0x0606

bool firstSync = false;

// Use NRF_TIMER1
#define TIMER NRF_TIMER1

// 10ms interval in microseconds
#define TIMER_INTERVAL_US 500000

bool timerIter1 = false;
bool timerStart = false;

void setup_timer_10ms() {
  // Stop timer if already running
  TIMER->TASKS_STOP = 1;
  TIMER->TASKS_CLEAR = 1;

  // Set 16-bit timer, prescaler 4 = 1MHz tick (1us per tick)
  TIMER->MODE = TIMER_MODE_MODE_Timer;
  TIMER->PRESCALER = 4; // 1MHz
  TIMER->BITMODE = TIMER_BITMODE_BITMODE_16Bit;

  // Set compare value for 10,000us (10ms)
  TIMER->CC[0] = TIMER_INTERVAL_US;

  // Enable interrupt on COMPARE[0]
  TIMER->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;

  // Use short to automatically clear and restart the timer on COMPARE[0] match
  TIMER->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled;

  // Attach the IRQ handler
  NVIC_SetPriority(TIMER1_IRQn, 1); // Set low priority
  NVIC_EnableIRQ(TIMER1_IRQn);

  // Start timer
  // TIMER->TASKS_START = 1;
}

// Interrupt handler
extern "C" void TIMER1_IRQHandler(void) {
if (TIMER->EVENTS_COMPARE[0]) {
    TIMER->EVENTS_COMPARE[0] = 0;

    if (!timerStart) {
        TIMER->TASKS_STOP = 1;
        TIMER->TASKS_CLEAR = 1;   // Reset timer counter to zero
        return;
    }

    TIMER->TASKS_STOP = 1;
    TIMER->TASKS_CLEAR = 1;       // Clear so next start is from zero
    measurementSetup();
}
}

void setup() {
  // Init Serial communication
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb



  Serial.println("Starting Central");

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  Bluefruit.setName("Xiao Central2");

  // Initialize HRM client
  sampleService.begin();

  // Initialize client characteristics of HRM.

  // set up callback for receiving commands
  // commandChar.setWriteCallback(writeCallback);
  // Note: Client Char will be added to the last service that is begin()ed.
  commandChar.begin();

  sampleChar.setNotifyCallback(notifyCallback);
  sampleChar.begin();


  Bluefruit.setTxPower(8);               // Max power
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Filter only accept BioZ service
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  // Bluefruit.Scanner.restartOnDisconnect(true);
  // Bluefruit.Scanner.setInterval(160, 160); // 100% duty cycle, reliable sync (units of 0.625)
  Bluefruit.Scanner.filterUuid(sampleService.uuid);
  // Bluefruit.Scanner.useActiveScan(false);
  // Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds


   // --- Manually override SoftDevice scan parameters to use only channel 37 ---
  ble_gap_scan_params_t scan_params;
  memset(&scan_params, 0, sizeof(scan_params));  // Clear all fields

  scan_params.active        = 0;  // Passive scan
  scan_params.interval      = 160;  // 100 ms (160 × 0.625ms)
  scan_params.window        = 160;  // Scan window = interval for 100% duty cycle
  scan_params.timeout       = 0;    // Scan indefinitely
  scan_params.scan_phys     = BLE_GAP_PHY_1MBPS;
  scan_params.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;

  // Disable channel 38 and 39 (only scan on 37)
  // scan_params.channel_mask[0] = (1 << 1) | (1 << 2);  // Bit 1 = ch38, Bit 2 = ch39
  ble_gap_ch_mask_t     channel_mask;  

  static uint8_t scan_buffer[BLE_GAP_SCAN_BUFFER_EXTENDED_MIN];
  ble_data_t scan_buffer_data = {
    .p_data = scan_buffer,
    .len    = sizeof(scan_buffer)
  };

  uint32_t err = sd_ble_gap_scan_start(&scan_params, &scan_buffer_data);
  if (err != NRF_SUCCESS) {
    Serial.print("sd_ble_gap_scan_start failed: ");
    Serial.println(err, HEX);
    while (1);
  }


  Serial.println("Central scanning for peripherals...");



  //////////////
  // SPI Init //
  //////////////

  // pinMode(SPI_MISO, INPUT_PULLDOWN);  // Explicitly set pulldown
  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);
  pinMode(extLED, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  digitalWrite(LED_BUILTIN, LOW);  
  delay(100);
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);  // Ensure SS is high
  SPI.begin();

  // RTC Setup
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time to compile time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING); // Trigger on falling edge

  endCollection();
  // if the remote device wrote to the characteristic,
  // use the value to control the LED:

  Serial.println("BLEUART Available.");

  configureMAX_REGS(); // Configure REGS
  Serial.println("Regs Configured");

  digitalWrite(ledPin, HIGH);  // LED OFF

  digitalWrite(ledPin, LOW);  // LED ON

  setup_timer_10ms();

}

 

// ************************************************************
// MAIN
// ************************************************************

bool allConnected = false;

void loop() {

  switch (STATE){
    case STATE_WAIT:
      // delay(10);
      break;
    case STATE_START:

      iterationCnt = 0;
      dataError = 0;
      STATE = STATE_DATA;
      interruptFlag = false;
      break;
    case STATE_SYNC:
      sync();
      STATE = STATE_DATA;
      break;
    case STATE_DATA:
      while(!interruptFlag);
      if (interruptFlag) {
        interruptFlag = false; // Reset the flag
        // Begin Data Collection
        onAfeInt();
        if(dataError != 1){
          sendData();
          iterationCnt += 1;
          break;
        }
        else if (dataError == 1){
          interruptFlag = true;
          dataError = 0;
          break;
        }
      }
    case STATE_WAITFORSYNC:
      break;

    case STATE_STOP:
      endCollection();
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);  
      STATE = STATE_WAIT;
      break;

    // default:
    //   delayMicroseconds(1); 
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////
  // BlueTooth Helper Functions //
  ////////////////////////////////

// Scan callback
void scan_callback(ble_gap_evt_adv_report_t* report) {
  const uint8_t* data = report->data.p_data;
  uint8_t len = report->data.len;

  const uint8_t WHITELISTED_UUID[16] = {
    0xF0, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x14, 0x12
  };

  uint8_t index = 0;
  bool uuid_found = false;

  while (index < len) {
    uint8_t field_len = data[index];
    if (field_len == 0 || index + field_len >= len) {
      break;  // malformed or end of data
    }

    uint8_t ad_type = data[index + 1];

    if ((ad_type == 0x06 || ad_type == 0x07) && field_len == 17) {
      const uint8_t* adv_uuid = &data[index + 2];

      if (memcmp(adv_uuid, WHITELISTED_UUID, 16) == 0) {
        uuid_found = true;
        if (!timerStart) {
          timerStart = true;
          // NRF_PPI->CHENCLR = (1 << 0); // disable PPI channel 0 to stop hardware start trigger
        }
        // digitalWrite(TRIG_PIN, HIGH);
        STATE = STATE_START;
        Serial.println("UUID found...");
        // Start timer
        TIMER->TASKS_START = 1;
        Bluefruit.Central.connect(report);
        delay(30);
        break;  // found valid UUID, stop scanning further fields
      }
    }
    index += field_len + 1;
  }

  // if (!uuid_found) {
  //   // UUID not found anywhere in the advertisement — stop the timer once
  //   // TIMER->TASKS_STOP = 1;
  //   // TIMER->TASKS_CLEAR = 1;
  // }

  Serial.println("Measurement Setup Complete!");
  Bluefruit.Scanner.stop();
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  Serial.println("Connected");
  Serial.print("Discovering BioZ Service ... ");

  // If BioZ is not found, disconnect and return
  if ( !sampleService.discover(conn_handle) )
  {
    Serial.println("Found NONE");

    // disconnect since we couldn't find HRM service
    Bluefruit.disconnect(conn_handle);

    return;
  }

  // Once BioZ service is found, we continue to discover its characteristic
  Serial.println("Found it");
  
  Serial.print("Discovering BioZ Data characteristic ... ");
  if ( !sampleChar.discover() )
  {
    // Measurement chr is mandatory, if it is not found (valid), then disconnect
    Serial.println("not found !!!");  
    Serial.println("Data characteristic is mandatory but not found");
    Bluefruit.disconnect(conn_handle);
    return;
  }
  Serial.println("Found it");

  // Data is found, continue to look for option Command
  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.body_sensor_location.xml
  // Body Sensor Location is optional, print out the location in text if present
  Serial.print("Discovering Body Sensor Location characteristic ... ");
  if ( commandChar.discover() )
  {
    Serial.println("Found it");

  }else
  {
    Serial.println("Found NONE");
  }

  // Reaching here means we are ready to go, let's enable notification on measurement chr
  if ( sampleChar.enableNotify() )
  {
    Serial.println("Ready to send BioZ data");
    STATE = STATE_START;
  }else
  {
    Serial.println("Couldn't enable notify for BioZ Data. Increase DEBUG LEVEL for troubleshooting");
  }
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

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

  // Retry connection by restarting scanner
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.start(0); // scan indefinitely
}

// Handle incoming NOTIFY data (previously write_callback)
void notifyCallback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  // Convert raw data to a String and trim it
  String cmd = String((char*)data).substring(0, len);  // Adjust length here to avoid garbage data
  cmd.trim();  // Remove newline, carriage return, etc.

  uint16_t conn_handle = chr->connHandle();
  Serial.print("Notify received from conn_handle ");
  Serial.print(conn_handle);
  Serial.print(": ");
  Serial.println(cmd);

  if (cmd == "RESEND") {
    Serial.printf("Received resend request from controller (conn_handle: %d)\n", conn_handle);
    sendData();
    Serial.println("Data resent!");
    STATE = STATE_WAITFORSYNC;
  }
  else if (cmd == "START") {
    Serial.println("START command received!");
    STATE = STATE_START;
  }
  else if (cmd == "STOP") {
    Serial.println("STOP command received! Pausing data transmission.");
    STATE = STATE_STOP;
  }
  else if (cmd == "SYNC") {
    STATE = STATE_SYNC;
  }
  else if (cmd == "CONNECTED") {
    Serial.println("All Nodes Connected.");
    allConnected = true;
  }
  else {
    Serial.println("Unknown command:");
    Serial.println(cmd);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////


// HEX to uint8 Function
uint8_t hexToUint8(String hexString) {
  return (uint8_t) strtol(hexString.c_str(), NULL, 16);
}

void uint8ToHex(uint8_t number, char* hexString) {
  sprintf(hexString, "%02X", number);
}

void uint32ToHex(uint32_t number, char* hexString) {
  sprintf(hexString, "%06X", number);
}


// ************************************************************
// MAX 30009 Functions
// ************************************************************

void syncAllSteps(void){
    // Disable BIOZ_BG_EN, BIOZ_Q_EN, and BIOZ_I_EN
    writeRegister(REG_BIOZ_CONFIG_1, 0x70); 
    // Reset MAX and flush FIFO by setting SHDN, then clear SHDN
    writeRegister(REG_SYS_CONFIG_1, 0x02);
    writeRegister(REG_SYS_CONFIG_1, 0x00);
    delay(6); // Delay 6 ms
    // Program Config Registers
    writeRegister(REG_PIN_FUNC_CONFIG, PIN_FUNC_CONFIG);
    // readRegister(REG_PLL_CONFIG_1);
    writeRegister(REG_PLL_CONFIG_2, PLL_CONFIG_2);
    writeRegister(REG_PLL_CONFIG_3, PLL_CONFIG_3);
    writeRegister(REG_PLL_CONFIG_4, PLL_CONFIG_4);
    // writeRegister(REG_BIOZ_CONFIG_1,0x74);
    writeRegister(REG_BIOZ_CONFIG_2, BIOZ_CONFIG_2);
    writeRegister(REG_BIOZ_CONFIG_3, BIOZ_CONFIG_3);
    writeRegister(REG_BIOZ_CONFIG_4, BIOZ_CONFIG_4);
    writeRegister(REG_BIOZ_CONFIG_5, BIOZ_CONFIG_5);
    writeRegister(REG_BIOZ_CONFIG_6, BIOZ_CONFIG_6);
    writeRegister(REG_BIOZ_LW_THRSHLD, BIOZ_LW_THRSHLD);
    writeRegister(REG_BIOZ_HI_THRSHLD, BIOZ_HI_THRSHLD);
    writeRegister(REG_BIOZ_CONFIG_7, BIOZ_CONFIG_7);
    writeRegister(REG_BIOZ_MUX_CONFIG_1, BIOZ_MUX_CONFIG_1);
    writeRegister(REG_BIOZ_MUX_CONFIG_2, BIOZ_MUX_CONFIG_2);
    writeRegister(REG_BIOZ_MUX_CONFIG_3, BIOZ_MUX_CONFIG_3);
    writeRegister(REG_BIOZ_MUX_CONFIG_4, BIOZ_MUX_CONFIG_4);
    writeRegister(REG_LEAD_BIAS_CONFIG_1, LEAD_BIAS_CONFIG_1);

    // Program FIFO_A_FULL for INT Pin
    writeRegister(REG_FIFO_CONFIG_1,0xFA); // 6 Points triggers INT  
    writeRegister(REG_FIFO_CONFIG_1,0xE2); // 6 Points triggers INT  

    // Enable PLL  
    writeRegister(REG_PLL_CONFIG_1, PLL_CONFIG_1);
    // Wait for PLL to lock
    delay(10); //May need to change depending on PLL settings
    // Enable Bioz
    writeRegister(REG_BIOZ_CONFIG_1, 0x74);
    // Broadcast TIMING_SYS_RESET
    writeRegister(REG_SYS_SYNC, 0x80);

    return;
}

void sync(void){
  writeRegister(REG_SYS_SYNC, 0x80);
  return;
}

// Read specified register and return 8-bit value
//  Inputs: address of register to read (uint8)
//  Outputs: uint8 of register value
uint8_t readRegister(uint8_t addr)
{
    // SPI Communication Starts
    SPI.beginTransaction(SPI_DEFINE);
    digitalWrite(CS, LOW);
    // NOTE: Arduino Library Buffer Transfer doesn't work for this set-up
    // It transmits bytes in reverse order, meaning command word is sent last
    // if above bit shifts are applied
    SPI.transfer(addr);
    SPI.transfer(READ_BYTE);
    // Temporary 8bit holder for read values
    rxByte[0] = SPI.transfer(0xFF); // DONT CARE Transmitted Bits
    digitalWrite(CS, HIGH);
    SPI.endTransaction();
    // SPI Communication Ends

    return rxByte[0];
}

// Read register function OVERLOAD for BURST reads
//  If byteCount is greater than two bytes (two register read),
//  then assume FIFO burst read.
void readRegister(uint8_t addr, uint8_t byteCount)
{
    // SPI Communication Starts
    SPI.beginTransaction(SPI_DEFINE);
    digitalWrite(CS, LOW);
    // NOTE: Arduino Library Buffer Transfer doesn't work for this set-up
    // It transmits bytes in reverse order, meaning command word is sent last
    // if above bit shifts are applied
    SPI.transfer(addr);
    SPI.transfer(READ_BYTE);
    
    // Check if burst 
    if(byteCount == 2){
      // 2-Register BURST Read
      // for (int i = 0; i < byteCount; i++)
      // {
          // MAX 30009 returns 2 bytes, one for each reg
          rxByte[0] = SPI.transfer(0);
          rxByte[1] = SPI.transfer(1);
      // }
    }
    else{
      // Read BioZ BURST FIFO
      //  Modifies: BiozFifoBurstValues global
      for (int i = 0; i < FIFOcount; i++)
      {
          // MAX returns 24-bit word with 20 data bits and 4 tagging bits
          // Capture each 8-bit chunk and store into global array
          rxByte[0] = SPI.transfer(0xFF);
          rxByte[1] = SPI.transfer(0xFF);
          rxByte[2] = SPI.transfer(0xFF);
          // Save into global FIFOBurstValues array to reuse rxByte array
          BiozFifoBurstValues[i] = (rxByte[0] << 16) | (rxByte[1] << 8) | (rxByte[2]) & 0xFFFFFF;

          // rxByte = { 0 };
      }
    }
    digitalWrite(CS, HIGH);
    SPI.endTransaction();
    // SPI Communication Ends
    return;
}

// Write value to specified register
//  Inputs:
//    - uint8 address of register to write
//    - uint32 value to write into register
void writeRegister(uint8_t addr, uint8_t writeValue)
{

    SPI.beginTransaction(SPI_DEFINE);
    digitalWrite(CS, LOW);
    SPI.transfer(addr);
    SPI.transfer(WRITE_BYTE);
    SPI.transfer(writeValue);
    digitalWrite(CS, HIGH);
    SPI.endTransaction();
    return;
}

// Configure the MAX30009 for BIOZ measurements
void configureMAX_REGS()
{
    writeRegister(REG_BIOZ_CONFIG_1, 0x04);	// BIOZ_BG_EN
		writeRegister(REG_SYS_CONFIG_1, 0x00);	// clear SHDN
		writeRegister(REG_PLL_CONFIG_1, 0x00);	// clear PLL_EN
		writeRegister(REG_PLL_CONFIG_4, 0x00);	// clear REF_CLK_SEL
		delay(1000);
		writeRegister(REG_SYS_CONFIG_1, 0x01);	// RESET

    // Read status register to clear any remaining flags
    readRegister(REG_STATUS1,2);

    writeRegister(REG_FIFO_CONFIG_1, 0x7F);	// FIFO_A_FULL; assert A_FULL on NUM_SAMPLES_PER_INT samples   (AFE_FIFO_SIZE-NUM_SAMPLES_PER_INT)

    writeRegister(REG_SYS_CONFIG_1, SYS_CONFIG_1);
    writeRegister(REG_PIN_FUNC_CONFIG, PIN_FUNC_CONFIG);

    writeRegister(REG_PLL_CONFIG_1, PLL_CONFIG_1);
    writeRegister(REG_PLL_CONFIG_2, PLL_CONFIG_2);
    writeRegister(REG_PLL_CONFIG_3, PLL_CONFIG_3);
    writeRegister(REG_PLL_CONFIG_4, PLL_CONFIG_4);

    writeRegister(REG_BIOZ_CONFIG_1, BIOZ_CONFIG_1);
    writeRegister(REG_BIOZ_CONFIG_2, BIOZ_CONFIG_2);
    writeRegister(REG_BIOZ_CONFIG_3, BIOZ_CONFIG_3);
    writeRegister(REG_BIOZ_CONFIG_4, BIOZ_CONFIG_4);
    writeRegister(REG_BIOZ_CONFIG_5, BIOZ_CONFIG_5);
    writeRegister(REG_BIOZ_CONFIG_6, BIOZ_CONFIG_6);
    writeRegister(REG_BIOZ_LW_THRSHLD, BIOZ_LW_THRSHLD);
    writeRegister(REG_BIOZ_HI_THRSHLD, BIOZ_HI_THRSHLD);
    writeRegister(REG_BIOZ_CONFIG_7, BIOZ_CONFIG_7);

    writeRegister(REG_BIOZ_MUX_CONFIG_1, BIOZ_MUX_CONFIG_1);
    writeRegister(REG_BIOZ_MUX_CONFIG_2, BIOZ_MUX_CONFIG_2);
    writeRegister(REG_BIOZ_MUX_CONFIG_3, BIOZ_MUX_CONFIG_3);
    writeRegister(REG_BIOZ_MUX_CONFIG_4, BIOZ_MUX_CONFIG_4);

    writeRegister(REG_LEAD_BIAS_CONFIG_1, LEAD_BIAS_CONFIG_1);

    writeRegister(REG_INT_EN_1, INT_EN_1);
    writeRegister(REG_INT_EN_2, INT_EN_2);
    // Read status register to clear any remaining flags
    readRegister(REG_STATUS1,2);
}



void measurementSetup(void)
{
    // 1. Write default values to registers
    readRegister(REG_BIOZ_CONFIG_1);
    //delayMicroseconds(5);
    readRegister(REG_PLL_CONFIG_4);
    //delayMicroseconds(5);
    readRegister(REG_PLL_CONFIG_1);
    //delayMicroseconds(5);
    readRegister(REG_BIOZ_CONFIG_1);
    //delayMicroseconds(5);
    writeRegister(REG_FIFO_CONFIG_1,0xFA); // 6 Points triggers INT
    // writeRegister(REG_FIFO_CONFIG_1,0xE2); // 30 Points triggers INT 

    // writeRegister(REG_FIFO_CONFIG_1,0x80); // 128 Points triggers INT
    //delayMicroseconds(5);
    readRegister(REG_BIOZ_CONFIG_1);
    //delayMicroseconds(5);
    // writeRegister(REG_BIOZ_CONFIG_1,0xF4);
    writeRegister(REG_BIOZ_CONFIG_1,0x74);
    //delayMicroseconds(5);
    readRegister(REG_FIFO_CONFIG_2);
    //delayMicroseconds(5);
    writeRegister(REG_FIFO_CONFIG_2,0x1A);  // Flush FIFO
    //delayMicroseconds(5);
    readRegister(REG_SYS_CONFIG_1);
    //delayMicroseconds(5);
    writeRegister(REG_SYS_CONFIG_1,0x00);
    //delayMicroseconds(5);
    // writeRegister(REG_BIOZ_CONFIG_1,0xF5);
    writeRegister(REG_BIOZ_CONFIG_1,0x77);
    //delayMicroseconds(5);
    readRegister(REG_FIFO_CONFIG_2);
    //delayMicroseconds(5);
    writeRegister(REG_FIFO_CONFIG_2,0x1A); // Flush FIFO
    //delayMicroseconds(5);
    readRegister(REG_STATUS1,2);
    //delayMicroseconds(5);

    // Setup Finished
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Interrupt Service Routine (ISR)
void handleInterrupt(void) {
    interruptFlag = true; // Set the flag for loop()
}


void endCollection(void)
{
      // End Collection
    readRegister(REG_BIOZ_CONFIG_1);
    //delayMicroseconds(5);
    // writeRegister(REG_BIOZ_CONFIG_1,0xF4);
    writeRegister(REG_BIOZ_CONFIG_1,0x74);
    //delayMicroseconds(5);
    readRegister(REG_FIFO_CONFIG_2);
    //delayMicroseconds(5);
    writeRegister(REG_FIFO_CONFIG_2,0x1A);
    //delayMicroseconds(5);
    readRegister(REG_SYS_CONFIG_1);
    //delayMicroseconds(5);
    writeRegister(REG_SYS_CONFIG_1,0x02);
    //delayMicroseconds(5);
    // writeRegister(REG_BIOZ_CONFIG_1,0xF5);
    writeRegister(REG_BIOZ_CONFIG_1,0x77);
    //delayMicroseconds(5);
    readRegister(REG_FIFO_CONFIG_2);
    //delayMicroseconds(5);
    writeRegister(REG_FIFO_CONFIG_2,0x1A);
    //delayMicroseconds(5);
}

// Send data function
void sendData() {
  uint16_t temp_count = FIFOcount;
  uint8_t header = 0x02;
  int total_bytes = 0;

  if ( Bluefruit.connected() ) {
    uint8_t packet[20] = {0};
    // packet[0] = header;
    packet[0] = temp_count >> 8;
    packet[1] = temp_count & 0xFF;
    total_bytes = 2;

    for (int i = 0; i < FIFOcount; i++) {
      uint32_t val = BiozFifoBurstValues[i] & 0xFFFFFF;
      packet[total_bytes++] = val & 0xFF;
      packet[total_bytes++] = (val >> 8) & 0xFF;
      packet[total_bytes++] = (val >> 16) & 0xFF;
    }
    if (commandChar.write(packet, sizeof(packet)) ){
      Serial.println("BioZ Data Sent!");
    }
    else{
      Serial.println("ERROR: Notify not set in the CCCD or not connected!");
    }

    // Clear your data array
    memset((void*)BiozFifoBurstValues, 0, sizeof(BiozFifoBurstValues)); 
  }
}


// ***************************************************
// On AFE Interrupt Pin Signal Goes LOW (from Example Source Code)
// ***************************************************

void onAfeInt(void)	// call this on AFE interrupt
{
  // READ and CLEAR all status registers
  readRegister(REG_STATUS1, 2);
  //delayMicroseconds(5);
  if (!(rxByte[0] & 0b1000)){	// check A_FULL bit 
    Serial.println("ERROR: A_FULL bit OFF");
    Serial.print("Expected:");
    Serial.println("0x8X or higher.");
    Serial.print("Received: 0x");
    Serial.println(rxByte[0], HEX);
    dataError = 1;
		return;
  }

  // Check count value available for FIFO Burst Read
  readRegister(REG_FIFO_CTR_1, 2);
  FIFOcount = rxByte[1];
  if(FIFOcount < 6){
    dataError = 1;
    return;
  }
  Serial.println(FIFOcount);
  readRegister(REG_FIFO_DATA,FIFOcount);

  return;
}
