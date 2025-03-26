// External libraries
#include <arduino.h>
#include <SPI.h> // SPI communication to MAX
#include <string.h>
// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include "RTClib.h"
// Xiao Added Libraries
#include <bluefruit.h>
#include <Wire.h>
//#include <Adafruit_TinyUSB.h>
#include <ctype.h>  // Needed for `isspace()`

// Custom libraries
//#include "ALRI_DUE_PINS.h"          // DUE pins for ALRI PCB
#include "MAX_REGISTER_ADDRESSES.h" // Register addresses of MAX
#include "MAX_BIOZ_VALUES.h"        // Macros with defined bit patterns for BIOZ operation of MAX30001
#include "MAX_EGG_VALUES.h"         // Macros with defined bit patterns for EGG operation of MAX



// Modifiable message string
String setupComplete = "Setup Complete!";  // Change this string to any message you want
String startMessage = "Begin MAX1";    
String endMessage = "End MAX1";  

#define MAX_PRPH_CONNECTION   2
uint8_t connection_count = 0;
uint8_t dataSendPriority = 1;

// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble


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

// bioz_arduino.ino
// Capture BIOZ data using MAX30001 and Arduino

// How to use
// 1) Select Serial IO using Serial and SerialSpeed
// 2) Deploy to DUE and establish Serial connection (PC or Raspberry Pi)
// 3) Issue "START" command to begin BIOZ sweep
// 4) Wait about 5 seconds for sweep, then table of data will be printed
//  
//  Data contains both raw values (hex) and computed values (ohm)


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
#define STATE_STOP  3

uint8_t TURN = 1;

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

volatile uint16_t FIFOcount;

volatile uint8_t rxByte[3];	// array to store register reads

volatile bool interruptFlag = false; // Flag to indicate interrupt

const uint8_t headerMarker[2] = { 0xAA, 0x55 }; // Unique header

void setup() {
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb


  ////////////////////
  // BlueTooth Init //
  ////////////////////
  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Initialize Bluefruit with max concurrent connections as Peripheral = 2, Central = 0
  Bluefruit.begin(MAX_PRPH_CONNECTION, 0);
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

///////////////////////////////////////////////////////////////////////////////////
  // Speed up connections
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);  // Maximize MTU
  Bluefruit.Periph.setConnInterval(12, 24u);
  Bluefruit.setTxPower(8);
  ///////////////////////////////////////////////////////////////////////////////////

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Seeed Studio");
  bledis.setModel("Xiao nRF52840");
  //bledis.setName("Xiao nRF52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  bleuart.setRxCallback(rx_callback);

  // Set up and start advertising
  startAdv();

  //////////////
  // SPI Init //
  //////////////

  pinMode(SPI_MISO, INPUT_PULLDOWN);  // Explicitly set pulldown
  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);
  pinMode(extLED, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);  
  //Serial.begin(9600);
  delay(100);
  //Serial.println("Is Working?"); // Send serial message
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);  // Ensure SS is high
  SPI.begin();

  //SPI.setBitOrder(MSBFIRST);  // Set bit order
  //Serial.begin(9600);

  // Serial.println("Is Working Here?");


// RTC Setup
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, RTC needs a time reset");
  }

  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING); // Trigger on falling edge

  // Serial.println("Is Working Here 1?");

  // int32_t mtu = Bluefruit.requestMTU(247);
  // Serial.print("MTU Set to: ");
  // Serial.println(mtu);

}

  ////////////////////////////////////
  // BlueTooth Advertising Function //
  ////////////////////////////////////

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
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


// print a string to Serial Uart and all connected BLE Uart
void printAll(uint8_t* buf, int count)
{
  // Serial.write(buf, count);

  // Send to all connected centrals
  for (uint8_t conn_hdl=0; conn_hdl < MAX_PRPH_CONNECTION; conn_hdl++)
  {
    bleuart.write(conn_hdl, buf, count);
    // bleuart.write(buf, count, false);
  }
}


// ************************************************************
// MAIN
// ************************************************************

void loop() {


  // for(int loopVal= 1; loopVal<11; loopVal++){
  //   if( bleuart.available() ){
  //     // DateTime now = rtc.now(); 
  //     // printAll(setupComplete.c_str(), setupComplete.length());
  //     // bleuart.write(0, now.hour(), now.hour().length());
  //     // bleuart.write(0, now.minute(), now.minute().length());
  //     // bleuart.write(0, now.second(), now.second().length());
  //     Serial.println("BLEUART Available.");
  //     // Serial.println("Setup complete");
  //   }
  //   else{
  //     break;
  //   }
  // }
      // DateTime now = rtc.now();
      // Serial.print(now.hour(), DEC);
      // Serial.print(':');
      // Serial.print(now.minute(), DEC);
      // Serial.print(':');
      // Serial.println(now.second(), DEC);

  // while (bleuart.available()) {
  // processCommand();
  if (STATE == STATE_START){ 
    delay(2000);

    endCollection();
    // if the remote device wrote to the characteristic,
    // use the value to control the LED:

    Serial.println("BLEUART Available.");

    configureMAX_REGS(); // Configure REGS
    Serial.println("Regs Configured");
    // strncpy((char *)buf, setupComplete.c_str(), setupComplete.length());
    // printAll(buf, setupComplete.length());
    sendTextMessage("Setup Complete!");

    digitalWrite(ledPin, HIGH);  // LED OFF
    //digitalWrite(OUTPUT_PIN, LOW);  // LED OFF

    digitalWrite(ledPin, LOW);  // LED ON
      

    Serial.println("Sweeping BIOZ");

    char hexVal[3]; // Buffer to hold the hexadecimal string
    measurementSetup();

    iterationCnt = 0;
    dataError = 0;
    //while (dataError != 1)
    //while (iterationCnt != 1)
    while (1)
    {
      if(STATE == STATE_STOP){
        break;
      }
      if (interruptFlag) {
        interruptFlag = false; // Reset the flag
        // Begin Data Collection
        onAfeInt();
        // if(priority = 1){
        //  printDataBluetooth(buf);
        // sendDataBLE();
        // while ( TURN != 1 ) {
        //   delayMicroseconds(5);
        // }
        sendTextMessage("MAX 1 Start!");
        sendDataCount();
        sendData();
        sendTextMessage("MAX 1 Complete!");
        // }
        //printData(count);
        iterationCnt += 1;
      }

    }
  }
// }
  if(STATE == STATE_STOP){
    endCollection();
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);  
    STATE = STATE_WAIT;
  }

}

////////////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////
  // BlueTooth Helper Functions //
  ////////////////////////////////

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

  connection_count++;
  Serial.print("Connection count: ");
  Serial.println(connection_count);
  
  // // Keep advertising if not reaching max
  // if (connection_count < MAX_PRPH_CONNECTION)
  // {
  //   Serial.println("Keep advertising");
  //   Bluefruit.Advertising.start(0);
  // }
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

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

  connection_count--;
}


// RX Callback Function
// void rx_callback(uint16_t conn_handle) {
//     static uint8_t index = 0;

//     while (bleuart.available()) {
//         char c = bleuart.read();

//         if (index < BUFFER_SIZE - 1) {
//             receivedCommand[index++] = c;
//         }

//         if (c == '\n' || c == '\r') {
//             receivedCommand[index] = '\0';  // Null-terminate string
//             trimCommand(receivedCommand);   // Trim unwanted characters
//             index = 0;
//             commandReceived = true;
//         }
//     }
// }

void rx_callback(uint16_t conn_handle) {
    char buffer[32];
    int len = bleuart.read(buffer, sizeof(buffer) - 1);
    if (len > 0) {
        buffer[len] = '\0';  // Null-terminate
        String receivedStr = String(buffer);
        receivedStr.trim();  // Remove unwanted characters
        if (receivedStr == "START") {
            Serial.println("START command received!");
            STATE = STATE_START;
        }
        else if (receivedStr == "STOP") {
            Serial.println("STOP command received! Pausing data transmission.");
            STATE = STATE_STOP;
        }
        else if (receivedStr == "SYNC") {
            Serial.println("Processing SYNC command...");
            STATE = STATE_SYNC;
        }
        else if (receivedStr == "2") {
            Serial.println("Turn 2");
            TURN = 2;
        }
        else if (receivedStr == "1") {
            Serial.println("Turn 1");
            TURN = 1;
        }
        else {
            Serial.println("Unknown command.");
        }
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
          BiozFifoBurstValues[i] = (rxByte[0] << 16) | (rxByte[1] << 8) | (rxByte[2]) & 0xFFFFFF;;

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
    //setSwitch_BIOZ();
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
    // writeRegister(REG_FIFO_CONFIG_1,0x80); // 128 Points triggers INT
    //delayMicroseconds(5);
    readRegister(REG_BIOZ_CONFIG_1);
    //delayMicroseconds(5);
    writeRegister(REG_BIOZ_CONFIG_1,0xF4);
    //delayMicroseconds(5);
    readRegister(REG_FIFO_CONFIG_2);
    //delayMicroseconds(5);
    writeRegister(REG_FIFO_CONFIG_2,0x1A);
    //delayMicroseconds(5);
    readRegister(REG_SYS_CONFIG_1);
    //delayMicroseconds(5);
    writeRegister(REG_SYS_CONFIG_1,0x00);
    //delayMicroseconds(5);
    writeRegister(REG_BIOZ_CONFIG_1,0xF5);
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

// INT Pin Interrupt Function
// void checkVoltage(void) {
//   float voltage = analogRead(INT_PIN) * (3.3 / 1023.0); // Convert analog reading to voltage
//   if (voltage <= 0.8) {
//     Serial.print("Calculated Voltage:");
//     Serial.println(voltage);
//     interruptFlag = true; // Set the flag if voltage is above threshold
//   }
// }
// Interrupt Service Routine (ISR)
void handleInterrupt(void) {
    interruptFlag = true; // Set the flag for loop()
}

void endCollection(void)
{
      // End Collection
    readRegister(REG_BIOZ_CONFIG_1);
    //delayMicroseconds(5);
    writeRegister(REG_BIOZ_CONFIG_1,0xF4);
    //delayMicroseconds(5);
    readRegister(REG_FIFO_CONFIG_2);
    //delayMicroseconds(5);
    writeRegister(REG_FIFO_CONFIG_2,0x1A);
    //delayMicroseconds(5);
    readRegister(REG_SYS_CONFIG_1);
    //delayMicroseconds(5);
    writeRegister(REG_SYS_CONFIG_1,0x02);
    //delayMicroseconds(5);
    writeRegister(REG_BIOZ_CONFIG_1,0xF5);
    //delayMicroseconds(5);
    readRegister(REG_FIFO_CONFIG_2);
    //delayMicroseconds(5);
    writeRegister(REG_FIFO_CONFIG_2,0x1A);
    //delayMicroseconds(5);
}

void printBuffer(uint8_t *buffer, size_t length) {
    Serial.print("Buffer: ");
    for (size_t i = 0; i < length; i++) {
        Serial.printf("%02X ", buffer[i]);  // Print in hex format
    }
    Serial.println();
}


void printDataBluetooth(uint8_t* buf)
{
  if(FIFOcount >= 255){
    FIFOcount = 255;
  }
  // else if(FIFOcount == 0){ // Dummy fix to keep it contiously running
  //   FIFOcount = 255;
  // }

  strncpy((char *)buf, startMessage.c_str(), startMessage.length());
  // bleuart.write(0, buf, sizeof(buf));
  printAll(buf, startMessage.length());
  sprintf((char *)buf, "%d", FIFOcount);
  bleuart.write(0, buf, 3);

  memcpy(buf, (const void*)BiozFifoBurstValues, 6 * FIFOcount);  // Copy data

  // Debug: Print buffer to Serial
  // printBuffer(buf, 6 * FIFOcount);

  // Send data over BLE
  bleuart.write(0, buf, 240);

    
    //printAll((const uint8_t*)startBurstMessage.c_str(), startBurstMessage.length());
    // bleuart.write(0, (const char*)startBurst.hour(), startBurst.hour().length());
    // bleuart.write(0, (const char*)startBurst.minute(), startBurst.minute().length());
    // bleuart.write(0, (const char*)startBurst.second(), startBurst.second().length());
      // for (int j = 0; j < FIFOcount; j++)
      // {
      //   uint32_t tempValue = BiozFifoBurstValues[j];
      //   tempValue &= 0xFFFFFF;
      //   // Convert the uint32_t value to a char array (hex string)
      //   sprintf((char *)buf, "%06X", tempValue);
      //   // Serial.print("0x");
      //   // Serial.println((char *)buf);
      //   bleuart.write(0, buf, 6);      

      // }
    // strncpy((char *)buf, setupComplete.c_str(), setupComplete.length());
    // printAll(buf, sizeof(buf));
    strncpy((char *)buf, endMessage.c_str(), endMessage.length());
    printAll(buf, endMessage.length());
    //printAll((const uint8_t*)endBurstMessage.c_str(), endBurstMessage.length());
    // bleuart.write(0, (const char*)endBurst.hour(), endBurst.hour().length());
    // bleuart.write(0, (const char*)endBurst.minute(), endBurst.minute().length());
    // bleuart.write(0, (const char*)endBurst.second(), endBurst.second().length());

  //}
  return;
}

// Send data_count with header 0x01
void sendDataCount() {
    uint8_t buffer[5];  // 1-byte header + 4-byte count
    buffer[0] = 0x01;
    // memcpy(&buffer[1], &FIFOcount, 4);  // Copy data_count as uint32_t
    uint16_t temp_count = FIFOcount; // Copy volatile value to non-volatile
    memcpy(&buffer[1], &temp_count, 2);  // Use 2 bytes for uint16_t
    bleuart.write(buffer, 5);
    // delay(10);  // Ensure reliable transmission
}

// Send sensor data with header 0x02
void sendData() {
    uint8_t buffer[19];  // 1-byte header + 6 × 24-bit values
    buffer[0] = 0x02;

    for (int i = 0; i < FIFOcount; i += 6) {
        int chunk_size = min(6, FIFOcount - i);  // Handle remaining data

        for (int j = 0; j < chunk_size; j++) {
            uint32_t value = BiozFifoBurstValues[i + j] & 0xFFFFFF;
            buffer[1 + j * 3] = value & 0xFF;
            buffer[2 + j * 3] = (value >> 8) & 0xFF;
            buffer[3 + j * 3] = (value >> 16) & 0xFF;
        }

        bleuart.write(buffer, 1 + chunk_size * 3);
        // delay(10);
    }
}

// Send text messages with header 0x03
void sendTextMessage(const char* msg) {
    uint8_t buffer[21];  // 1-byte header + 20 bytes of text max
    buffer[0] = 0x03;

    int msg_len = strlen(msg);
    for (int i = 0; i < msg_len; i += 20) {
        int chunk_size = min(20, msg_len - i);
        memcpy(&buffer[1], msg + i, chunk_size);

        bleuart.write(buffer, 1 + chunk_size);
        delay(10);
    }
}


// ***************************************************
// On AFE Interrupt Pin Signal Goes LOW (from Example Source Code)
// ***************************************************

void onAfeInt(void)	// call this on AFE interrupt
{
  // READ and CLEAR all status registers
	// readRegister(REG_STATUS1, 2);
  readRegister(REG_STATUS1, 2);
  //delayMicroseconds(5);
	//if (!(rxByte[0] & 0b1000)){	// check A_FULL bit readRegister(REG_STATUS1
  if (!(rxByte[0] & 0b1000)){	// check A_FULL bit 
    Serial.println("ERROR: A_FULL bit OFF");
    Serial.print("Expected:");
    Serial.println("0x8X or higher.");
    Serial.print("Received: 0x");
    Serial.println(rxByte[0], HEX);
    dataError = 1;
		return;
  }
  // delayMicroseconds(5);
  // Check count value available for FIFO Burst Read
  // readRegister(REG_FIFO_CTR_1,2);
  // Serial.println(rxByte[0]);
  // Serial.println(rxByte[1]);
  // count = ((rxByte[0]&0b1000)<<1)|rxByte[1];	// FIFO_DATA_COUNT should be equal to NUM_SAMPLES_PER_INT
  readRegister(REG_FIFO_CTR_1, 2);
  FIFOcount = rxByte[1];
  Serial.println(FIFOcount);
	// read FIFO_DATA
  // if(FIFOcount == 0){
  //   readRegister(REG_FIFO_DATA,255); // Dummy fix
  // }
  // else{
	  readRegister(REG_FIFO_DATA,FIFOcount);
  // }
  return;
}
