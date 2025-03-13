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

// Custom libraries
//#include "ALRI_DUE_PINS.h"          // DUE pins for ALRI PCB
#include "MAX_REGISTER_ADDRESSES.h" // Register addresses of MAX
#include "MAX_BIOZ_VALUES.h"        // Macros with defined bit patterns for BIOZ operation of MAX30001
#include "MAX_EGG_VALUES.h"         // Macros with defined bit patterns for EGG operation of MAX



// Modifiable message string
String setupComplete = "Setup Complete!";  // Change this string to any message you want
String startBurstMessage = "Burst Meas Start";  
String endBurstMessage = "Burst Meas End";  

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

RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};



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
#define INT_PIN 1
#define TRIG_PIN 2
#define extLED 0

#define AFE_FIFO_SIZE			256

#define NUM_BYTES_PER_SAMPLE	3
#define NUM_SAMPLES_PER_INT		129	/* number of samples in FIFO that generates a FIFO_A_FULL interrupt */

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
volatile int32_t BiozFifoBurstValues[6];    // Array to store valid BIOZ reads during measurement
volatile int32_t savedBiozResistance[9][6]; // Array for storing BIOZ read data after measurement
                                            // First index tracks the frequency
                                            // Second index tracks the repeated measurement number
volatile int32_t savedBiozReactance[9][6];

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

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Seeed Studio");
  bledis.setModel("Xiao nRF52840");
  //bledis.setName("Xiao nRF52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

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


// RTC Setup
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, RTC needs a time reset");
  }

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
  Serial.write(buf, count);

  // Send to all connected centrals
  for (uint8_t conn_hdl=0; conn_hdl < MAX_PRPH_CONNECTION; conn_hdl++)
  {
    bleuart.write(conn_hdl, buf, count);
  }
}


// ************************************************************
// MAIN
// ************************************************************

void loop() {

  uint8_t buf[64];
  int count;

  // int intState = digitalRead(INT_PIN);
  //   //Serial.println("Working Here 1");
  // //delay(1000);
  // configureMAX_REGS(); // Configure REGS
  // Serial.println("Regs Configured");
  //delay(100);
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

      // char hexVal[3]; // Buffer to hold the hexadecimal string
        /////////////////////////////////////////////////////////////////////////////////////////////////
      // Serial.println("What Registers to check?");
      // delay(500);
      // Serial.println("Checking FIFO_A_FULL Reg Val");
      // uint8ToHex(REG_FIFO_CONFIG_1, hexVal);
      // Serial.print("0x");
      // Serial.println(hexVal);
      // uint8ToHex(readRegister(REG_FIFO_CONFIG_1), hexVal);
      // Serial.print("0x");
      // Serial.println(hexVal);

  //Serial.println("Working Here *2*");
  while (bleuart.available()) {
    // if the remote device wrote to the characteristic,
    // use the value to control the LED:

    Serial.println("BLEUART Available.");

    // count = bleuart.read(buf, sizeof(buf));

    // if(strncmp((char*)buf, "START", 5) == 0){
    //   // Delay to wait for enough input
    //   delay(2);
    // }

    //int intState = digitalRead(INT_PIN);
    configureMAX_REGS(); // Configure REGS
    Serial.println("Regs Configured");

    digitalWrite(ledPin, HIGH);  // LED OFF
    //digitalWrite(OUTPUT_PIN, LOW);  // LED OFF

    digitalWrite(ledPin, LOW);  // LED ON
      
    // Serial.println("Enter START to sweep BIOZ");

    // while (Serial.available() == 0) // Wait for UART to settle
    //     ;

    //Serial.println("Is Working?");
    // delay(50);
    // String userInput = Serial.readString();
    // userInput.trim();  
    // Serial.println("I received: ");
    // Serial.println(userInput);

    // String userInput = Serial.readString();
    // userInput.trim();
    // while (userInput != "START");
    //     userInput = Serial.readString(); // Idle until START received
    // digitalWrite(LED_BUILTIN, HIGH);  
    Serial.println("Sweeping BIOZ");

    char hexVal[3]; // Buffer to hold the hexadecimal string
    measurementSetup();

    iterationCnt = 0;
    dataError = 0;
    //while (dataError != 1)
    //while (iterationCnt != 1)
    while (1)
    {
      getMeasurement();
      // printData();
      printDataBluetooth(buf);
      // Serial.println("Press ENTER to continue, STOP to end.");
      // while (Serial.available() == 0) // Wait for UART to settle
      // ;
      // userInput = Serial.readString();
      // userInput.trim();
      // delay(50);
      iterationCnt += 1;
      //dataError = 1;
    }

  }

  endCollection();
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);  

}

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
  
  // Keep advertising if not reaching max
  if (connection_count < MAX_PRPH_CONNECTION)
  {
    Serial.println("Keep advertising");
    Bluefruit.Advertising.start(0);
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

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

  connection_count--;
}



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
// BIOZ Functions
// ************************************************************

// Read specified register and return 32bit value
//  Inputs: address of register to read (uint8)
//  Outputs: uint32 of 3 data words (8-bits each)
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
    uint8_t read = SPI.transfer(0xFF); // DONT CARE Transmitted Bits
    digitalWrite(CS, HIGH);
    SPI.endTransaction();
    // SPI Communication Ends

    return read;
}


void readStatusRegisters()
{
    // SPI Communication Starts
    SPI.beginTransaction(SPI_DEFINE);
    digitalWrite(CS, LOW);
    // NOTE: Arduino Library Buffer Transfer doesn't work for this set-up
    // It transmits bytes in reverse order, meaning command word is sent last
    // if above bit shifts are applied
    SPI.transfer(0x00);
    SPI.transfer(READ_BYTE);
    // Temporary 8bit holder for read values
    StatusReg1 = SPI.transfer(0xFF); // DONT CARE Transmitted Bits
    StatusReg2 = SPI.transfer(0xFF); // DONT CARE Transmitted Bits
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

// Read BioZ BURST FIFO
//  Modifies: BiozFifoBurstValues global
void readBiozFifoBurst()
{
    SPI.beginTransaction(SPI_DEFINE);
    digitalWrite(CS, LOW);
    SPI.transfer(REG_FIFO_DATA);
    SPI.transfer(READ_BYTE);
    for (int i = 0; i < 6; i++)
    {
        // MAX returns 24-bit word with 20 data bits and 4 tagging bits
        // Capture each 8-bit chunk and store into global array

        //Serial.println(i+1);
        uint8_t read1 = SPI.transfer(1 + i * 3);
        //Serial.println(read1, HEX);
        uint8_t read2 = SPI.transfer(2 + i * 3);
        //Serial.println(read2, HEX);
        uint8_t read3 = SPI.transfer(3 + i * 3);
        //Serial.println(read3, HEX);
        BiozFifoBurstValues[i] = (read1 << 16) | (read2 << 8) | (read3);
    }
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
    readStatusRegisters();

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
    readStatusRegisters();
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
    writeRegister(REG_FIFO_CONFIG_1,0xFA);
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
    readStatusRegisters();
    //delayMicroseconds(5);

    // Setup Finished
}

void getMeasurement()
{
    //delay(200);
    intPinState = analogRead(INT_PIN);
    while (intPinState >= 0.8)
    {
      intPinState = analogRead(INT_PIN);

    }
    Serial.println("INT Pin Active");
    intPinState = 1.7;

      
    // Begin Data Collection
    startBurst = rtc.now(); 
    onAfeInt();
    endBurst = rtc.now(); 
    delay(90);
    // 
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

void printData(void)
{
  //char val[9]; // Buffer to hold the hexadecimal string
  for (int j = 0; j < 6; j++)
    {
      Serial.println((iterationCnt*6)+(j+1));
      //Serial.println(BiozFifoBurstValues[j]);
      //Serial.println(BiozFifoBurstValues[j], HEX);
      uint32_t tempValue = BiozFifoBurstValues[j];
      tempValue &= 0xFFFFFF;
      //uint32ToHex(tempValue, val);
      Serial.print("0x");
      Serial.println(tempValue, HEX);
      //tempValue &= 0xFFFFF;
      //uint32ToHex(tempValue, val);

      // 2s Complement Conversion

      double ZBIOZ = tempValue * 1/ (524288 * 10.0 * (2 / PI) * 0.0000320362);
      Serial.print("0d");
      Serial.println(ZBIOZ);
    }
  return;
}


void printDataBluetooth(uint8_t* buf)
{
  //uint8_t buf[64];
  int count;

  //if ( bleuart.available() ){

    //*buf = startBurstMessage.c_str();
    // printAll((const uint8_t*)startBurstMessage.c_str(), startBurstMessage.length());
    // bleuart.write(0, (const char*)startBurst.hour(), startBurst.hour().length());
    // bleuart.write(0, (const char*)startBurst.minute(), startBurst.minute().length());
    // bleuart.write(0, (const char*)startBurst.second(), startBurst.second().length());
      for (int j = 0; j < 6; j++)
      {
        uint32_t tempValue = BiozFifoBurstValues[j];
        tempValue &= 0xFFFFFF;
        Serial.print("Hex: 0x");
        Serial.println(tempValue, HEX);

        uint8_t buf[64];
        char charArr[6]; // Enough space for 8 characters + null terminator

        // Convert the uint32_t value to a char array (hex string)
        //snprintf(charArr, sizeof(charArr), "%08X", tempValue);
        sprintf(charArr, "%06X", tempValue);
        Serial.print("Ascii: 0x");
        Serial.println(charArr);

        strncpy((char *)buf, charArr, sizeof(buf));
        buf[sizeof(buf) - 1] = '\0';

        bleuart.write(0, buf, sizeof(buf)-1);

        //*buf = '1FBED4';
        // unsigned maskByte1 = ((1 << 8) - 1) << 16;
        // unsigned maskByte2 = ((1 << 8) - 1) << 8;
        // unsigned maskByte3 = (1 << 8) - 1;
        // uint8_t tempValByte1 = tempValue & maskByte1;
        // uint8_t tempValByte2 = tempValue & maskByte2;
        // uint8_t tempValByte3 = tempValue & maskByte3;

        // for(uint8_t charArrIndex = 0; charArrIndex < 6; charArrIndex++){
        //   *buf = charArr[charArrIndex];
        //   bleuart.write(0, buf, sizeof(buf)-1);
        //   //Serial.println(charArr[charArrIndex]);
        // }

        // bleuart.write(0, (uint8_t*)charArr, 1);
        //bleuart.write(0, buf, 6);
        // *buf = '1';
        // //printAll(buf,sizeof(buf)-1);
        // bleuart.write(0, buf, sizeof(buf)-1);
        // *buf = 'F';
        // //printAll(buf,sizeof(buf)-1);
        // bleuart.write(0, buf, sizeof(buf)-1);
        // *buf = 'B';
        // //printAll(buf,sizeof(buf)-1);
        // bleuart.write(0, buf, sizeof(buf)-1);
        // *buf = 'E';
        // //printAll(buf,sizeof(buf)-1);
        // bleuart.write(0, buf, sizeof(buf)-1);
        // *buf = 'D';
        // //printAll(buf,sizeof(buf)-1);
        // bleuart.write(0, buf, sizeof(buf)-1);
        // *buf = '4';
        // //printAll(buf,sizeof(buf)-1);
        // bleuart.write(0, buf, sizeof(buf)-1);
        

      }
    // printAll((const uint8_t*)endBurstMessage.c_str(), endBurstMessage.length());
    // bleuart.write(0, (const char*)endBurst.hour(), endBurst.hour().length());
    // bleuart.write(0, (const char*)endBurst.minute(), endBurst.minute().length());
    // bleuart.write(0, (const char*)endBurst.second(), endBurst.second().length());

  //}
  return;
}

// ***************************************************
// On AFE Interrupt Pin Signal Goes LOW (from Example Source Code)
// ***************************************************

void onAfeInt(void)	// call this on AFE interrupt
{
  // read and clear all status registers
	readStatusRegisters();
  //delayMicroseconds(5);
	if (!(StatusReg1 & 0x80)){	// check A_FULL bit
    Serial.println("ERROR: A_FULL bit OFF");
    Serial.print("Expected:");
    Serial.println("0x8X or higher.");
    Serial.print("Received: 0x");
    Serial.println(StatusReg1, HEX);
    dataError = 1;
		return;
  }
	//regCheck1 = readRegister(REG_FIFO_CTR_1);	
  //unint8_t regCheck2 = readRegister(REG_FIFO_CTR_2);  // read FIFO_DATA_COUNT
  uint8_t cntCheck = readRegister(REG_FIFO_CTR_2);
  //delayMicroseconds(5);
  if((!(cntCheck & 0x06))&(!(cntCheck & 0x08))){
    Serial.println("ERROR: FIFO Count Check FAILED");
    Serial.print("Expected:");
    Serial.println("0x06");
    Serial.print("Received: 0x");
    Serial.println(cntCheck, HEX);
    dataError = 1;
    writeRegister(REG_FIFO_CTR_2,0x06); // Set FIFO Cnt
    return;
  }
	//uint32_t count = ((regCheck1 & 0x80)<<1)|regCheck2;	// FIFO_DATA_COUNT should be equal to NUM_SAMPLES_PER_INT
	readBiozFifoBurst();	// read FIFO_DATA
}


