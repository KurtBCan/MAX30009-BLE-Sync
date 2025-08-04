// bioz_arduino.ino
// Capture BIOZ data using MAX30001 and Arduino

// How to use
// 1) Select Serial IO using Serial and SerialSpeed
// 2) Deploy to DUE and establish Serial connection (PC or Raspberry Pi)
// 3) Issue "START" command to begin BIOZ sweep
// 4) Wait about 5 seconds for sweep, then table of data will be printed
//  
//  Data contains both raw values (hex) and computed values (ohm)


// External libraries
#include <Arduino.h>
#include <SPI.h> // SPI communication to MAX

// Xiao Added Libraries
//#include <bluefruit.h>
#include <Wire.h>
//#include <Adafruit_TinyUSB.h>

// Custom libraries
//#include "ALRI_DUE_PINS.h"          // DUE pins for ALRI PCB
// #include "bioz_arduino_Rev2.h"
#include "MAX_REGISTER_ADDRESSES.h" // Register addresses of MAX
#include "MAX_BIOZ_VALUES.h"        // Macros with defined bit patterns for BIOZ operation of MAX30001
#include "MAX_EGG_VALUES.h"         // Macros with defined bit patterns for EGG operation of MAX


// define Xiao CS
#define CS 7



// Define MAX30009/Xiao INT Pin Connection
#define INT_PIN 2 // Use an interrupt-capable pin (D2)


#define AFE_FIFO_SIZE			256

#define NUM_BYTES_PER_SAMPLE	3
#define NUM_SAMPLES_PER_INT		129	/* number of samples in FIFO that generates a FIFO_A_FULL interrupt */

// Global Variable
//uint8_t gReadBuf[NUM_SAMPLES_PER_INT*NUM_BYTES_PER_SAMPLE];	// array to store register reads
uint8_t errCnt;
uint8_t StatusReg1, StatusReg2;
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
volatile int32_t BiozFifoBurstValues[256];    // Array to store valid BIOZ reads during measurement
// volatile int32_t savedBiozResistance[9][6]; // Array for storing BIOZ read data after measurement
//                                             // First index tracks the frequency
//                                             // Second index tracks the repeated measurement number
// volatile int32_t savedBiozReactance[9][6];

uint32_t count;

volatile uint8_t rxByte[3];	// array to store register reads

volatile bool interruptFlag = false; // Flag to indicate interrupt


void setup()
{
  digitalWrite(LED_BUILTIN, LOW);  
  //Serial.begin(9600);
  delay(100);
  //Serial.println("Is Working?"); // Send serial message
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);  // Ensure SS is high
  SPI.begin();

  //SPI.setBitOrder(MSBFIRST);  // Set bit order
  Serial.begin(115200);

  // pinMode(INT_PIN, INPUT);
  pinMode(INT_PIN, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(INT_PIN), checkVoltage, CHANGE); // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING); // Trigger on falling edge


}

// ************************************************************
// MAIN
// ************************************************************

void loop()
{
    delay(2000);
    endCollection();
    Serial.println("Ending Collection.");
    
    delay(2000);

    // endCollection();

    // Configure MAX
    //configureMAX_BIOZ(); // Configure BIOZ settings
    configureMAX_REGS(); // Configure REGS

    Serial.println("Setup complete");

    while (true)
    {
        
        Serial.println("Enter START to sweep BIOZ");

        while (Serial.available() == 0) // Wait for UART to settle
            ;

        String userInput = Serial.readString();
        userInput.trim();
        Serial.println("I received: ");
        Serial.println(userInput);
        while (userInput != "START");
            userInput = Serial.readString(); // Idle until START received
        digitalWrite(LED_BUILTIN, HIGH);  
        Serial.println("Sweeping BIOZ");

        char hexVal[3]; // Buffer to hold the hexadecimal string
        /////////////////////////////////////////////////////////////////////////////////////////////////
        Serial.println("What Registers to check?");
        delay(500);
        Serial.println("Checking FIFO_A_FULL Reg Val");
        uint8ToHex(REG_FIFO_CONFIG_1, hexVal);
        Serial.print("0x");
        Serial.println(hexVal);
        uint8ToHex(readRegister(REG_FIFO_CONFIG_1), hexVal);
        Serial.print("0x");
        Serial.println(hexVal);
        delay(1000);
        Serial.println("Checking PLL_CONFIG_1 Reg Val");
        uint8ToHex(REG_PLL_CONFIG_1, hexVal);
        Serial.print("0x");
        Serial.println(hexVal);
        uint8ToHex(readRegister(REG_PLL_CONFIG_1), hexVal);
        Serial.print("0x");
        Serial.println(hexVal);
        // delay(1000);
        ////////////////////////////////////////////////////////////////////////////////////////

        measurementSetup();

        iterationCnt = 0;
        dataError = 0;
        // swhile (dataError != 1)
        while (true)
        {
          if (interruptFlag) {
            interruptFlag = false; // Reset the flag
            Serial.println("INT Pin Active");
            // Begin Data Collection
            onAfeInt();
            printData(count);
            iterationCnt += 1;
            // interruptFlag = false; // Reset the flag
          }
          
          //getMeasurement();
          //printData(count);


          // // Check if User wants to STOP MAX30009 Reads
          // userInput = Serial.readStringUntil('\n'); // Read the input until newline
          // userInput.trim(); // Remove leading and trailing whitespace
          // if (userInput.equals("STOP")) {
          //   Serial.println("Loop ended.");
          //   dataError = 1; // Stop the loop
          // } else if(!(userInput.equals(""))){
          //   Serial.print("You entered: ");
          //   Serial.println(userInput);
          // }
        }
        endCollection();
        Serial.println("Ending Collection.");
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);  

    }
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
// MAX 30009 Functions
// ************************************************************

// void getMeasurement()
// {
//     //delay(200);
//     volatile int intPinState = analogRead(INT_PIN);
//     while (intPinState >= 0.8)
//     {
//       intPinState = analogRead(INT_PIN);

//     }
//     Serial.println("INT Pin Active");
//     intPinState = 1.7;

      
//     // Begin Data Collection
//     onAfeInt();
//     //delay(90);
//     return;
//     // 
// }

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
          rxByte[0] = SPI.transfer(1);
          rxByte[1] = SPI.transfer(2);
      // }
    }
    else{
      // Read BioZ BURST FIFO
      //  Modifies: BiozFifoBurstValues global
      for (int i = 0; i < byteCount; i++)
      {
          // MAX returns 24-bit word with 20 data bits and 4 tagging bits
          // Capture each 8-bit chunk and store into global array
          rxByte[0] = SPI.transfer(0xFF);
          rxByte[1] = SPI.transfer(0xFF);
          rxByte[2] = SPI.transfer(0xFF);
          // Save into global FIFOBurstValues array to reuse rxByte array
          BiozFifoBurstValues[i] = (rxByte[0] << 16) | (rxByte[1] << 8) | (rxByte[2]);
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
//    - uint8 value to write into register
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

void printData(volatile uint32_t &count)
{
  if(count >= 255) 
    count = 255;
  for (int j = 0; j < count; j++)
    {
      Serial.println((iterationCnt*count)+(j+1));
      uint32_t tempValue = BiozFifoBurstValues[j];
      tempValue &= 0xFFFFFF;
      Serial.print("0x");
      Serial.println(tempValue, HEX);

      // 2s Complement Conversion
      // double ZBIOZ = tempValue * 1/ (524288 * 10.0 * (2 / PI) * 0.0000320362);
      // Serial.print("0d");
      // Serial.println(ZBIOZ);
    }
  return;
}

// ****************************************************************//
// On AFE Interrupt Pin Signal Goes LOW (from Example Source Code) //
// ****************************************************************//

void onAfeInt(void)	// call this on AFE interrupt
{
  // READ and CLEAR all status registers
	// readRegister(REG_STATUS1, 2);
  uint8_t regVal = readRegister(REG_STATUS1);
  //delayMicroseconds(5);
	//if (!(rxByte[0] & 0b1000)){	// check A_FULL bit readRegister(REG_STATUS1
  if (!(regVal & 0x80)){	// check A_FULL bit 
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
  count = readRegister(REG_FIFO_CTR_2);
  Serial.println(count);
  //delayMicroseconds(5);

	// read FIFO_DATA
	readRegister(REG_FIFO_DATA,count);	
  return;
}
