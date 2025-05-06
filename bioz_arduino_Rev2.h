#ifndef bioz_arduino_Rev2_H
#define bioz_arduino_Rev2_H

// ************************************************************
// Function Forward Declarations
// ************************************************************

void checkVoltage(); // Forward declaration of the interrupt function

void configureMAX_REGS();
void measurementSetup();
void onAfeInt();
void printData();
void endCollection();


#endif