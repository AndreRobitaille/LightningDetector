/*
      SEN0290 Lightning Sensor
       This sensor can detect lightning and display the distance and intensity of the lightning within 40 km
       It can be set as indoor or outdoor mode.
       The module has three I2C, these addresses are:
       AS3935_ADD1  0x01   A0 = 1  A1 = 0
       AS3935_ADD2  0x02   A0 = 0  A1 = 1
       AS3935_ADD3  0x03   A0 = 1  A1 = 1

Lightning Detector; products designed and built by DFRobot, and screen programming by Etienne Robitaille

PLEASE MAKE VERSION HISTORY HERE:

0.1 - Major Change - Etienne Robitaille: "Program created. Initialize device."

*/

//#include <Lib_I2C.h>
#include <DFRobot_AS3935_I2C.h>
#include <math.h>
#include <DFRobot_LCD.h> // library for the screen
#include <Wire.h> // 99% Sure this is included in <DFROBOT_LCD.h> but just to be safe

uint8_t colorR = 255; //Red of RGB
uint8_t colorG = 0; // Green of RGB
uint8_t colorB = 0; // Blue of RGB
DFRobot_LCD lcd(16,2);  //16 characters and 2 lines

volatile int8_t AS3935IsrTrig = 0;

#define IRQ_PIN              2

// Antenna tuning capcitance (must be integer multiple of 8, 8 - 120 pf)
#define AS3935_CAPACITANCE   96

// Indoor/outdoor mode selection
#define AS3935_INDOORS       0
#define AS3935_OUTDOORS      1
#define AS3935_MODE          AS3935_INDOORS

// Enable/disable disturber detection
#define AS3935_DIST_DIS      0
#define AS3935_DIST_EN       1
#define AS3935_DIST          AS3935_DIST_EN

// I2C address
#define AS3935_I2C_ADDR       AS3935_ADD3

void AS3935_ISR();

DFRobot_AS3935_I2C  lightning0((uint8_t)IRQ_PIN, (uint8_t)AS3935_I2C_ADDR);

void setup() {
  // Setup for serial, speed is 115200
  Serial.begin(115200);
  Serial.println("Sensor input beginning!");

  // Initialize screen
  lcd.init();
  lcd.setRGB(colorR, colorG, colorB);

  // Setup for the the I2C library: (enable pullups, set speed to 400kHz)
  //I2c.begin();
  //I2c.pullup(true);
  //I2c.setSpeed(1);

  delay(2); // This might be part of the I2C code above.

  // if I2C cannot begin
  if (lightning0.defInit() != 0) {
    Serial.println("I2C init fail");
    lcd.setRGB(255, 0, 13); // set color to red
    lcd.setCursor(0,0); // line 1
    lcd.print("   I2C start up  "); // Must be 16 characters
    lcd.setCursor(0,1); // line 2
    lcd.print("    has failed   "); // Must be 16 characters
    while (1) {} // while I2C remains failed, continue displaying message
  }

  // Initialize sensor
  lightning0.manualCal(AS3935_CAPACITANCE, AS3935_MODE, AS3935_DIST);

  //  Connect the IRQ and GND pin to the oscilloscope.
  //  uncomment the following sentences to fine tune the antenna for better performance.
  //  This will dispaly the antenna's resonance frequency/16 on IRQ pin (The resonance frequency will be divided by 16 on this pin)
  //  Tuning AS3935_CAPACITANCE to make the frequency within 500/16 kHz ± 3.5%
  //  lightning0.setLcoFdiv(0);
  //  lightning0.setIRQOutputSource(3);

  // Enable interrupt (connect IRQ pin IRQ_PIN: 2, default)
  // *********************** this is setting the IRQ pin to 0. Is that where it's connected? This should use the IRQ_PIN constant instead.
  attachInterrupt(0, AS3935_ISR, RISING);
}

void loop() {
  // It does nothing until an interrupt is detected on the IRQ pin.
  while (AS3935IsrTrig == 0) {}
  delay(5);

  // Reset interrupt flag
  AS3935IsrTrig = 0;

  // Get interrupt source
  uint8_t intSrc = lightning0.getInterruptSrc();
  
  if (intSrc == 1) { // Lightning strike

  } else if (intSrc == 2) { // Disturber

  } else if (intSrc == 3) { // Noise
   
  }
}

void printToScreen() {

}

//IRQ handler for AS3935 interrupts
void AS3935_ISR() {
  AS3935IsrTrig = 1;
}