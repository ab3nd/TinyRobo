#include <Wire.h>

/*
 *  Test the I2C motor drivers on the TinyRobo board
 *  
 *  According to my schematic, the motor drivers are at addresses
 *  0xD0h (read) 0xD1h (write) and 0xCCh (read) and 0xCDh (write)
 *  Arduino uses 7-bit addresses, so these get shifted one bit right, 
 *  and read() write() are used instead of different addresses.
 */
#define addr1 (0xCC >> 1)
#define addr2 (0xD0 >> 1)
byte faultVals = 0x00;

/*  According to the DRV8830 data sheet, there are two registers:
 *  0x00 - Control
 *  bit function 
 *  7-2 Output voltage (VSET), which roughly correlates with speed in DC motors
 *  1   IN2 One of the H-bridge control lines
 *  0   IN1 The other H-bridge control line
 *  
 *  The valid range for VSET is 0x06h (0.48V) to 0x3Fh (5.06V) in 0.08V increments.
 *  My batteries are 3.7V, so 0x30h (3.86V) ahd higher is 100% power. 
 *  0x00h-0x05h are "reserved".
 *  
 *  H-bridge truth table
 *  IN1 IN2 OUT1  OUT2  Function
 *  0   0   Z     Z     Coast (motor leads high impedence)
 *  0   1   L     H     Reverse
 *  1   0   H     L     Forward (relative to "reverse", anyway)
 *  1   1   H     H     Brake
 *  
 *  0x01 - Fault
 *  bit function
 *  7   CLEAR, clears fault status when written to 1
 *  6-5 Unused
 *  4   Current limit (ILIMIT) if set, indicates fault was extended overcurrent
 *  3   Overtemperature (OTS) if set, chip is too hot 
 *  2   undervoltage (UVLO) if set, supply voltage is too low
 *  1   Overcurrent (OCP) if set, overcurrent event
 *  0   FAULT, if set, a fault condition exists
 */

void setup() {
  Wire.begin(14, 12);
  Serial.begin(9600);
}

void loop(){

  Wire.beginTransmission(addr1); // Target device
  Wire.write(byte(0x01));        // sets register pointer to fault register
  Wire.endTransmission();        // stop transmitting

  Wire.requestFrom(addr1, 1);
  if(Wire.available())
  {
    faultVals = Wire.read();
    Serial.println(faultVals, HEX);
  }

  Wire.beginTransmission(addr1);
  Wire.write(byte(0x00)); //Control register
  Wire.write(byte(0x1F << 2) | byte(0x01)); //Moderate speed
  Wire.endTransmission();

  delay(700);
  
  Wire.beginTransmission(addr1);
  Wire.write(byte(0x00)); //Control register
  Wire.write(byte(0x00)); //Stop
  Wire.endTransmission();

  delay(700);

  Wire.beginTransmission(addr1);
  Wire.write(byte(0x00)); //Control register
  Wire.write(byte(0x1F << 2) | byte(0x02)); //Moderate speed, other way
  Wire.endTransmission();

  delay(700);

  Wire.beginTransmission(addr1);
  Wire.write(byte(0x00)); //Control register
  Wire.write(byte(0x00)); //Stop
  Wire.endTransmission();

  delay(700);
  
}


