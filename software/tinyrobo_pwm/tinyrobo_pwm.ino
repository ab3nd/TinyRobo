#include <ESP8266WiFi.h>
#include <Wire.h>

#define DEBUG
/*
    Test the I2C motor drivers on the TinyRobo board

    According to my schematic, the motor drivers are at addresses
    0xD0h (read) 0xD1h (write) and 0xCCh (read) and 0xCDh (write)
    Arduino uses 7-bit addresses, so these get shifted one bit right,
    and read() write() are used instead of different addresses.
*/
#define addr1 0x63
#define addr2 0x64
byte faultVals = 0x00;

/*  According to the DRV8830 data sheet, there are two registers:
    0x00 - Control
    bit function
    7-2 Output voltage (VSET), which roughly correlates with speed in DC motors
    1   IN2 One of the H-bridge control lines
    0   IN1 The other H-bridge control line

    The valid range for VSET is 0x06h (0.48V) to 0x3Fh (5.06V) in 0.08V increments.
    My batteries are 3.7V, so 0x30h (3.86V) and higher is 100% power.
    0x00h-0x05h are "reserved".

    H-bridge truth table
    IN1 IN2 OUT1  OUT2  Function
    0   0   Z     Z     Coast (motor leads high impedence)
    0   1   L     H     Reverse
    1   0   H     L     Forward (relative to "reverse", anyway)
    1   1   H     H     Brake

    0x01 - Fault
    bit function
    7   CLEAR, clears fault status when written to 1
    6-5 Unused
    4   Current limit (ILIMIT) if set, indicates fault was extended overcurrent
    3   Overtemperature (OTS) if set, chip is too hot
    2   undervoltage (UVLO) if set, supply voltage is too low
    1   Overcurrent (OCP) if set, overcurrent event
    0   FAULT, if set, a fault condition exists
*/

//Stores fault codes (one per motor) to send to client
byte fault[2] = {0x00, 0x00};

void setMotor(byte address, byte vel, byte dir)
{
  //TODO add limit checking to motor speeds
  //TODO add checking to motor directions
  Wire.beginTransmission(address);
  Wire.write(byte(0x00)); //Control register
  Wire.write(byte(vel << 2) | byte(dir));
  Wire.endTransmission();
}

byte getFault(byte addr)
{
  Wire.beginTransmission(addr); // Target device
  Wire.write(byte(0x01));        // sets register pointer to fault register
  Wire.endTransmission();        // stop transmitting

  Wire.requestFrom(int(addr), 1);
  if (Wire.available())
  {
    return Wire.read();
  }

  //This is probably an error
  return 0x00;
}

void setup() {
  Wire.begin(13, 12); //Correct for v2 boards, arguments are (SDA, SCL).
#ifdef DEBUG
  Serial.begin(115200);
#endif

  pinMode(2, OUTPUT);
  for(int ii = 0; ii < 10; ii++)
  {
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
    yield();
  }
}

void loop() {
  uint8_t motStatus[6] = {0,0,0,0,0,0};
  yield();
  //setMotor(addr1, 0x20, motor_cmd[1]);
          
  //Get the fault bits
  fault[0] = getFault(addr1);
  fault[1] = getFault(addr2);

  //count up
  for(byte spd = 0x06; spd < 0x16; spd++)
  {
    setMotor(addr1, spd, 0x01);
    setMotor(addr2, spd, 0x01); 
    yield();
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(1000);
#ifdef DEBUG
    Serial.println(spd, HEX);
#endif      
  }

  //count down
  for(byte spd = 0x16; spd > 0x06; spd--)
  {
    setMotor(addr1, spd, 0x01);
    setMotor(addr2, spd, 0x01); 
    yield();
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(1000);
#ifdef DEBUG
    Serial.println(spd, HEX);
#endif    
  }
}
