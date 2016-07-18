#include <Wire.h>

//0x63 and 0x64 for the white boards (bad I2C address wiring)
#define addr1 0x63
#define addr2 0x64
byte faultVals = 0x00;

void setMotor(byte address, byte vel, byte dir)
{
  //TODO add limit checking to motor speeds
  //TODO add checking to motor directions
  Wire.beginTransmission(address);
  Wire.write(byte(0x00)); //Control register
  Wire.write(byte(vel << 2) | byte(dir));
  Wire.endTransmission();
}

void setup() {
  Wire.begin(13, 12); //Correct for v2 boards, arguments are (SDA, SCL).
  pinMode(2, OUTPUT); //Diganostic LED
}

void loop() {
  /* Motor controller sets voltage across motor, range is
   * 0x06h (0.48V) to 0x3Fh (5.06V)
   * 0x30h is 3.86v, max speed with 3.7V batteries.
   * last two bits of the motor speed control the H-bridge outputs,
   * if they're the same, the motor won't turn. 
   */
  //Test motor 1
  digitalWrite(2, HIGH);
  setMotor(addr1, 0x30, 0x02);
  delay(3000);
  setMotor(addr1, 0x00, 0x00);
  digitalWrite(2, LOW);
  setMotor(addr2, 0x30, 0x02);
  delay(3000);
  setMotor(addr2, 0x00, 0x00); 
}
