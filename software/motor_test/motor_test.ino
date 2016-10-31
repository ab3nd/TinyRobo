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
  pinMode(2, OUTPUT); //Diganostic LED

  Serial.begin(9600);
}

void loop() {
  /* Motor controller sets voltage across motor, range is
   * 0x06h (0.48V) to 0x3Fh (5.06V)
   * 0x30h is 3.86v, max speed with 3.7V batteries.
   * last two bits of the motor speed control the H-bridge outputs,
   * if they're the same, the motor won't turn. 
   */
  //Test motor 1
  for(int ii = 0; ii < 38; ii++)
  {
    digitalWrite(2, LOW);
    setMotor(addr1, 0x06 + ii, 0x02);
    delay(1000);
    digitalWrite(2, HIGH);
    faultVals = getFault(addr1);

    //Debug print everything
    Serial.print("Motor speed: ");
    Serial.println(0x06+ii, HEX);
    Serial.print("Fault: ");
    Serial.print(faultVals, HEX);
    Serial.println(' ');
    delay(10);
  }
  setMotor(addr1, 0x00, 0x00);
  digitalWrite(2, LOW);
}
