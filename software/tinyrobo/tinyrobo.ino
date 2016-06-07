#include <ESP8266WiFi.h>
#include <Wire.h>

char ssid[] = "full_of_internets";     //  your network SSID (name)
char pass[] = "sofullsuchinternets";  // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

WiFiServer server(4321); //Totally arbitrary port number

/*
    Test the I2C motor drivers on the TinyRobo board

    According to my schematic, the motor drivers are at addresses
    0xD0h (read) 0xD1h (write) and 0xCCh (read) and 0xCDh (write)
    Arduino uses 7-bit addresses, so these get shifted one bit right,
    and read() write() are used instead of different addresses.
*/
#define addr1 (0xCC >> 1)
#define addr2 (0xD0 >> 1)
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

typedef enum {
  CONN_WAIT,
  CONNECTED,
  CLI_READ,
  QUERY_RESP,
  MOTOR_READ,
  MOTOR_DRV,
} machine_state;

//Stores the motor commands [speed1, direction1, speed2, direction2]
//received from the client
byte motor_cmd[4] = {0, 0, 0, 0};
int cmd_index = 0;

//Stores fault codes (one per motor) to send to client
byte fault[2] = {0x00, 0x00};

machine_state state = CONN_WAIT; //Waiting for connection from client

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
  Wire.begin(12, 13); //Correct for v2 boards, arguments are (SDA, SCL).
  Serial.begin(9600);

  // attempt to connect to Wifi network:
  WiFi.mode(WIFI_STA);
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 5 seconds for connection:
    delay(5000);
  }

  //Now connected to WiFi
  //Print the IP address
  Serial.print("You're connected to the network");
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  //Start the server
  server.begin();


}

/* Protocol from the client is as follows:
   Q - version query, send back some kind of ident string
   M[m1 speed][m1 direction][m2 speed][m2 direction] - motor control command
*/
void loop() {
  WiFiClient client = server.available();

  //client goes out of scope when it gets redeclared there
  if (client) {
    state = CLI_READ; //start reading from the client
    Serial.println("Got a client");    
    while (client.connected()) {
      switch (state)
      {
        case CLI_READ:
          if (client.available()) {
            //Read from client
            char c = client.read();
            Serial.print("Read: ");
            Serial.print(c);
            Serial.print(" (");
            Serial.print(c, HEX);
            Serial.println(")");
            if ( c == 'Q') {
              state = QUERY_RESP;
            } else if ( c == 'M') {
              cmd_index = 0; //start of new motor command
              state = MOTOR_READ;
            } else {
              //This is an error, how to deal with it?
            }
          }
          break;
        case QUERY_RESP:
          Serial.println("TinyRobo");
          client.println("TinyRobo"); //TODO include the fault byte
          state = CLI_READ;
          break;
        case MOTOR_READ:
          //Read a byte for motors
          if (client.available()) {
            //Read command from client and store in buffer
            motor_cmd[cmd_index] = client.read();
            Serial.print("Motor read: ");
            Serial.print(motor_cmd[cmd_index]);
            Serial.print(" (");
            Serial.print(motor_cmd[cmd_index], HEX);
            Serial.println(")");
            cmd_index++;
            //If we have received 4 bytes (2 speed, 2 direction), then change the motors
            if (cmd_index == 4) {
              cmd_index = 0;
              state = MOTOR_DRV;
            }
          }
          break;
        case MOTOR_DRV:
          //Set the motor state from what the client sent
          setMotor(addr1, motor_cmd[0], motor_cmd[1]);
          setMotor(addr2, motor_cmd[2], motor_cmd[3]);
          //Get the fault bits
          fault[0] = getFault(addr1);
          fault[1] = getFault(addr2);

          //Debug print everything
          Serial.print("Set motor state");
          for (int ii = 0; ii < 4; ii++) {
            Serial.print(" ");
            Serial.print(motor_cmd[ii], HEX);
          }
          for (int ii = 0; ii < 2; ii++) {
            Serial.print(" ");
            Serial.print(fault[ii], HEX);
          }
          Serial.println(' ');
          state = CLI_READ;
          break;
      }
    }
  }
}
