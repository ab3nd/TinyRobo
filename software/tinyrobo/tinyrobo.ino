#include <ESP8266WiFi.h>
#include <Wire.h>

#define DEBUG

char ssid[] = "TinyRoboBase";     //  your network SSID (name)
//char pass[] = "sofullsuchinternets";  // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

WiFiServer server(4321); //Totally arbitrary port number

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
    My batteries are 3.7V, so 0x30h (3.86V) ahd higher is 100% power.
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

void printWiFiStatus()
{
#ifdef DEBUG
  if(Serial.availableForWrite())
  {
    switch(status){
      case WL_CONNECTED:
        Serial.println("Connected: WL_CONNECTED");
        break;
      case WL_NO_SHIELD:
        Serial.println("No WiFi Shield: WL_NO_SHIELD");
        break;
      case WL_IDLE_STATUS:
        Serial.println("Idle: WL_IDLE_STATUS");
        break;
      case WL_NO_SSID_AVAIL:
        Serial.println("SSID not found: WL_NO_SSID_AVAILABLE");
        break;
      case WL_SCAN_COMPLETED:
        Serial.println("Scan Completed: WL_SCAN_COMPLETED");
        break;
      case WL_CONNECT_FAILED:
        Serial.println("Failed: WL_CONNECT_FAILED");
        break;
      case WL_CONNECTION_LOST:
        Serial.println("Lost Connection: WL_CONNECTION_LOST");
        break;
      case WL_DISCONNECTED:
        Serial.println("Disconnected: WL_DISCONNECTED");
        break;
      default:
        Serial.println("Undefined state");
        break;
    }
  }
#endif
}
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
  Wire.begin(13, 12); //Correct for v2 boards, arguments are (SDA, SCL).
#ifdef DEBUG
  Serial.begin(115200);
#endif
  // attempt to connect to Wifi network:
  WiFi.mode(WIFI_STA);
  
  //No password needed, TinyRoboBase is unecrypted
#ifdef DEBUG
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);  
#endif
  status = WiFi.begin(ssid);//, pass);

  //Wait for the connection
  while ( WiFi.status() != WL_CONNECTED) {
    // let the ESP8266 do stuff and wait for connection:
    yield();
    delay(2000);
    printWiFiStatus();
  }

  //Now connected to WiFi
  //Print the IP address
#ifdef DEBUG
  Serial.print("You're connected to the network ");
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
#endif
  
  //Start the server
  server.begin();

  //Blink the LED to indicate connection
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

/* Protocol from the client is as follows:
   Q - version query, send back some kind of ident string
   M[m1 speed][m1 direction][m2 speed][m2 direction] - motor control command
*/
void loop() {
  WiFiClient client = server.available();
  yield();
  //client goes out of scope when it gets redeclared there
  if (client) {
    state = CLI_READ; //start reading from the client
#ifdef DEBUG    
    Serial.println("Got a client");    
#endif
    while (client.connected()) {
      switch (state)
      {
        case CLI_READ:
          if (client.available()) {
            //Read from client
            char c = client.read();
#ifdef DEBUG
            Serial.print("Read: ");
            Serial.print(c);
            Serial.print(" (");
            Serial.print(c, HEX);
            Serial.println(")");
#endif
            if ( c == 'Q') {
              state = QUERY_RESP;
            } else if ( c == 'M') {
              cmd_index = 0; //start of new motor command
              state = MOTOR_READ;
            } else {
#ifdef DEBUG
              //This is an error, how to deal with it?
              Serial.println("Got something weird");
#endif              
            }
          }
          yield();
          break;
        case QUERY_RESP:
#ifdef DEBUG
          Serial.println("TinyRobo");
#endif          
          client.println("TinyRobo"); //TODO include the fault byte
          state = CLI_READ;
          yield();
          break;
        case MOTOR_READ:
          //Read a byte for motors
          if (client.available()) {
            //Read command from client and store in buffer
            motor_cmd[cmd_index] = client.read();
#ifdef DEBUG
            Serial.print("Motor read: ");
            Serial.print(motor_cmd[cmd_index]);
            Serial.print(" (");
            Serial.print(motor_cmd[cmd_index], HEX);
            Serial.println(")");
#endif            
            cmd_index++;
            //If we have received 4 bytes (2 speed, 2 direction), then change the motors
            if (cmd_index == 4) {
              cmd_index = 0;
              state = MOTOR_DRV;
            }
          }
          yield();
          break;
        case MOTOR_DRV:
          //Set the motor state from what the client sent
          setMotor(addr1, motor_cmd[0], motor_cmd[1]);
          setMotor(addr2, motor_cmd[2], motor_cmd[3]);
          //Get the fault bits
          fault[0] = getFault(addr1);
          fault[1] = getFault(addr2);

#ifdef DEBUG
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
#endif
          //Send it back to the commander as 6 bytes
          uint8_t motStatus[6] = {motor_cmd[0], motor_cmd[1], fault[0], motor_cmd[2], motor_cmd[3], fault[1]};
          //The cast is a hack, may cause nasal demons
          client.write((const uint8_t*)motStatus, 6); 
          state = CLI_READ;
          yield();
          break;
      }
    }
  }
}

