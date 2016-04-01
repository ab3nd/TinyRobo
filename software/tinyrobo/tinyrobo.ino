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

enum states {
  CONN_WAIT,
  CONNECTED,
  CLI_READ,
  QUERY_RESP,
  MOTOR_READ,
  MOTOR_DRV,
}

//Stores the motor commands [speed1, direction1, speed2, direction2] 
//received from the client
byte motor_cmd[4] = {0, 0, 0, 0};
int cmd_index = 0;

//Stores fault codes (one per motor) to send to client
byte fault[2] = 0x00;

enum states state = CONN_WAIT; //Waiting for connection from client

void setup() {
  Wire.begin(14, 12); //May not be right for the actual boards.
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
  switch ()
  {
    case CONN_WAIT:
      WiFiClient client = server.available();
      if (client)
      {
        Serial.println("Got a client");
        state = CONNECTED;
      }
      break;
    case CONNECTED:
      if (client.connected()) {
        Serial.println("Ready to read");
        state = CLI_READ;
      }
      else {
        state = CONN_WAIT;
      }
      break;
    case CLI_READ:
      if (client.available()) {
        //Read from client
        char c = client.read();
        if ( c == 'Q') {
          state = QUERY_RESP;
        } else if ( c == "M") {
          cmd_index = 0; //start of new motor command
          state = MOTOR_READ;
        } else {
          //This is an error, how to deal with it?
        }
        break;
      }
    case QUERY_RESP:
      server.println("TinyRobo"); //TODO include the fault byte
      break;
    case MOTOR_READ:
      //Read a byte for motors
      if (client.available()) {
        //Read command from client and store in buffer
        motor_cmd[cmd_index] = client.read();
        cmd_index++;
        //If we have received 4 bytes (2 speed, 2 direction), then change the motors
        if (cmd_index == 4) {
          cmd_index = 0;
          state = MOTOR_DRV;
        }
      }
      break;
    case MOTOR_DRV:
      //TODO Set the motor state
      //TODO Get the fault bits
      Serial.println("Set motor state");
      state = CONNECTED;
      break;
  }
}

  //  // print your MAC address:
  //  byte mac[6];
  //  WiFi.macAddress(mac);
  //  Serial.print("MAC address: ");
  //  Serial.print(mac[5],HEX);
  //  Serial.print(":");
  //  Serial.print(mac[4],HEX);
  //  Serial.print(":");
  //  Serial.print(mac[3],HEX);
  //  Serial.print(":");
  //  Serial.print(mac[2],HEX);
  //  Serial.print(":");
  //  Serial.print(mac[1],HEX);
  //  Serial.print(":");
  //  Serial.println(mac[0],HEX);


  //  Wire.beginTransmission(addr1); // Target device
  //  Wire.write(byte(0x01));        // sets register pointer to fault register
  //  Wire.endTransmission();        // stop transmitting
  //
  //  Wire.requestFrom(addr1, 1);
  //  if(Wire.available())
  //  {
  //    faultVals = Wire.read();
  //    Serial.println(faultVals, HEX);
  //  }

  //  Wire.beginTransmission(addr1);
  //  Wire.write(byte(0x00)); //Control register
  //  Wire.write(byte(0x1F << 2) | byte(0x01)); //Moderate speed
  //  Wire.endTransmission();
  //
  //  delay(700);
  //
  //  Wire.beginTransmission(addr1);
  //  Wire.write(byte(0x00)); //Control register
  //  Wire.write(byte(0x00)); //Stop
  //  Wire.endTransmission();
  //
  //  delay(700);
  //
  //  Wire.beginTransmission(addr1);
  //  Wire.write(byte(0x00)); //Control register
  //  Wire.write(byte(0x1F << 2) | byte(0x02)); //Moderate speed, other way
  //  Wire.endTransmission();
  //
  //  delay(700);
  //
  //  Wire.beginTransmission(addr1);
  //  Wire.write(byte(0x00)); //Control register
  //  Wire.write(byte(0x00)); //Stop
  //  Wire.endTransmission();
  //
  //  delay(700);
  //
  //}


