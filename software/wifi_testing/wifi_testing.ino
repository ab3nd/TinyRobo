
#include <ESP8266WiFi.h>

char ssid[] = "TinyRoboBase";     //  your network SSID (name)
int status = WL_IDLE_STATUS;     // the Wifi radio's status
WiFiServer server(4321); 


void printWiFiStatus()
{
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
}

void setup() {
  // Debug output
  Serial.begin(115200);

  //Connect to AP
  WiFi.mode(WIFI_STA);
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    //No password needed, TinyRoboBase is unecrypted
    status = WiFi.begin(ssid);//, pass);
    // wait 5 seconds for connection:
    delay(5000);
    printWiFiStatus();
  }

  //Now connected to WiFi
  //Print the IP address
  Serial.print("You're connected to the network");
  //IPAddress ip = WiFi.localIP();
  //Serial.print("IP Address: ");
  //Serial.println(ip);
  //Start the server
  //server.begin();
}

void loop() {
  // Wei Wu Wei
}
