
/* Blink the LED on GPIO 2 for the ESP8266
 *  
 */
void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
}

void loop() {
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(500);
}
