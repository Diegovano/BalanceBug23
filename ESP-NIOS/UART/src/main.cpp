#include <Arduino.h>

HardwareSerial NIOS(2);

void setup() {

  NIOS.begin(115200, SERIAL_8N1, 16, 17); 


  Serial.begin(115200);
  Serial.println("Hello brothers!");

  while (!NIOS.available()) {
    delay(500);
    Serial.println("Waiting for message from nios");
  }
  while (NIOS.available()){
    Serial.println("Recevied from nios!");
    Serial.println(NIOS.readString());
  }
}

int i = 0;

void loop() {
  NIOS.print(i++);
  NIOS.println("Luigi");
  delay(5000);
}
