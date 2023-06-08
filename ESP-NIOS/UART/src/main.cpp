#include <Arduino.h>

HardwareSerial SerialPort(2);

void setup() {

  SerialPort.begin(115200, SERIAL_8N1, 16, 17); 


  Serial.begin(9600);
  Serial.println("Hello brothers!");
}

void loop() {
  if (SerialPort.available()){
    Serial.println(SerialPort.read());
  }
}
