#include <Arduino.h>

#define NIOS_RESET_PIN 18
#define NIOS_UART_RX   17
#define NIOS_UART_TX   16

HardwareSerial NIOS(2);

void reset_NIOS();

void setup() {



  Serial.begin(115200);
  Serial.println("ESP-32\nImperial College London EE2 Project Magenta");

  pinMode(NIOS_RESET_PIN, OUTPUT);
  reset_NIOS();

  NIOS.begin(115200, SERIAL_8N1, 16, 17); 
  while (!NIOS.available()) {
    delay(500);
    Serial.println("Waiting for message from NIOS");
  }
  while (NIOS.available()){
    Serial.println("Recevied from NIOS:");
    Serial.println(NIOS.readString());
  }
}

int i = 0;

void loop() {
  NIOS.print(i++);
  NIOS.println("Luigi");
  delay(5000);
}

void reset_NIOS() {
  Serial.println("Resetting NIOS");
  digitalWrite(NIOS_RESET_PIN, LOW);
  delay(20);
  digitalWrite(NIOS_RESET_PIN, HIGH);
  delay(20);
  digitalWrite(NIOS_RESET_PIN, LOW);

}