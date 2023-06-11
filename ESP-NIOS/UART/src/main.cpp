#include <Arduino.h>

#define NIOS_RESET_PIN 18
#define NIOS_UART_RX   17
#define NIOS_UART_TX   16

HardwareSerial NIOS(2);

void reset_NIOS();

void setup() {



  Serial.begin(115200);
  Serial.println("ESP-32\nImperial College London EE2 Project Magenta");


  // NIOS SETUP
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

  delay(500);
  NIOS.printf("E%08x\n", 0x2d0);
  delay(50);
  NIOS.printf("G%08x\n", 0x0);
  delay(50);
  NIOS.printf("H%08x\n", 0x0000FF);
  delay(50);
  NIOS.printf("T%08x\n", 0x2ddd);
  delay(50);
  NIOS.printf("A%08x\n", 0x5);


}

int i = 0;

void loop() {

  

  // if(NIOS.available()){
  //   char msgType = NIOS.read();
  //   int value = NIOS.readStringUntil('\n').toInt(); 
  //   Serial.printf("NIOS. type: %c, val: %8x\n", msgType, value);
  // }

  // delay(20);
  // NIOS.printf("E%08x\n", i += 2);
  // Serial.printf("Sent message:E%08x\n", i);
  // delay(500);
}

void reset_NIOS() {
  Serial.println("Resetting NIOS");
  digitalWrite(NIOS_RESET_PIN, LOW);
  delay(20);
  digitalWrite(NIOS_RESET_PIN, HIGH);
  delay(20);
  digitalWrite(NIOS_RESET_PIN, LOW);

}