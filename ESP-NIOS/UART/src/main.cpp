#include <Arduino.h>
#include <Stepper.h>

// NIOS defines
#define NIOS_RESET_PIN 18
#define NIOS_UART_RX   17
#define NIOS_UART_TX   16

HardwareSerial NIOS(2);
void reset_NIOS();
void setRED(), setBLUE(), setYELLOW();

// ULN2003 Motor Driver Pins
#define IN1 19
#define IN2 22
#define IN3 5
#define IN4 21

const double stepsPerRevolution = 2037.8864;
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

void cameraSpin(double angle);

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
}

int i = 0;

void loop() {

  switch(i++%3) {
    case 0: {
      Serial.println("Setting RED");
      setRED();
      break;
    }
    case 1: {
      Serial.println("Setting YELLOW");
      setYELLOW();
      break;
    }
    case 2: {
      Serial.println("Setting BLUE");
      setBLUE();
      break;
    }
  }

  delay(4000);

  NIOS.printf("B");

  while(!NIOS.available()){

  }

  while(NIOS.available()) {
    NIOS.readStringUntil('D');
    // int 
    Serial.println(NIOS.readStringUntil('\n'));
    Serial.println(NIOS.readStringUntil('\n'));
    Serial.println(NIOS.readStringUntil('\n'));
  }

  // if(NIOS.available()){
  //   char msgType = NIOS.read();
  //   int value = NIOS.readStringUntil('\n').toInt(); 
  //   Serial.printf("NIOS. type: %c, val: %8x\n", msgType, value);
  // }

  delay(20);
  // NIOS.printf("E%08x\n", i += 2);
  // Serial.printf("Sent message:E%08x\n", i);
  // delay(500);
  delay(500);
}

void reset_NIOS() {
  Serial.println("Resetting NIOS");
  digitalWrite(NIOS_RESET_PIN, LOW);
  delay(20);
  digitalWrite(NIOS_RESET_PIN, HIGH);
  delay(20);
  digitalWrite(NIOS_RESET_PIN, LOW);
}

void setRED(){
  // RED
  NIOS.printf("E%08x\n", 0x3D0);
  delay(50);
  NIOS.printf("G%08x\n", 0x0);
  delay(50);
  NIOS.printf("H%08x\n", 0xFF0000);
  delay(50);
  NIOS.printf("T%08x\n", 0x1e3d);
  delay(50);
  NIOS.printf("A%08x\n", 0x1);
  delay(50);
  NIOS.printf("P%08x\n", 10);
}

void setBLUE(){
  // BLUE
  NIOS.printf("E%08x\n", 0x400);
  delay(50);
  NIOS.printf("G%08x\n", 0x0);
  delay(50);
  NIOS.printf("H%08x\n", 0x0000FF);
  delay(50);
  NIOS.printf("T%08x\n", 0x2db1);
  delay(50);
  NIOS.printf("A%08x\n", 0x1);
  delay(50);
  NIOS.printf("P%08x\n", 10);
} 

void setYELLOW(){
  // YELLOW
  NIOS.printf("E%08x\n", 0xd80);
  delay(50);
  NIOS.printf("G%08x\n", 0x0);
  delay(50);
  NIOS.printf("H%08x\n", 0xEEFF00);
  delay(50);
  NIOS.printf("T%08x\n", 0x1d15);
  delay(50);
  NIOS.printf("A%08x\n", 0x5);
  delay(50);
  NIOS.printf("P%08x\n", 20);
}

void cameraSpin(double angle){
  
}