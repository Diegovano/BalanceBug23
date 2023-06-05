#include <Arduino.h>

const int STEPRpin = 32;
const int STEPLpin = 33;

void setup() {
  Serial.begin(115200);

  pinMode(STEPRpin, OUTPUT);
  pinMode(STEPLpin, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  digitalWrite(25, HIGH);
  digitalWrite(26, HIGH);
  digitalWrite(27, HIGH);
}

void loop() {
  digitalWrite(STEPRpin, HIGH);
  digitalWrite(STEPLpin, HIGH);
  delayMicroseconds(1000);
  digitalWrite(STEPRpin, LOW);
  digitalWrite(STEPLpin, LOW);
  delayMicroseconds(1000);
}