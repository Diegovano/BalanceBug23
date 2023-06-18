#include <Arduino.h>

// PIN DEFINITIONS
const int STPRpin = 25;
const int DIRRpin = 26;
const int STPLpin = 32;
const int DIRLpin = 33;

// TIMER DEFINITIONS
hw_timer_t * step_timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// STEP HANDLING
volatile bool isStep = 0;
long int delaymu = 0;
double rpm = 0;

void stepISR(){
  if (isStep) {
    digitalWrite(STPRpin, HIGH);
    digitalWrite(STPLpin, HIGH);
    isStep = 0;
  } else {
    digitalWrite(STPRpin, LOW);
    digitalWrite(STPLpin, LOW);
    isStep = 1;
  }
  // isStep = ~isStep;
}

void setup() {
  Serial.begin(115200);
  Serial.println("starting!");

  pinMode(STPRpin, OUTPUT);
  pinMode(STPLpin, OUTPUT);
  pinMode(DIRRpin, OUTPUT);
  pinMode(DIRLpin, OUTPUT);

  // set some direction
  digitalWrite(DIRRpin, HIGH);
  digitalWrite(DIRLpin, HIGH);

  rpm = 60;
  delaymu = (1e6 * 60) / (abs(rpm) * 3200);
  // delaymu = 1000000;

  step_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(step_timer, &stepISR, true);
  timerAlarmWrite(step_timer, (delaymu / 2) , true);

  delay(1000);
  Serial.println("Starting timer");

  timerAlarmEnable(step_timer);
}

void loop() {
  if(Serial.available()){
    rpm = Serial.parseFloat();
    delaymu = (1e6 * 60) / (abs(rpm) * 3200);
    timerAlarmWrite(step_timer, (delaymu / 2) , true);
    Serial.printf("%f, %i \n", rpm, delaymu);
  }
  delay(10);
}