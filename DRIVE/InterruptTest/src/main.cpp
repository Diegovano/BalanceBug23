#pragma once
#include <Arduino.h>

#include "secrets.hpp"
#include <WiFi.h>
#include <HTTPClient.h>


#define USE_WIFI true
#define STEP_PER_REVOLUTION 3200

const float WHEEL_RADIUS = 3.5;
const float WHEEL_CENTRE_OFFSET = 9.5;

// include later so constants are included (very dodgy)
#include "MazeLogic.cpp"

// PIN DEFINITIONS
const int STPRpin = 25;
const int DIRRpin = 26;
const int STPLpin = 27;
const int DIRLpin = 33;

// TIMER DEFINITIONS
hw_timer_t *step_timer = NULL;
portMUX_TYPE stepTimerMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t *control_timer = NULL;
portMUX_TYPE controlTimerMux = portMUX_INITIALIZER_UNLOCKED;


// ISR TEP HANDLING
volatile bool isStep = 0;
volatile int ISRstepCounter = 0;
volatile int ISRdegreeStepCount = 0;
volatile bool ISRdegreeControl = 0;
volatile long int ISRcontrolStepCounter = 0;

#if USE_WIFI
// #define WIFI_SSID "iPhone di Luigi"
// #define WIFI_PASSWORD "chungusVBD"
// #define SERVER_IP "54.82.44.87"

const String SERVER_IP = "54.82.44.87";
const String HTTP_PORT = "3001";

const String motorEndPoint = "http://" + SERVER_IP + ":" + HTTP_PORT + "/api/motor";

// HTTPClient http;
#endif




void setDelay(long int delay){
  if (delay == 0 && timerAlarmEnabled(step_timer)){
    timerAlarmDisable(step_timer);
  } else {
    timerAlarmWrite(step_timer, (delay / 2) , true);
    if(!timerAlarmEnabled(step_timer)){
      timerAlarmEnable(step_timer);
    }
  }
}

void setRPM(double rpm){
  setDelay((1e6 * 60) / (abs(rpm) * STEP_PER_REVOLUTION));
}

// positive angle turns right, right wheel goes back and left one goes fowards
void turnBy(double angle, double speed){
  setRPM(0);
  int degreeSteps = abs(angle) / (asin((2 * PI * WHEEL_RADIUS) / (WHEEL_CENTRE_OFFSET * 3200)) * 180/PI); // assume 16th steps

  digitalWrite(DIRRpin, angle < 0);
  digitalWrite(DIRLpin, angle > 0);

  portENTER_CRITICAL(&controlTimerMux);
  ISRdegreeControl = 1;
  ISRdegreeStepCount = degreeSteps;
  portEXIT_CRITICAL(&controlTimerMux);
  setRPM(speed);
}

// set the direction, 1 forward 0 back ?
void setDir(bool direction){
  digitalWrite(DIRRpin, direction);
  digitalWrite(DIRLpin, direction);
}

void setSpeed(double speed){
  setRPM(speed);
  setDir(speed > 0);
}

void setTurnSpeed(double speed){
  setRPM(speed);
  digitalWrite(DIRRpin, speed < 0);
  digitalWrite(DIRLpin, speed > 0);
}

int getSteps(){
  uint32_t steps = 0;
  portENTER_CRITICAL(&stepTimerMux);
  steps = ISRstepCounter;
  portEXIT_CRITICAL(&stepTimerMux);
  return steps;
}

void resetSteps(){
  portENTER_CRITICAL(&stepTimerMux);
  ISRstepCounter = 0;
  portEXIT_CRITICAL(&stepTimerMux);
}

void ARDUINO_ISR_ATTR stepISR(){
  if (isStep) {
    digitalWrite(STPRpin, HIGH);
    digitalWrite(STPLpin, HIGH);
    isStep = 0;
    // critical section of keeping track of steps done
    portENTER_CRITICAL_ISR(&stepTimerMux);
    ISRstepCounter++;
    portEXIT_CRITICAL_ISR(&stepTimerMux);
  } else {
    digitalWrite(STPRpin, LOW);
    digitalWrite(STPLpin, LOW);
    isStep = 1;
  }
}

void ARDUINO_ISR_ATTR controlISR(){
  int32_t ISRstepDiff = 0;
  portENTER_CRITICAL(&stepTimerMux);
  ISRstepDiff = ISRstepCounter - ISRcontrolStepCounter;
  ISRcontrolStepCounter = ISRstepCounter;
  portEXIT_CRITICAL(&stepTimerMux);


  portENTER_CRITICAL_ISR(&controlTimerMux);
  if(ISRdegreeControl) {
    if(ISRdegreeStepCount > 0) {
      ISRdegreeStepCount -= ISRstepDiff;
    } else {
      ISRdegreeStepCount = 0;
      setRPM(0);
      ISRdegreeControl = 0;
    }
  }
  portEXIT_CRITICAL_ISR(&controlTimerMux);
}

// intialize motor controller
WebBuffer *buffer = new WebBuffer(motorEndPoint);
MotorController *motor = new MotorController(&getSteps, &setSpeed, &setTurnSpeed, buffer);
// R , FR , F , FL , L
const int LDRpins[5] = {36, 39, 34, 35, 32};
MazeLogic labyrinthController(LDRpins, motor);

void setup() {
  Serial.begin(115200);
  Serial.println("starting!");


  // MOTOR INIT
  pinMode(STPRpin, OUTPUT);
  pinMode(STPLpin, OUTPUT);
  pinMode(DIRRpin, OUTPUT);
  pinMode(DIRLpin, OUTPUT);

  // set some direction
  digitalWrite(DIRRpin, HIGH);
  digitalWrite(DIRLpin, HIGH);

  step_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(step_timer, &stepISR, true);

  control_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(control_timer, &controlISR, true);
  timerAlarmWrite(control_timer, 1000 , true);
  timerAlarmEnable(control_timer);

  labyrinthController.init();
  labyrinthController.setPrintLevel(2);

  // Wifi Setup
  #if USE_WIFI
  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
  }
  Serial.print("Connected to WiFi as");
  Serial.println(WiFi.localIP());
  digitalWrite(LED_BUILTIN, HIGH);
  #endif
}

int count = 0;

void loop() {
  // if (count % 20 == 0){
  //   labyrinthController.printLDRs();
  // }

  if(Serial.available()){
    switch(Serial.read()){
      case 'w': {
        motor->moveFW();
        Serial.println("Moving forward");
        break;
      }
      case 's': {
        motor->moveBCK();
        Serial.println("Moving Backwards");
        break;
      }
      case 'a': {
        motor->moveTurnLeft();
        Serial.println("Turning Left");
        break;
      }
      case 'd': {
        motor->moveTurnRight();
        Serial.println("Turning Right");
        break;
      }
      case ' ': {
        motor->stop();
        Serial.println("Stopping");
        break;
      }
      default:
        Serial.println("Option invalid");
        motor->stop();
    }
  } 


  labyrinthController.update();

  // buffer->update();
  // count++;
  delay(50);
}