#include <Arduino.h>

#define STEP_PER_REVOLUTION 3200

const float WHEEL_RADIUS = 3.5;
const float WHEEL_CENTRE_OFFSET = 9.5;

// PIN DEFINITIONS
const int STPRpin = 25;
const int DIRRpin = 26;
const int STPLpin = 32;
const int DIRLpin = 33;

// TIMER DEFINITIONS
hw_timer_t * step_timer = NULL;
portMUX_TYPE stepTimerMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t * control_timer = NULL;
portMUX_TYPE controlTimerMux = portMUX_INITIALIZER_UNLOCKED;


// STEP HANDLING
volatile bool isStep = 0;
volatile int ISRstepCounter = 0;
volatile int ISRdegreeStepCount = 0;
volatile bool ISRdegreeControl = 0;
volatile long int ISRcontrolStepCounter = 0;

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
  // Serial.printf("[ctrl]%i,%i\n", ISRstepDiff, ISRcontrolStepCounter);
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

  step_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(step_timer, &stepISR, true);

  control_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(control_timer, &controlISR, true);
  timerAlarmWrite(control_timer, 1000 , true);
  timerAlarmEnable(control_timer);
}

long int stepCounter = 0;

void loop() {
  if(Serial.available()){
    switch(Serial.read()){
      case ('t'): {
        Serial.println("Insert turning amount");
        double angle = Serial.parseFloat();
        turnBy(angle, 30);
        Serial.printf("Turn by %.2f degree\n", angle);
        break; }
      case ('r'): {
        Serial.println("Set rpm amount");
        double rpm = Serial.parseFloat();
        setRPM(rpm);
        setDir(rpm > 0);
        Serial.printf("Set Speed to: %.2f\n", rpm);
        break; }
      default:
        Serial.println("Option invalid");
    }
  }

  // Step counting
  // uint32_t ISRstepCount = 0;
  // portENTER_CRITICAL(&stepTimerMux);
  // ISRstepCount = ISRstepCounter;
  // portEXIT_CRITICAL(&stepTimerMux);

  // Serial.printf("Performed %i steps, total %i\n", ISRstepCount - stepCounter, stepCounter = ISRstepCount);
  delay(100);
}