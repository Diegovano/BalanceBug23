#pragma once
#include <Arduino.h>

#include <WiFi.h>
#include <HTTPClient.h>

#include "MazeLogic.cpp"

#define USE_WIFI false
#define STEP_PER_REVOLUTION 3200

const float WHEEL_RADIUS = 3.5;
const float WHEEL_CENTRE_OFFSET = 9.5;

// PIN DEFINITIONS
const int STPRpin = 25;
const int DIRRpin = 26;
const int STPLpin = 32;
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

// HEADING AND DISTANCE TRACKING
enum direction 
{
  FW, BCK, L, R, S
};

#if USE_WIFI
#define WIFI_SSID "iPhone di Luigi"
#define WIFI_PASSWORD "chungusVBD"
#define SERVER_IP "54.82.44.87"

const String SERVER_IP = "192.168.0.99";
const String HTTP_PORT = "3001";

const String motorEndPoint = "http://" + SERVER_IP + ":" + HTTP_PORT + "/api/motor";

HTTPClient http;
#endif

// struct to pass to task scheduler for motor data
// type is 0 for angle, 1 for distance
// value is the double variable
typedef struct {
  int type;
  double val;
} motorParams;


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

void moveAt(double speed){
  setRPM(speed);
  setDir(speed > 0);
}

void turnAt(double speed){
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

void POSTmotorVals(void * payload) {
  #if USE_WIFI
  Serial.println("[POST motor pos] Starting task");
  motorParams *data = (motorParams*)payload; // dodgy pointer casting

  String msgType = "";
  if (data->type == 0) msgType = "angle";
  else if (data->type == 1) msgType = "distance";
  String JSONdata = "{\"type\":\"" + msgType + "\",\"value\":" + String(data->val) + "}";
  
  http.begin(motorEndPoint);
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(JSONdata);

  Serial.print("[POST motor pos]");
  if (httpResponseCode > 0)
  {
    String response = http.getString();
    Serial.print(" POST Response code: " + String(httpResponseCode));
    Serial.println("Response: " + response);
  }
  else
  {
    Serial.println(" Error sending POST request");
  }
  http.end();

  // delete parameters to avoid mem leak
  free(data);

  vTaskDelete(NULL);
  #else
  Serial.println("Cannot POST: WiFi deactivated!");
  #endif
}

void handleNewDirection(direction prevD){
  int steps = getSteps();
  resetSteps();

  // allocate on heap so it can be used to xcreatetask
  motorParams *payload = new motorParams();
  payload->type = -1;

  if (prevD == FW || prevD == BCK){
    double dist = ( double(steps) / STEP_PER_REVOLUTION ) * ( 2 * PI * WHEEL_RADIUS);
    if(prevD == FW) Serial.printf("FW by %.2f cm\n", dist);
    else Serial.printf("BCK by %.2f cm\n", dist);

    payload->type = 1;
    payload->val = (prevD == FW ? dist : -dist);
    // payload = "{\"type\":\"distance\",\"value\":" + (prevD == FW ? String(dist) : String(-dist)) + "}";
  } else if (prevD == L || prevD == R) {
    double angle = steps * (asin((2 * PI * WHEEL_RADIUS) / (WHEEL_CENTRE_OFFSET * STEP_PER_REVOLUTION)) * 180/PI);  // thanks diego
    if(prevD == L) Serial.printf("Left by %.2f degrees\n", angle);
    else Serial.printf("Right by %.2f degrees\n", angle);


    payload->type = 0;
    payload->val = (prevD == R ? angle : -angle);
    // payload = "{\"type\":\"angle\",\"value\":" + (prevD == R ? String(angle) : String(-angle)) + "}";
  } else {
    // Serial.printf("Remained still, %i steps\n", steps);
  }

  #if USE_WIFI
  if (payload->type != -1){
    Serial.println("Starting POST task");
    xTaskCreate(
      POSTmotorVals,
      "Send movement info",
      2000,
      (void*) payload,
      1,
      NULL
    );    
  }
  #endif
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

// R , FR , F , FL , 
const int LDRpins[5] = {4, 2, 15, 12, 13};
MazeLogic labyrinthController(LDRpins);

void setup() {
  Serial.begin(115200);
  Serial.println("starting!");

  labyrinthController.init();

  // pinMode(STPRpin, OUTPUT);
  // pinMode(STPLpin, OUTPUT);
  // pinMode(DIRRpin, OUTPUT);
  // pinMode(DIRLpin, OUTPUT);

  // // set some direction
  // digitalWrite(DIRRpin, HIGH);
  // digitalWrite(DIRLpin, HIGH);

  // step_timer = timerBegin(0, 80, true);
  // timerAttachInterrupt(step_timer, &stepISR, true);

  // control_timer = timerBegin(1, 80, true);
  // timerAttachInterrupt(control_timer, &controlISR, true);
  // timerAlarmWrite(control_timer, 1000 , true);
  // timerAlarmEnable(control_timer);



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

long int stepCounter = 0;
int speed = 10;
direction dir = S;

void loop() {
  // if(Serial.available()){
  //   handleNewDirection(dir);
  //   switch(Serial.read()){
  //     case 'w': {
  //       dir = FW;
  //       moveAt(speed);
  //       break;
  //     }
  //     case 's': {
  //       dir = BCK;
  //       moveAt(-speed);
  //       break;
  //     }
  //     case 'a': {
  //       dir = L;
  //       turnAt(-speed);
  //       break;
  //     }
  //     case 'd': {
  //       dir = R;
  //       turnAt(speed);
  //       break;
  //     }
  //     case ' ': {
  //       dir = S;
  //       moveAt(0);
  //       break;
  //     }
  //     case ('t'): {
  //       Serial.println("Insert turning amount");
  //       double angle = Serial.parseFloat();
  //       turnBy(angle, 30);
  //       Serial.printf("Turn by %.2f degree\n", angle);
  //       break; }
  //     case ('r'): {
  //       Serial.println("Set rpm amount");
  //       double rpm = Serial.parseFloat();
  //       moveAt(rpm);
  //       Serial.printf("Set Speed to: %.2f\n", rpm);
  //       break; }
  //     default:
  //       Serial.println("Option invalid");
  //       dir = S;
  //       moveAt(0);
  //   }
  // } 

  labyrinthController.printLDRs();
  delay(200);
}