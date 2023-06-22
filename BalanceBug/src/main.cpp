#include <Arduino.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_MPU6050.h>
#include "WebSocketsClient.h"

#define USE_SERIAL Serial

#define USE_WIFI true
#define WEBSOCKET true
#define STEP_PER_REVOLUTION 3200

const float WHEEL_RADIUS = 3.5;
const float WHEEL_CENTRE_OFFSET = 9.5;

// PIN DEFINITIONS
const int STPRpin = 25;
const int DIRRpin = 26;
const int STPLpin = 32;
const int DIRLpin = 33;

WebSocketsClient webSocket;

// TIMER DEFINITIONS
hw_timer_t *step_timer = NULL;
portMUX_TYPE stepTimerMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t *control_timer = NULL;
portMUX_TYPE controlTimerMux = portMUX_INITIALIZER_UNLOCKED;
#define CONTROL_PERIOD 50 // in microseconds

// ISR TEP HANDLING
volatile bool isStep = 0;
volatile int ISRstepCounter = 0;
volatile int ISRdegreeStepCount = 0;
volatile bool ISRdegreeControl = 0;
volatile long int ISRcontrolStepCounter = 0;

// control ISR stuff
volatile double ISRmotorAccel = 0;
volatile double ISRmotorSpeed = 0;

// KALMAN FILTERING
  float Q_angle  =  0.001;   //0.005
  float Q_gyro   =  0.003;  //0.0003
  float R_angle  =  0.03;     //0.008

  float x_angle = 0, x_bias = 0;
  float P_00 = 1000, P_01 = 0, P_10 = 0, P_11 = 1000;	
  float  y, S_var;
  float K_0, K_1;
class Cozzy_Kalman_filter_pitch {
    private:
        unsigned long T;
        double theta[2];
        double P[2][2];
        double y;
        double K[2];
        double Q[2];
        double R;
        double S;
    public:
        Cozzy_Kalman_filter_pitch(double Q_theta,double Q_theta_rate,double R_variance) {
            theta[0] = 0;
            theta[1] = 0; 
            P[0][0] = 1000;
            P[0][1] = 0;
            P[1][0] = 0;
            P[1][1] = 1000;
            K[0] = 0;
            K[1] = 0;
            R = R_variance;
            Q[0] = Q_theta;
            Q[1] = Q_theta_rate;
        }
        void init(){
            T = micros();
        }
        double filter(double angle_rate,double pitch){
            double delta_t = double(micros() - T)/1000000;
            T = micros();
            theta[0] += delta_t*(angle_rate-theta[1]);
            P[0][0] += delta_t*(delta_t*P[1][1]-P[0][1]-P[1][0]+Q[0]);
            P[0][1] -= delta_t*P[1][1];
            P[1][0] -= delta_t*P[1][1];
            P[1][1] += delta_t*Q[1];
            y = pitch - theta[0];
            S = P[0][0] + R;
            K[0] = P[0][0]/S;
            K[1] = P[1][0]/S;
            theta[0] += K[0]*y; 
            theta[1] += K[1]*y; 
            P[1][0] -= K[1]*P[0][0];
            P[1][1] -= K[1]*P[0][1];
            P[0][0] -= K[0]*P[0][0];
            P[0][1] -= K[0]*P[0][1];
            return theta[0];
        }
        double get_bias(){
          return theta[1];
        }
};

// HEADING AND DISTANCE TRACKING
enum direction 
{
  FW, BCK, L, R, S
};

Adafruit_MPU6050 mpu;

// PID

float kpr = 500;
float kir = 1;
float kdr = 1;
float kpp = 10;
float last_value = 0;
float integral_error = 0;
float ur_max = 5000;

float kps = 0;
float kis = 0;
float kds = 0;
float us_max = 200;
float speedSetpoint = 0;

class PID
{
private:

  float _kp;
  float _ki;
  float _kd;
  float e0 = 0, e1 = 0, e2 = 0;
  float u0 = 0, u1 = 0;
  float _setpoint;
  float lastTime;
  float _u_max;
public:
  PID(float kp, float ki, float kd, float setpoint, float u_max)
  {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _setpoint = setpoint;
    _u_max = u_max;
  }

  float compute(float value, float dt = 0)
  {
    // if (dt == 0)
    // {
    //   if (lastTime == 0)
    //   {
    //     lastTime = micros();
    //     // dt = lastTime; // use 0 for this time
    //   }
    //   unsigned long now = micros();
    //   dt = lastTime - now;
    //   lastTime = now;
    // }

    e0 = _setpoint - value;
    float delta_u = _kp*(e0 - e1) + _ki*dt*e0 + _kd/dt*(e0 - 2*e1 + e2);
    u0 = delta_u + u1;

    e2 = e1;
    e1 = e0;
    u1 = u0;

    if (abs(u1) >= _u_max) u1 = _u_max * (u1 < 0 ? -1 : 1);

    return u1;
  }

  void setgain(float kp, float ki, float kd)
  {
    _kp = kp;
    _ki = ki;
    _kd = kd;

    e0 = 0, e1 = 0, e2 = 0, u0 = 0, u1 = 0;

    Serial.print("Set PID values to: ");
    Serial.print(kp);
    Serial.print(", ");
    Serial.print(ki);
    Serial.print(", ");
    Serial.println(kd);
  }

  void setSetpoint(float setpoint)
  {
    _setpoint = setpoint;
  }
};

PID *rateControl;
PID *speedControl;

#if USE_WIFI
#define WIFI_SSID "Diego-XPS"
#define WIFI_PASSWORD "helloGitHub!"

const String SERVER_IP = "54.82.44.87";
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
  if ( delay > 10) { // hard limit on how small the delay can be
  if (delay == 0 && timerAlarmEnabled(step_timer)){
    timerAlarmDisable(step_timer);
  } else {
    timerAlarmWrite(step_timer, (delay / 2) , true);
    if(!timerAlarmEnabled(step_timer)){
      timerAlarmEnable(step_timer);
    }
  } 
  }
}

void setRPM(double rpm){
  // if (rpm > 150) rpm = 150; // do not exceed
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

double getSpeed(){
  double speed = 0;
  portENTER_CRITICAL(&controlTimerMux);
  speed = ISRmotorSpeed;
  portEXIT_CRITICAL(&controlTimerMux);
  return speed;
}

void resetSteps(){
  portENTER_CRITICAL(&stepTimerMux);
  ISRstepCounter = 0;
  portEXIT_CRITICAL(&stepTimerMux);
}

void setAcceleration(const double &accel){
  portENTER_CRITICAL(&controlTimerMux);
  ISRmotorAccel = accel;
  portEXIT_CRITICAL(&controlTimerMux);
}

void setSpeed(const double &speed){
  portENTER_CRITICAL(&controlTimerMux);
  ISRmotorSpeed = speed;
  portEXIT_CRITICAL(&controlTimerMux);
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
  double accel_value = 0, speed = 0;
  portENTER_CRITICAL(&controlTimerMux);
  accel_value = ISRmotorAccel;
  speed = ISRmotorSpeed;
  portEXIT_CRITICAL(&controlTimerMux);

  speed += accel_value * (CONTROL_PERIOD / (double) 1000000);
 // if(speed >= 150){
    //speed = 150;
 // }
  //if(speed <= -150){
    //speed = -150;
  //}
  if (abs(speed) >= 100) speed = 100 * (speed < 0 ? -1 : 1); // set max speed
  moveAt(speed);

  portENTER_CRITICAL(&controlTimerMux);
  ISRmotorSpeed = speed;
  portEXIT_CRITICAL(&controlTimerMux);
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

	switch(type) {
		case WStype_DISCONNECTED:
			USE_SERIAL.printf("[WSc] Disconnected!\n");
			break;
		case WStype_CONNECTED:
			USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);

			// send message to server when Connected
			webSocket.sendTXT("Connected");
			break;
		case WStype_TEXT:
			USE_SERIAL.printf("[WSc] get text: %s\n", payload);

			// send message to server
			// webSocket.sendTXT("message here");
			break;
		case WStype_BIN:
			USE_SERIAL.printf("[WSc] get binary length: %u\n", length);

			// send data to server
			// webSocket.sendBIN(payload, length);
			break;
		case WStype_ERROR:			
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;
	}

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
  digitalWrite(DIRLpin, HIGH  );

  step_timer = timerBegin(3, 80, true);
  timerAttachInterrupt(step_timer, &stepISR, true);

  control_timer = timerBegin(2, 80, true);
  timerAttachInterrupt(control_timer, &controlISR, true);
  timerAlarmWrite(control_timer, CONTROL_PERIOD , true);
  timerAlarmEnable(control_timer);

  rateControl = new PID(kpr, kir, kdr, 0, ur_max);

  speedSetpoint = 0;
  speedControl = new PID(kps, kis, kds, speedSetpoint, us_max);

  if (!mpu.begin()) Serial.println("Failed to find MPU6050 chip");
  else 
  {
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
    // mpu.setI2CBypass(true);
  }

  
  // lastRun = millis();

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
  #if WEBSOCKET
  // WEBSOCKET

  webSocket.begin("127.0.0.1", 8080, "/");

  webSocket.onEvent(webSocketEvent);

  webSocket.setReconnectInterval(5000);


  #endif
}

long int stepCounter = 0;
double speed = 0;
direction dir = S;

unsigned long lastRun;
double prevPitch = 0;

unsigned int loopCount = 0;
unsigned int filterCount = 0;
double trigFilter[5];
double filteredPitch = 0;

Cozzy_Kalman_filter_pitch Kalman(0.001, 0.003, 0.03);

void loop() {
  unsigned long now = micros();
  if(Serial.available())
  {
    kps = Serial.parseFloat();
    kis = Serial.parseFloat();
    kds = Serial.parseFloat();
    //kpp = Serial.parseFloat();

    String bruh = Serial.readString(); // flush any values left

    speedControl->setgain(kpr, kir, kdr);
    Serial.printf("prop term set to %f\n", kpp);


    setSpeed(0);
    setAcceleration(0);
    delay(500);
    lastRun = micros();
  }
  static unsigned long timestamp = micros();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // double filter0[5];

  // filter0[filterpos++%5] = a.acceleration.x;
  // if (filterpos < 5) filter0[filterpos++] = a.acceleration.x;
  // else filter0[filterpos = 0] = a.acceleration.x;

  // double gravTorque = 0;
  // for(int i = 0; i < 5; i++)
  // { 
  //   gravTorque += filter0[i];
  // }
  // gravTorque /= 5;

  // double trigPitch = -(acos(max(-1.0, min(1.0, gravTorque/9.81))) * 180/PI - 90);
  // double trigPitch = -(acos(max(-1.0, min(1.0,  a.acceleration.x/9.81))) * 180/PI - 90);
  double AnglePitch = atan2(a.acceleration.x,sqrt(a.acceleration.y*a.acceleration.y+a.acceleration.z*a.acceleration.z));


  if (filterCount < 5) trigFilter[filterCount++] = AnglePitch;
  else trigFilter[filterCount = 0] = AnglePitch;

  filteredPitch = 0;
  for(int i = 0; i < 5; i++){
    filteredPitch += trigFilter[i];
  }

  filteredPitch /= 5;


  // kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  // KalmanAngleRoll=Kalman1DOutput[0]; 
  // KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  
  double time_step = (double) (now - timestamp) / 1000000;

  double gyroRead = -(g.gyro.y);
  double KalmanPitch = Kalman.filter(gyroRead,AnglePitch);

  double pitchSetpoint = 0; //speedControl->compute(speed, time_step);
  double pitchError = pitchSetpoint - KalmanPitch;

  double rateSetpoint = pitchError * kpp;

  // double gyroRead = g.gyro.y + 0.1;
  // double rateError = rateSetpoint - gyroXread;
  
  timestamp = now;

  rateControl->setSetpoint(rateSetpoint);
  float reqAccel = rateControl->compute(gyroRead - Kalman.get_bias(), time_step);
  if (abs(reqAccel) >= 5000) reqAccel = 5000 * (reqAccel < 0 ? -1 : 1);
  setAcceleration(reqAccel);
  /*float error = rateSetpoint - gyroRead;
  float de = -(error - last_value)/time_step;
  integral_error += ki * error * time_step;;
  last_value = error;
  float reqAccel = kp * error + integral_error + kd * de;
  setAcceleration(reqAccel);*/



  // // speed += reqAccel * (now - lastRun) * 0.001;

  // // now = micros();
  // // double pitchRateComp = (trigPitch - prevPitch) / (now - lastRun);

  if (loopCount++%25) Serial.printf("Speed:%f,Acceleration:%f,rateSetpoint:%f,pitchSetpoint:%f,AngleFilter:%f,Kalman:%f,PitchRateMeasure:%f,TimeStep:%f\n", getSpeed(), reqAccel, rateSetpoint, pitchSetpoint, filteredPitch, KalmanPitch, gyroRead,time_step);

  // // moveAt(speed);
  // prevPitch = trigPitch;


  // Serial.printf("Speed:%f\n", getSpeed());
  // delay(1);
}
