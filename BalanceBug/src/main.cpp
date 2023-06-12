#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <QMC5883LCompass.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define MANUAL_SPEED_CONTORL false

const int MAX_ACCEL = 1000;
const int MAX_SPEED = 100;

const int STPRpin = 25;
const int DIRRpin = 26;
const int STPLpin = 32;
const int DIRLpin = 33;

TaskHandle_t stepTask;
TaskHandle_t otherTasks;

Adafruit_MPU6050 mpu;
QMC5883LCompass compass;

float filter0[10] = {0, 0, 0, 0, 0};
float filter1[10] = {0, 0, 0, 0, 0};
int filterpos = 0;

float e0, e1, e2, u0, u1 = 0, e_integration, delta_u;
float kp, ki, kd;
float Ts;
float u_max, u_min;

void pid(float e){
    e0 = e;
    e_integration = e0;
    
    //anti_windup
    if(u1 >= u_max){
        e_integration = 0;
    }
    else if (u1 <= u_min)
    {
        e_integration = 0;
    }
    
    //incremental PID
    delta_u = kp*(e0-e1) + ki*Ts*e_integration + kd/Ts*(e0 - 2*e1 + e2);
    u0 = u1 + delta_u;

    e2 = e1;
    e1 = e0;
    u1 = u0;
}

enum direction
{
  BCK, FWD
};

enum microstep
{
  FULL, HALF, QUAR, EIGH, SIXT
};

class motor
{
  const int stpPin;
  const int dirPin;
  direction dir;
  microstep stepping;
  int speed; // between 0 and 100
  int clk = 0;
public:
  motor(const int pinStepSet, const int pinDirSet, int speedSet, microstep stepSet = SIXT) : stpPin(pinStepSet), dirPin(pinDirSet), speed(speedSet), stepping(stepSet)
  { 
    setSpeed(speedSet);
  }

  void step()
  {
    digitalWrite(stpPin, HIGH);
  }

  void setStep(microstep stepSet)
  {
    stepping = stepSet;

    // need code here to set pins
  }

  void setSpeed(int speedSet)
  {
    dir = speedSet < 0 ? BCK : FWD;
    speed = speedSet < -100 || speedSet > 100 ? 100 : abs(speedSet);

    if (dir == BCK) digitalWrite(dirPin, HIGH);
    else digitalWrite(dirPin, LOW);
  }

  void setDir(direction dir)
  {
    if (dir == BCK) digitalWrite(dirPin, HIGH);
    else digitalWrite(dirPin, LOW);
  }

  void setLow()
  {
    digitalWrite(stpPin, LOW);
  }
};

motor left(STPLpin, DIRLpin, 100, QUAR);
motor right(STPRpin, DIRRpin, 100, QUAR);

float rpm;
int accel;
int delaymu;
float prop = 1;

void motorCode(void *param)
{
  Serial.print("Starting Motor Code on Core ");
  Serial.println(xPortGetCoreID());

  while( true ) // loop to run in core
  {
    if (rpm != 0 && delaymu != 0)
    {
      if (rpm < 0)
      {
        left.setDir(BCK);
        right.setDir(BCK);
      }
      else // rpm not equal to 0
      {
        left.setDir(FWD);
        right.setDir(FWD);
      }
      left.step();
      right.step();
      delayMicroseconds(delaymu / 2);
      left.setLow();
      right.setLow();
      delayMicroseconds(delaymu / 2);
    }
    else
    {
      vTaskDelay(1); // delay here?
    }
  }
}

void controlCode(void *param)
{
  Serial.print("Starting Control Code on Core ");
  Serial.println(xPortGetCoreID());

  while (true)
  {
    vTaskDelay(1);
    if(Serial.available())
    {
    #if MANUAL_SPEED_CONTORL
      accel = Serial.parseInt();
      if (abs(accel) > MAX_ACCEL) accel *= ((float)MAX_ACCEL / abs(accel)); // convoluted way to cap acceleration magnitude.
      Serial.print("accel Set: ");

      Serial.println(accel);
    #else
      kp = Serial.parseInt();
      Serial.print("prop term set: ");
      Serial.println(prop);
      rpm = 0;
    #endif
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // a.acceleration.x = 9.81f;

    // mpu.setI2CBypass(true);
    // compass.read();
    // mpu.setI2CBypass(false);

    // float trigpitch = acos(max(-1.0f, min(1.0f, a.acceleration.x/9.81f))) * 180/3.1415f - 90;

    filter0[filterpos%10] = a.acceleration.x;
    float sum = 0;
    for(int i = 0; i < 10; i++)
    { 
      sum += filter0[i];
    }
    float gravTorque = sum / 10;
    
    filter1[++filterpos%10] = g.gyro.x;
    sum = 0;
    for(int i = 0; i < 10; i++)
    { 
      sum += filter1[i];
    }
    float rate = sum + 1.01f;

    float trigpitch = acos(max(-1.0f, min(1.0f, gravTorque/9.81f))) * 180/3.1415f - 90;

    // int azi = compass.getAzimuth();

    float pitchSetpoint = 0;
    float pitchError = pitchSetpoint - trigpitch;

    float rateSetpoint = pitchError * 0.1;

    int RATE_SP_SAT = 50;
    // pitch rate set point sat
    rateSetpoint = abs(rateSetpoint) > RATE_SP_SAT ? abs(rateSetpoint) / RATE_SP_SAT : rateSetpoint; 

    float rateError = rateSetpoint - rate;

    pid(rateError);

    accel = u0;

    // Serial.print("Pitch Setpoint: ");
    // Serial.print(pitchSetpoint);
    Serial.print("\tTrig Pitch: ");
    Serial.print(trigpitch);
    // Serial.print("\tPitch Error: ");
    // Serial.print(pitchError);
    Serial.print("\tRate Setpoint: ");
    Serial.print(rateSetpoint);
    Serial.print("\tRate: ");
    Serial.print(rate);
    Serial.print("\tRate Error: ");
    Serial.print(rateError);
    Serial.print("\tAccel: ");
    Serial.print(accel);
    Serial.print("\tRPM: ");
    Serial.println(rpm);

    if (accel != 0 ) {
      if (abs(accel) > MAX_ACCEL) accel *= ((float)MAX_ACCEL / abs(accel));
      float accel_step = (accel * 1e-3 * 10);
      if ( rpm + accel_step > MAX_SPEED && accel_step > 0 ) rpm = MAX_SPEED;
      else if (rpm + accel_step < -MAX_SPEED && accel_step < 0) rpm = -MAX_SPEED;
      else rpm += accel_step;
    }

    // Serial.println(rpm);

    if ((int)rpm != 0)
    {
      delaymu = 1e6 * 60 / (abs(rpm) * 3200); // cast to int
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("starting!");

  pinMode(STPRpin, OUTPUT);
  pinMode(STPLpin, OUTPUT);
  pinMode(DIRRpin, OUTPUT);
  pinMode(DIRLpin, OUTPUT);

  rpm = 0;
  accel = 0;

  Ts = 0.001;

  kp = 5000;
  ki = 10000;
  kd = 100;

  if (!mpu.begin()) Serial.println("Failed to find MPU6050 chip");
  else 
  {
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    mpu.setI2CBypass(true);
    compass.init();
    mpu.setI2CBypass(false);
  }

  xTaskCreatePinnedToCore(
    controlCode,
    "Controller Code",
    4096,
    NULL,
    tskIDLE_PRIORITY,
    &otherTasks,
    0);

  delay(500);

  xTaskCreatePinnedToCore(
    motorCode,
    "MotorSteppingCode",
    4096,
    NULL,
    10,
    &stepTask,
    1);
    
  vTaskDelete(NULL);

}

void loop() 
{
  // empty
}