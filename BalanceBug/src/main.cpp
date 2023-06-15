#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <QMC5883LCompass.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define MANUAL_SPEED_CONTORL false

const int MAX_ACCEL = 1500;
const int MAX_SPEED = 100;

const int STPRpin = 25;
const int DIRRpin = 26;
const int STPLpin = 32;
const int DIRLpin = 33;

float kp, ki, kd, kpp;

TaskHandle_t stepTask;
TaskHandle_t otherTasks;

Adafruit_MPU6050 mpu;
QMC5883LCompass compass;

unsigned long time123;

double filter0[10] = {0, 0, 0, 0, 0};
double filter1[10] = {0, 0, 0, 0, 0};
int filterpos = 0;

unsigned long lastTime;

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
public:
  PID(float kp, float ki, float kd, float setpoint)
  {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _setpoint = setpoint;
  }

  float compute(float value, float dt = 0)
  {
    if (dt == 0)
    {
      if (lastTime == 0)
      {
        lastTime = millis();
        // dt = lastTime; // use 0 for this time
      }
      unsigned long now = millis();
      dt = lastTime - now;
      lastTime = now;
    }

    e0 = _setpoint - value;
    float delta_u = _kp*(e0 - e1) + _ki*dt*e0 + _kd/dt*(e0 - 2*e1 + e2);
    u0 = delta_u + u1;

    e2 = e1;
    e1 = e0;
    u1 = u0;

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

PID *innerController;

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
public:
  motor(const int pinStepSet, const int pinDirSet, microstep stepSet = SIXT) : stpPin(pinStepSet), dirPin(pinDirSet), speed(speedSet), stepping(stepSet)
  { }

  void step()
  {
    digitalWrite(stpPin, HIGH);
  }

  void setStep(microstep stepSet)
  {
    stepping = stepSet;

    // need code here to set pins
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

motor left(STPLpin, DIRLpin, QUAR);
motor right(STPRpin, DIRRpin, QUAR);

double rpm;
double accel;
int delaymu;

/// @brief The function which will run on core 1 (application core).
/// This function steps with appropriate delay to acheive the speed as defined by delaymu. This is calculated based on rpm.
/// @param _param Not used
void motorCode(void *_param)
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

/// @brief The function which will run on core 0 (other core).
/// This function recomputes the values which ultimately drive the motors, based on the control algorithm.
/// @param _param Not used
void controlCode(void *_param)
{
  Serial.print("Starting Control Code on Core ");
  Serial.println(xPortGetCoreID());

  int fake_bandwidth_watchdog = 0;

  while (true)
  {
    // Serial.println(millis() - time123);
    time123 = millis();
    // vTaskDelay(1);
    if(Serial.available())
    {
    #if MANUAL_SPEED_CONTORL
      accel = Serial.parseInt();
      if (abs(accel) > MAX_ACCEL) accel *= ((float)MAX_ACCEL / abs(accel)); // convoluted way to cap acceleration magnitude.
      Serial.print("accel Set: ");

      Serial.println(accel);
    #else
      kp = Serial.parseFloat();
      ki = Serial.parseFloat();
      kd = Serial.parseFloat();
      // kpp = Serial.parseFloat();
      Serial.print("prop term set: ");
      Serial.println(kp);
      rpm = 0;
      accel = 0;
      
      innerController->setgain(kp, ki, kd);
    #endif
    }

    // *****************************
    //
    //  SENSOR CODE
    //
    // *****************************

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // mpu.setI2CBypass(true);
    // compass.read();
    // mpu.setI2CBypass(false);

    // double trigpitch = acos(max(-1.0f, min(1.0f, a.acceleration.x/9.81f))) * 180/3.1415f - 90;

    filter0[filterpos%10] = a.acceleration.x;
    double sum = 0;
    for(int i = 0; i < 10; i++)
    { 
      sum += filter0[i];
    }
    double gravTorque = sum / 10;
    
    filter1[++filterpos%10] = g.gyro.x;
    sum = 0;
    for(int i = 0; i < 10; i++)
    { 
      sum += filter1[i];
    }
    double rate = sum + 1.01f;

    double accelPitch = atan(a.acceleration.x/a.acceleration.z)*180/PI;
    double accelPitch2 = -atan2(a.acceleration.x, sqrt(a.acceleration.z*a.acceleration.z + a.acceleration.y*a.acceleration.y))*180/PI;
    double gyroPitch = g.gyro.x * 0.004;

    double trigPitch = -(acos(max(-1.0, min(1.0, gravTorque/9.81))) * 180/3.1415 - 90);
    double compPitch = (0.1 * (compPitch + gyroPitch) + 0.9 * accelPitch);

    // BIAS

    trigPitch += 6.4;
    compPitch += 6.4;

    // int azi = compass.getAzimuth();

    // *****************************
    //
    //  CONTROL CODE
    //
    // *****************************

    double pitchSetpoint = -2;
    double pitchError = pitchSetpoint - trigPitch;

    double rateSetpoint = pitchError * kpp;

    int RATE_SP_SAT = 50;
    // pitch rate set point sat
    // rateSetpoint = abs(rateSetpoint) > RATE_SP_SAT ? abs(rateSetpoint) / RATE_SP_SAT : rateSetpoint; 

    // innerController->setSetpoint(rateSetpoint);

    double rateError;
    // if (fake_bandwidth_watchdog % 10 == 0) 
    rateError = rateSetpoint - rate;

    // innerController->setSetpoint(rateSetpoint);
    innerController->setSetpoint(pitchSetpoint);
    double reqRpm = (double)innerController->compute(trigPitch);
    rpm = abs(reqRpm) < MAX_SPEED ? reqRpm : ((reqRpm > 0) - (reqRpm < 0)) * MAX_SPEED;


    // float reqAccel = innerController->compute(rate);

    // accel = abs(reqAccel) < MAX_ACCEL ? reqAccel : ((reqAccel > 0) - (reqAccel < 0)) * MAX_ACCEL;
    
    if (accel != 0 ) {
      if (abs(accel) > MAX_ACCEL) accel *= ((float)MAX_ACCEL / abs(accel));
      double accel_step = (accel * 1e-3 * 10);
      if ( rpm + accel_step > MAX_SPEED && accel_step > 0 ) rpm = MAX_SPEED;
      else if (rpm + accel_step < -MAX_SPEED && accel_step < 0) rpm = -MAX_SPEED;
      else rpm += accel_step;
    }

    // Serial.println(rpm);

    if ((int)rpm != 0)
    {
      delaymu = 1e6 * 60 / (abs(rpm) * 3200); // cast to int
    }

    // *****************************
    //
    //  PLOTTING CODE
    //
    // *****************************

    if(fake_bandwidth_watchdog % 10 == 0)
    {
      // Serial.print("Pitch Setpoint: ");
      // Serial.print(pitchSetpoint);
      Serial.print("TrigPitch:");
      Serial.print(trigPitch);
      // Serial.print("CombPitch:");
      // Serial.print(compPitch);
      // Serial.print("\tCozPitch:");
      // Serial.println(accelPitch2);
      Serial.print("\tPitch Error: ");
      Serial.print(pitchError);
      Serial.print("\tRateSetpoint:");
      Serial.print(rateSetpoint);
      Serial.print("\tRate:");
      Serial.print(rate);
      Serial.print("\tRateError:");
      Serial.print(rateError);
      Serial.print("\tAccel:");
      Serial.print(accel);
      Serial.print("\tRPM:");
      Serial.println(rpm);
    }
    fake_bandwidth_watchdog++;
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

  kp = 5.5;
  ki = 0.0001;
  kd = 0.00000001;
  kpp = 1;

  innerController = new PID(kp, ki, kd, 0);

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
  delete innerController;
}

void loop() 
{
  // empty as core 1 tasks already defined (motorCode)
  // defining the task outself allows priority override.
}