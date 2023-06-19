#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <QMC5883LCompass.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <string>

#define WIFI false

#define WIFI_SSID "Diego-XPS"
#define WIFI_PASSWORD "helloGitHub!"
#define SERVER_IP "54.82.44.87"
#define HTTP_PORT "3001"

#define MANUAL_SPEED_CONTORL false
#define PLOTTING false

std::string posURL = "http://" + std::string(SERVER_IP) + ':' + std::string(HTTP_PORT) + "/posRequest";

HTTPClient http;

const int MAX_ACCEL = 1500;
const int MAX_SPEED = 100;

const int STPRpin = 25;
const int DIRRpin = 26;
const int STPLpin = 32;
const int DIRLpin = 33;

float kp, ki, kd, kpp;
int delaymu = 0;

unsigned long time123;

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
  motor(const int pinStepSet, const int pinDirSet) : stpPin(pinStepSet), dirPin(pinDirSet)
  { 
    pinMode(pinStepSet, OUTPUT);
    pinMode(pinDirSet, OUTPUT);
    digitalWrite(pinStepSet, LOW);
    digitalWrite(pinDirSet, HIGH);
  }

  void step()
  {
    digitalWrite(stpPin, HIGH);
  }

  void setLow()
  {
    digitalWrite(stpPin, LOW);
  }

  void setDir(direction dir)
  {
    if (dir == BCK) digitalWrite(dirPin, HIGH);
    else digitalWrite(dirPin, LOW);
  }
};

double rpm;
double accel;

/// @brief The function which will run on core 1 (application core).
/// This function steps with appropriate delay to acheive the speed as defined by delaymu. This is calculated based on rpm.
/// @param motors array of both motor objects
void motorCode(void *motors)
{
  Serial.print("Starting Motor Code on Core ");
  Serial.println(xPortGetCoreID());

  motor left = ((motor *)motors)[0];
  motor right = ((motor *)motors)[1];

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

void pollServerForDistance(void *_param)
{
  Serial.print("Starting Polling Code on Core ");
  Serial.println(xPortGetCoreID());

  while( true )
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      Serial.println("Connecting to Wi-Fi");

      do  
      {
        Serial.print(".");
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        vTaskDelay(500);
      } while (WiFi.status() != WL_CONNECTED);
    }

    http.begin(posURL.c_str());
    int responseCode = http.GET();

    if (responseCode != -1)
    {
      std::string response = http.getString().c_str();

      // Serial.println(response.c_str());
      Serial.println(10); // constant for now;
    }
    else
    {
      Serial.println("Error Receiving Response!");
      http.end();
    }
    http.end();
    vTaskDelay(1 * 1000);
  }
}


/// @brief The function which will run on core 0 (other core).
/// This function recomputes the values which ultimately drive the motors, based on the control algorithm.
/// @param _param Not used
void controlCode(void *mpu)
{
  Serial.print("Starting Control Code on Core ");
  Serial.println(xPortGetCoreID());

  int fake_bandwidth_watchdog = 0;
  
  unsigned int filterpos = 0;
  double filter0[10];
  double filter1[10];

  PID innerController(kp, ki, kd, 0);

  while (true)
  {
    // Serial.println(millis() - time123);
    time123 = millis();
    vTaskDelay(10);
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
      
      innerController.setgain(kp, ki, kd);
    #endif
    }

    // *****************************
    //
    //  SENSOR CODE
    //
    // *****************************

    sensors_event_t a, g, temp;
    ((Adafruit_MPU6050*)mpu)->getEvent(&a, &g, &temp);

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

    double trigPitch = -(acos(max(-1.0, min(1.0, gravTorque/9.81))) * 180/PI - 90);
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
    innerController.setSetpoint(pitchSetpoint);
    double reqRpm = (double)innerController.compute(trigPitch);
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

    if (rpm != 0)
    {
      delaymu = 1e6 * 60 / (abs(rpm) * 3200); // cast to int
    }

    // Serial.println(rpm);

    // *****************************
    //
    //  PLOTTING CODE
    //
    // *****************************

    #if PLOTTING
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
    #else

    // int distance = pollServerForDistance();

    // Serial.println(distance);

    #endif
  }
}

void setup() {
  motor left(STPLpin, DIRLpin);
  motor right(STPRpin, DIRRpin);

  motor motors[2] = { left, right };

  Adafruit_MPU6050 mpu;
  QMC5883LCompass compass;
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("starting!");
  
  #if WIFI
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

  rpm = 0;
  accel = 0;

  kp = 3;
  ki = 0.0;
  kd = 0.01;
  kpp = 1;

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

  TaskHandle_t pollTask;
  TaskHandle_t otherTasks;
  TaskHandle_t stepTask;

  #if WIFI

  xTaskCreatePinnedToCore(
    pollServerForDistance,
    "ServerPollCode",
    4096,
    NULL,
    1,
    &pollTask,
    0);

  #endif

  delay(500);

  xTaskCreatePinnedToCore(
    controlCode,
    "ControllerCode",
    4096,
    &mpu, // mpu only for now
    tskIDLE_PRIORITY,
    &otherTasks,
    0);

  delay(500);

  xTaskCreatePinnedToCore(
    motorCode,
    "MotorSteppingCode",
    4096,
    motors,
    10,
    &stepTask,
    1);

  vTaskDelete(NULL);
}

void loop() 
{
  // empty as core 1 tasks already defined (motorCode)
  // defining the task outself allows priority override.
}