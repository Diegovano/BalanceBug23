#include <Arduino.h>
#define MANUAL_SPEED_CONTORL true

const int MAX_ACCEL = 250;

const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

const int STPRpin = 25;
const int DIRRpin = 26;
const int STPLpin = 32;
const int DIRLpin = 33;

TaskHandle_t stepTask;
TaskHandle_t otherTasks;

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

motor left(STPLpin, DIRLpin, 100, SIXT);
motor right(STPRpin, DIRRpin, 100, SIXT);

float rpm;
int accel;
int delaymu;

void motorCode(void *param) // runs once per mu.s
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
    vTaskDelay(10);
    #if MANUAL_SPEED_CONTORL
    if(Serial.available())
    {
      accel = Serial.parseInt();
      if (abs(accel) > MAX_ACCEL) accel *= ((float)MAX_ACCEL / abs(accel)); // convoluted way to cap acceleration magnitude.
      Serial.print("accel Set: ");

      Serial.println(accel);
    }
    #endif

    if (accel != 0 ) {
      if (abs(accel) > MAX_ACCEL) accel *= ((float)MAX_ACCEL / abs(accel));
      float accel_step = (accel * 1e-3 * 10);
      if ( rpm + accel_step > 150 && accel_step > 0 ) rpm = 150;
      else if (rpm + accel_step < -150 && accel_step < 0) rpm = -150;
      else rpm += accel_step;
    }

    Serial.println(rpm);

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

  rpm = 60;
  accel = 0;

  xTaskCreatePinnedToCore(
    controlCode,
    "Controller Code",
    1024,
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

void loop() {
  //empty
}

// static TaskHandle_t multitask = NULL;

// static TaskHandle_t Task1 = NULL;

// void code(void *param){
//   Serial.print("Task1 is running on core ");
//   Serial.println(xPortGetCoreID());

//   // delay(1000);
//   for(;;){
//     // Serial.println("1 HIGH");
//     // delay(666);
//     // Serial.println("1 LOW");
//     // delay(666);
//   } 
// }

// void setup() {
//   Serial.begin(115200);
//   Serial.println(xPortGetCoreID());
//   xTaskCreatePinnedToCore(code, "MultiCore", 1024, NULL, 1, &Task1, 1);
// }

// void loop() {
//   // Serial.println("hello");
// }