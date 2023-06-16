#include <Arduino.h>

#define MANUAL_SPEED_CONTORL false
#define POS_INPUT true
#define SPE_INPUT false
#define ACC_INPUT false

const float WHEEL_RADIUS = 3.75;

const int MAX_ACCEL = 1500;
const int MAX_SPEED = 100;

const int STPRpin = 25;
const int DIRRpin = 26;
const int STPLpin = 32;
const int DIRLpin = 33;

TaskHandle_t stepTask;
TaskHandle_t otherTasks;

unsigned long time123;

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
  motor(const int pinStepSet, const int pinDirSet, microstep stepSet = SIXT) : stpPin(pinStepSet), dirPin(pinDirSet), stepping(stepSet)
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

float rpm;
float accel;
float position;
int steps;
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
    if (steps > 0)
    {
      steps--;
      if (rpm != 0)
      {
        delaymu = 1e6 * 60 / (abs(rpm) * 3200);

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
    }
    else if (false && rpm != 0)
    {
      delaymu = 1e6 * 60 / (abs(rpm) * 3200);

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

  while (true)
  {
    #if MANUAL_SPEED_CONTORL
    if(Serial.available())
    {
      rpm = Serial.parseInt();
      if (abs(rpm) > MAX_SPEED) rpm *= ((float)MAX_SPEED / abs(rpm)); // convoluted way to cap acceleration magnitude.
      Serial.print("rpm Set: ");

      Serial.println(rpm);
    }
    #endif

    // receive request accel, speed or position?

    // if is position

    #if POS_INPUT

    if(Serial.available())
    {
      position = Serial.parseInt();
      Serial.print("Position Set: ");

      Serial.println(position);
      accel = 0;
      steps = abs(position) / (2 * PI * WHEEL_RADIUS) * 3200; // assuming 16th steps
      if (position * rpm < 0)
      {
        rpm *= -1; // get rpm in correct dir
      }
    }

    // rpm = 0;


    // set direction

    #elif SPE_INPUT

    steps = 0;
    accel = 0;
    rpm = abs(inSpeed) < MAX_SPEED ? inSpeed : ((inSpeed > 0) - (inSpeed < 0)) * MAX_SPEED;

    #elif ACC_INPUT

    steps = 0;
    rpm = 0;
    accel = abs(inAccel) < MAX_ACCEL ? inAccel : ((inAccel > 0) - (inAccel < 0)) * MAX_ACCEL;

    #endif

    // accel = abs(reqAccel) < MAX_ACCEL ? reqAccel : ((reqAccel > 0) - (reqAccel < 0)) * MAX_ACCEL;
    
    if (accel != 0 ) {
      if (abs(accel) > MAX_ACCEL) accel *= ((float)MAX_ACCEL / abs(accel));
      float accel_step = (accel * 1e-3 * 10);
      if ( rpm + accel_step > MAX_SPEED && accel_step > 0 ) rpm = MAX_SPEED;
      else if (rpm + accel_step < -MAX_SPEED && accel_step < 0) rpm = -MAX_SPEED;
      else rpm += accel_step;
    }

    // Serial.println(rpm);


    // *****************************
    //
    //  PLOTTING CODE
    //
    // *****************************

    // Serial.print("\tAccel:");
    // Serial.print(accel);
    // Serial.print("\tRPM:");
    // Serial.println(rpm);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("starting!");

  pinMode(STPRpin, OUTPUT);
  pinMode(STPLpin, OUTPUT);
  pinMode(DIRRpin, OUTPUT);
  pinMode(DIRLpin, OUTPUT);

  rpm = 10;
  accel = 0;
  position = 0;

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
  // empty as core 1 tasks already defined (motorCode)
  // defining the task outself allows priority override.
}