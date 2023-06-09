#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <QMC5883LCompass.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define MANUAL_SPEED_CONTORL false

const int STPRpin = 25;
const int DIRRpin = 26;
const int STPLpin = 32;
const int DIRLpin = 33;

int i = 0;

Adafruit_MPU6050 mpu;
QMC5883LCompass compass;

// timer globals
hw_timer_t *Timer0 = NULL;

float filter[10] = {0, 0, 0, 0, 0};
int filterpos = 0;

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
    if (speed != 0 && clk % (101 - speed) == 0)
    {
      digitalWrite(stpPin, HIGH);
    }

    clk++;
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

volatile int rpm;
volatile int delaymu;
volatile long int num_delay;
volatile bool isStep;

void IRAM_ATTR motorISR()
{
  if (num_delay >= delaymu / 2) {
    if (isStep){
      left.step();
      right.step();
      isStep = false;
    } else {
      left.setLow();
      right.setLow();
      isStep = true;
    }
    num_delay = 0;
  } else num_delay++;
// if (num_delay >= 7)
// {
//   if (isStep)
//   {
//     left.step();
//     right.step();
//     isStep = false;
//   }
//   else
//   {
//     left.setLow();
//     right.setLow();
//     isStep = true;
//   }
//   num_delay = 0;
// }
// else num_delay++;
// Serial.println(num_delay);
}

void setup() {

  Timer0 = timerBegin(0, 80, true); // set timer clock to 80MHz/8 = 10Mhz
  timerAttachInterrupt(Timer0, &motorISR, true); 
  timerAlarmWrite(Timer0, 1, true); // trigger interrupt every 10 timer clock cycles 
  timerAlarmEnable(Timer0);

  Serial.begin(115200);

  pinMode(STPRpin, OUTPUT);
  pinMode(STPLpin, OUTPUT);
  pinMode(DIRRpin, OUTPUT);
  pinMode(DIRLpin, OUTPUT);

  rpm = 1;

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

  Wire.setClock(1e5);
}

float prop = 1;
void loop() {
  if (Serial.available())
  {
    prop = Serial.parseFloat();
    Serial.print("Prop term set to ");
    Serial.println(prop);
  }
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // a.acceleration.x = 9.81f;

  mpu.setI2CBypass(true);
  compass.read();
  mpu.setI2CBypass(false);

  float trigpitch = acos(max(-1.0f, min(1.0f, a.acceleration.x/9.81f))) * 180/3.1415f - 90;
  // float trigpitch = 20;

  filter[++filterpos%10] = trigpitch;
  float sum = 0;
  for(int i = 0; i < 10; i++)
  { 
    sum += filter[i];
  }
  float filtered = sum / 10;

  int azi = compass.getAzimuth();

  rpm = filtered * -1 * prop;
  // rpm = 60;
  // rpm = 100;

  if (rpm < 0)
  {
    left.setDir(BCK);
    right.setDir(BCK);
    rpm = -rpm;
  }
  else if (rpm > 0)
  {
    left.setDir(FWD);
    right.setDir(FWD);
  }


  if (rpm != 0)
  {
    delaymu = 1e6 * 60 / (rpm * 3200);
  }

  // Serial.println(rpm);

  #if MANUAL_SPEED_CONTORL
  if(Serial.available())
  {
    rpm = Serial.parseInt();
    Serial.print("Speed Set: ");
    Serial.println(rpm);
  }
  #endif
  
}