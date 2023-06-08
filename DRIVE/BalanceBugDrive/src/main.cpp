#include <Arduino.h>

const int STPRpin = 25;
const int DIRRpin = 26;
const int STPLpin = 32;
const int DIRLpin = 33;

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

motor left(STPLpin, DIRLpin, 100, SIXT);
motor right(STPRpin, DIRRpin, 100, SIXT);

int rpm = 1;

void setup() {
  Serial.begin(115200);

  pinMode(STPRpin, OUTPUT);
  pinMode(STPLpin, OUTPUT);
  pinMode(DIRRpin, OUTPUT);
  pinMode(DIRLpin, OUTPUT);
}

void loop() {
  if(Serial.available())
  {
    rpm = Serial.parseInt();
    Serial.print("Speed Set: ");
    Serial.println(rpm);
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
  }
  if (rpm != 0)
  {
    int delaymu = 1e6 * 60 / (rpm * 3200);

    left.step();
    right.step();
    delayMicroseconds(delaymu / 2);

    left.setLow();
    right.setLow();
    delayMicroseconds(delaymu / 2);
  }
}