#include <Arduino.h>
#include <AccelStepper.h>

// AccelStepper leftOther(AccelStepper::DRIVER, 25, 26);

const int STPRpin = 25;
const int DIRRpin = 26;
const int STPLpin = 32;
const int DIRLpin = 33;

int i = 0;

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

  void setLow()
  {
    digitalWrite(stpPin, LOW);
  }
};

motor left(STPLpin, DIRLpin, 100, SIXT);
motor right(STPRpin, DIRRpin, 100, SIXT);

void setup() {
  Serial.begin(115200);

  pinMode(STPRpin, OUTPUT);
  pinMode(STPLpin, OUTPUT);
  pinMode(DIRRpin, OUTPUT);
  pinMode(DIRLpin, OUTPUT);

  // leftOther.setMaxSpeed(5000);
  // leftOther.setSpeed(5000);
}

void loop() {
  // digitalWrite(STPRpin, HIGH);
  // digitalWrite(STPLpin, HIGH);
  left.step();
  right.step();
  delayMicroseconds(100);
  // // // digitalWrite(STPRpin, LOW);
  // // // digitalWrite(STPLpin, LOW);
  left.setLow();
  right.setLow();
  delayMicroseconds(100);

  // leftOther.runSpeed();

  // left.setSpeed(((i / 500) % 200) - 100);
  // right.setSpeed(((i / 500) % 200) - 100);
  // Serial.println(((i++ / 500) % 200) - 100);
}