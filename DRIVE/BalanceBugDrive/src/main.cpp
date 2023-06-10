#include <Arduino.h>

#define MANUAL_SPEED_CONTORL true
#define INTERRUPT_INTERVAL 50

const int STPRpin = 25;
const int DIRRpin = 26;
const int STPLpin = 32;
const int DIRLpin = 33;

int i = 0;

// timer globals
hw_timer_t *Timer0 = NULL;

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

  void IRAM_ATTR step()
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

  void IRAM_ATTR setLow()
  {
    digitalWrite(stpPin, LOW);
  }
};

motor left(STPLpin, DIRLpin, 100, SIXT);
motor right(STPRpin, DIRRpin, 100, SIXT);

volatile float rpm;
volatile int accel;
volatile int delaymu;
volatile long int num_delay;
volatile bool isStep;

volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR motorISR() // runs once per mu.s
{
  portENTER_CRITICAL_ISR(&timerMux);
  if (num_delay % 10 == 0 && accel != 0 ) {
    rpm += (accel * 1e-6 * INTERRUPT_INTERVAL);

    if ( rpm > 150 && accel > 0 ) rpm = 150;
    else if (rpm < -150 && accel < 0) rpm = -150;
  }

  if (num_delay >= delaymu / ( 2 * INTERRUPT_INTERVAL )) {
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
  portEXIT_CRITICAL_ISR(&timerMux);

  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

void setup() {

  Timer0 = timerBegin(1, 80, true); // set timer clock to 80MHz/8 = 10Mhz
  timerAttachInterrupt(Timer0, &motorISR, true); 
  timerAlarmWrite(Timer0, INTERRUPT_INTERVAL, true); // trigger interrupt every 10 timer clock cycles 

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  Serial.begin(115200);

  Serial.println("starting!");

  pinMode(STPRpin, OUTPUT);
  pinMode(STPLpin, OUTPUT);
  pinMode(DIRRpin, OUTPUT);
  pinMode(DIRLpin, OUTPUT);

  rpm = 1;
  accel = 0;

  timerAlarmEnable(Timer0);
}

float prop = 20;
void loop() {
  #if MANUAL_SPEED_CONTORL
  if(Serial.available())
  {
    accel = Serial.parseInt();
    Serial.print("accel Set: ");
    Serial.println(accel);
  }
  #endif

  if (rpm < 0)
  {
    left.setDir(BCK);
    right.setDir(BCK);
    // rpm = -rpm;
  }
  else if (rpm > 0)
  {
    left.setDir(FWD);
    right.setDir(FWD);
  }

  if ((int)rpm != 0)
  {
    delaymu = 1e6 * 60 / (abs((int)rpm) * 3200);
  }
  
  delay(10);
}