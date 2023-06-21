#pragma once
#include <Arduino.h>

class LDR
{
  int pin;
public:
  // empty initialiser
  LDR()
  {}

  LDR(int _pin)
  {
    pin = _pin;
  }

  void init()
  {
    pinMode(pin, INPUT);
  }

  int getValue()
  { 
    return analogRead(pin);
  }
};

class MazeLogic
{
  LDR R, FR, F, FL, L;
public:
  MazeLogic(const int LDRpins[5])
  {
    R = LDR(LDRpins[0]);
    FR = LDR(LDRpins[1]);
    F = LDR(LDRpins[2]);
    FL = LDR(LDRpins[3]);
    L = LDR(LDRpins[4]);
  }

  void init()
  {
    R.init();
    FR.init();
    F.init();
    FL.init();
    L.init();
  }

  void printLDRs()
  {
    Serial.printf("Right:%f,FrontRight%f,Front%f,FrontLeft%f,Left:%f\n", R.getValue(), FR.getValue(), F.getValue(), FL.getValue(), L.getValue());
  }
};



