#pragma once
#include <Arduino.h>

#define WALL_THRESH 2978
#define MOVE_SPEED 10
#define TURN_SPEED 10 

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
  int moveSpeed, turnSpeed;
  bool turning;
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

  void update()
  {
    if (R.getValue() < WALL_THRESH)
    {
      turning = 0;
      moveSpeed = MOVE_SPEED;
      Serial.printf("See right wall, move forward\n");
    } else {
      turning = 1;
      turnSpeed = TURN_SPEED;
      Serial.printf("Missed right wall, turn right\n");
    }
  }

  // returns speed and sets turn bool
  int getSpeed()
  {
    if (turning) return turnSpeed;
    else moveSpeed;
  }

  bool getTurn()
  {
    return turning;
  }
};



