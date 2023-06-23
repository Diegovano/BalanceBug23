#pragma once
#include <Arduino.h>

#include <vector>

// struct to pass to task scheduler for motor data
// type is 0 for angle, 1 for distance
// value is the double variable
typedef struct
{
  int type;
  double val;
} motorVals;

class WebBuffer
{
  String ServerEndpoint;
  std::vector<motorVals> buffer;
  unsigned long prevTime;
  int minTime = 1000000; // us, 0.1 seconds

public:
  WebBuffer(String endpoint)
      : ServerEndpoint(endpoint)
  {
  }

  void add(motorVals content)
  {
    buffer.push_back(content);
  }

  void update()
  {
    unsigned long now = micros();
    unsigned long time = now - prevTime;

    if (time > minTime && buffer.size() > 0)
    {
      prevTime = now;
      Serial.printf("[WebBufffer] Sending buffer of size %i\n", buffer.size());

      xTaskCreate(
          &send,
          "Send Buffer",
          4000,
          this,
          1,
          NULL);
    }
  }

  // function runs in separate thread
  static void send(void *bufferInstance)
  {
    WebBuffer *buf = (WebBuffer *)bufferInstance; // dodgy casting to retrieve buffer state;
    Serial.println("[WebBufffer][POST] Start sending");
    HTTPClient http;
    String msgType = "";
    String JSONdata = "[";

    for (int i = 0; i < buf->buffer.size(); i++)
    {
      motorVals val = buf->buffer[i];
      if (val.type == 0)
        msgType = "angle";
      else if (val.type == 1)
        msgType = "distance";
      JSONdata += "{\"type\":\"" + msgType + "\",\"value\":" + String(val.val) + "}";
      if (i < buf->buffer.size() - 1)
        JSONdata += ",";
    }

    buf->buffer.clear();

    JSONdata += "]";

    http.begin(buf->ServerEndpoint);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(JSONdata);

    Serial.print("[POST] ");
    if (httpResponseCode > 0)
    {
      String response = http.getString();
      Serial.print(" POST Response code: " + String(httpResponseCode));
      Serial.println(" Response: " + response);
    }
    else
    {
      Serial.println(" Error sending POST request");
    }
    http.end();

    vTaskDelete(NULL);
  }
};

enum LDRstate
{
  Wall, noWall
};

class LDR
{
  int pin;
  LDRstate state;
  String position;

public:
  // empty initialiser
  LDR()
  {
  }

  LDR(int _pin)
  {
    pin = _pin;
  }

  void init()
  {
    pinMode(pin, INPUT);
  }

  void setState(LDRstate _state)
  {
    state = _state;
  }

  LDRstate getState()
  {
    return state;
  }

  int getValue()
  {
    return analogRead(pin);
  }

  void setPos(String _pos)
  {
    position = _pos;
  }

  String getPos()
  {
    return position;
  }

  String print()
  {
    String val;
    if (state == Wall)
      val = "close";
    // else if (state == inRange)
    //   val = "inRange";
    else if (state == noWall)
      val = "away";
    return position + " " + val;
  }
};

enum direction
{
  FW,
  BCK,
  L,
  R,
  S
};

class MotorController
{
  int (*getSteps)();
  void (*setSpeed)(double), (*setTurnSpeed)(double);
  void (*setTurnBy)(double,double);
  void (*setMoveBy)(double,double);
  int prevSteps;
  direction dir;

  int moveSpeed = 10, turnSpeed = 10;

  WebBuffer *buf;

public:
  MotorController(int (*_getSteps)(), void (*_setSpeed)(double), void (*_setTurnSpeed)(double), void (*_turnBy)(double,double), void (*_moveBy)(double,double), WebBuffer *_buf)
      : getSteps(_getSteps), setSpeed(_setSpeed), setTurnSpeed(_setTurnSpeed), setTurnBy(_turnBy), setMoveBy(_moveBy), buf(_buf)
  {
    Serial.println("[Motor Control] Class initialised");
  }

  // MotorController(int (*_getSteps)(), void (*_setSpeed)(double), void (*_setTurnSpeed)(double))
  //     : getSteps(_getSteps), setSpeed(_setSpeed), setTurnSpeed(_setTurnSpeed)
  // {
  //   Serial.println("[Motor Control] Class initialised with no Buffer");
  // }

  void computeDistanceTravelled()
  {
    int steps = getSteps();
    int delta_steps = steps - prevSteps;
    prevSteps = getSteps();

    motorVals payload;
    payload.type = -1;

    if (dir == FW || dir == BCK)
    {
      double dist = (double(delta_steps) / STEP_PER_REVOLUTION) * (2 * PI * WHEEL_RADIUS);
      if (dir == FW)
        Serial.printf("[Motor Control] FW by %.2f cm\n", dist);
      else
        Serial.printf("[Motor Control] BCK by %.2f cm\n", dist);

      payload.type = 1;
      payload.val = (dir == FW ? dist : -dist);

      // payload = "{\"type\":\"distance\",\"value\":" + (prevD == FW ? String(dist) : String(-dist)) + "}";
    }
    else if (dir == L || dir == R)
    {
      double angle = delta_steps * (asin((2 * PI * WHEEL_RADIUS) / (WHEEL_CENTRE_OFFSET * STEP_PER_REVOLUTION)) * 180 / PI); // thanks diego
      if (dir == L)
        Serial.printf("[Motor Control] Left by %.2f degrees\n", angle);
      else
        Serial.printf("[Motor Control] Right by %.2f degrees\n", angle);

      payload.type = 0;
      payload.val = (dir == R ? angle : -angle);

      // payload = "{\"type\":\"angle\",\"value\":" + (prevD == R ? String(angle) : String(-angle)) + "}";
    }
    else
    {
      // Serial.printf("Remained still, %i steps\n", steps);
    }

    if (payload.type != -1)
    {
      buf->add(payload);
    }
  }

  void moveFW()
  {
    computeDistanceTravelled();
    dir = FW;
    setSpeed(moveSpeed);
  }

  void moveBCK()
  {
    computeDistanceTravelled();
    dir = BCK;
    setSpeed(-moveSpeed);
  }

  void moveTurnRight()
  {
    computeDistanceTravelled();
    dir = R;
    setTurnSpeed(turnSpeed);
  }

  void moveTurnLeft()
  {
    computeDistanceTravelled();
    dir = L;
    setTurnSpeed(-turnSpeed);
  }

  void stop()
  {
    computeDistanceTravelled();
    dir = S;
    setSpeed(0);
  }

  void turnByAngle(double angle)
  {
    computeDistanceTravelled();

    dir = angle < 0 ? L : R;

    setTurnBy(angle, turnSpeed);
    stop();
  }

  void moveByDist(double dist)
  {
    computeDistanceTravelled();

    dir = dist < 0 ? BCK : FW;

    setMoveBy(dist, moveSpeed);
    stop();
  }
};

class MazeLogic
{
  LDR sens[5]; // R, FR, F, FL,
  int moveSpeed, turnSpeed;
  bool turning;
  MotorController *motor;
  bool R, FR, F, FL, L;
  const int minWallThresh = 2000, maxWallThresh = 800;
  int printLevel = 0;

  unsigned int prevPathTime = 0;
  bool closeLeft, closeRight;

  bool pathFollow = true;
  bool makeNode = false;
  bool missingWalls  = -1; // 0 for right, 1 for left and 2 for both 

public:
  MazeLogic(const int LDRpins[5], MotorController *_motor)
  {
    for (int i = 0; i < 5; i++)
    {
      sens[i] = LDR(LDRpins[i]);
    }
    sens[0].setPos("Right");
    sens[1].setPos("Front Right");
    sens[2].setPos("Front");
    sens[3].setPos("Front Left");
    sens[4].setPos("Left");

    motor = _motor;
  }

  void init()
  {
    for (int i = 0; i < 5; i++)
    {
      sens[i].init();
    }
    motor->stop();
  }

  void printLDRs()
  {
    Serial.printf("VAL|Right:%04i,FrontRight:%04i,Front:%04i,FrontLeft:%04i,Left:%04i\n", sens[0].getValue(), sens[1].getValue(), sens[2].getValue(), sens[3].getValue(), sens[4].getValue());
  }

  void LDRoffsets()
  {
    int sum = 0;
    int vals[5];
    for (int i = 0; i < 5; i++)
    {
      vals[i] = sens[i].getValue();
      sum += vals[i];
    }
    sum /= 5;

    double offsetStd = 0;

    int offsets[5];
    for (int i = 0; i < 5; i++)
    {
      offsets[i] = vals[i] - sum;
      offsetStd += pow(offsets[i], 2);
    }

    offsetStd = sqrt(offsetStd); // compute standard deviation

    if (printLevel > 2)
    {
      Serial.printf("OFF|Right:%04i,FrontRight:%04i,Front:%04i,FrontLeft:%04i,Left:%04i,AVG:%04i,std:%f\n", offsets[0], offsets[1], offsets[2], offsets[3], offsets[4], sum, offsetStd);
    }

    if (offsetStd < 200) // if the deviation is smaller than a certain amount they are all similar values
    {
      if (printLevel > 1) {
        Serial.println("No or all walls");
      }
      // either all detecting a wall (unlikely) or all not detecting a wall
      // use the absolute average value to determine if they're all seeing a wall or not
    }
    else
    {
      if (printLevel > 1) {
      Serial.print("Found:");
      }
      for (int i = 0; i < 5; i++)
      {
        if (i == 1 || i == 3 || i == 2){
          if (offsets[i] < 100) // lower threshold for front edges 
          {
            sens[i].setState(noWall);
          }
          else
          {
            sens[i].setState(Wall);
            if (printLevel > 1) {
            Serial.print(sens[i].print() + ",");}
          }   
        } else {
          if (offsets[i] < 200)
          {
            sens[i].setState(noWall);
          }
          else
          {
            sens[i].setState(Wall);
            if (printLevel > 1) {
            Serial.print(sens[i].print() + ",");}
          }   
        }
        }
    if (printLevel > 1) {
      Serial.println("");}
    }
  }

  void update()
  {
    LDRoffsets();
    unsigned int now = millis();

    LDRstate R, FR, F, FL, L;
    R = sens[0].getState();
    FR = sens[1].getState();
    F = sens[2].getState();
    FL = sens[3].getState();
    L = sens[4].getState();

    if (!makeNode && pathFollow) {
      if (F == Wall && (FR == Wall || FL == Wall)) // front wall close
      {
        motor->stop();
        Serial.println("Stopping, about to hit wall!");
      }
      else if (R == Wall && L == Wall) // left and right in range
      {
        prevPathTime = now;
        motor->moveFW();
        if (FR == Wall) motor->turnByAngle(-10); // turn left a bit
        else if (FL == Wall) motor->turnByAngle(10); // turn left a bit

        closeRight = FR == Wall && FL != Wall;
        closeLeft = FL == Wall && FR != Wall;
      }
      else if (R == Wall && FL == Wall) // right or front-left
      {
        prevPathTime = now;
        if (closeLeft) motor->moveTurnRight();
        else motor->moveTurnLeft();
      }
      else if (L== Wall && FR == Wall) // left or front-right
      {
        prevPathTime = now;
        if (closeRight) motor->moveTurnLeft();
        else motor->moveTurnRight();
      }
      else if (now - prevPathTime < 1000) {
        if (L == Wall) {
          motor->moveTurnRight();
        } else if (R == Wall) {
          motor->moveTurnLeft();
        } else {
          // motor->stop();
          LDRoffsets();
          LDRstate StartR, StartL;
          R = sens[0].getState();
          FR = sens[1].getState();
          F = sens[2].getState();
          FL = sens[3].getState();
          L = sens[4].getState();
          int count = 0;
          while(!(R == Wall && L == Wall) && count++ < 40){
            motor->moveByDist(1);
            LDRoffsets();
            R = sens[0].getState();
            FR = sens[1].getState();
            F = sens[2].getState();
            FL = sens[3].getState();
            L = sens[4].getState();

            if (FR == Wall) motor->turnByAngle(-10); // turn left a bit
            else if (FL == Wall) motor->turnByAngle(10); // turn left a bit
          }
          Serial.println("Don't know what to do!");
        }
      } else {
        Serial.println("Path timeout expired, reached node!");
        motor->stop();
        
        makeNode = true;
        pathFollow = false;
      }
    } else if (makeNode) {

      motor->moveByDist(1);
      LDRoffsets();
      LDRstate StartR, StartL;
      R = sens[0].getState();
      FR = sens[1].getState();
      F = sens[2].getState();
      FL = sens[3].getState();
      L = sens[4].getState();

      StartR = R;
      StartL = L;
      int count = 1;
      while(F == noWall && StartR == R && StartL == L && count < 40){
        motor->moveByDist(1);
        LDRoffsets();
        R = sens[0].getState();
        FR = sens[1].getState();
        F = sens[2].getState();
        FL = sens[3].getState();
        L = sens[4].getState();
        count++;
      }

      // find out any updates on the adjacent walls
      motor->moveByDist(-count / 2);

      if (F == Wall){
        if (L == Wall && R == Wall) {
          Serial.println("Dead end");
          motor->turnByAngle(180);
          pathFollow = true;
        }
        else if (L == Wall && R == noWall) {
          Serial.println("Turn right");
          motor->turnByAngle(90);
          pathFollow = true;
        }
        else if (L == noWall && R == Wall) {
          Serial.println("Turn Left");
          motor->turnByAngle(-90);
          pathFollow = true;
        }
        else Serial.println("T junction, ask server");
      } else {        
        if (L == Wall && R == Wall) Serial.println("Not possible, straight path");
        else if (L == Wall && R == noWall) {
          Serial.println("T junction, either straight or right");
          motor->turnByAngle(90);

          pathFollow = true;
        }
        else if (L == noWall && R == Wall) {
          Serial.println("T junction, either striaght or left");
          motor->moveByDist(10);

          pathFollow = true;
        }
        else Serial.println("intersection, ask server");
      }

      if (pathFollow) {
        LDRoffsets();
        LDRstate StartR, StartL;
        R = sens[0].getState();
        FR = sens[1].getState();
        F = sens[2].getState();
        FL = sens[3].getState();
        L = sens[4].getState();
        int count = 0;
        while(!(R == Wall && L == Wall) && count++ < 40){
          motor->moveByDist(1);
          LDRoffsets();
          R = sens[0].getState();
          FR = sens[1].getState();
          F = sens[2].getState();
          FL = sens[3].getState();
          L = sens[4].getState();

          if (FR == Wall) motor->turnByAngle(-10); // turn left a bit
          else if (FL == Wall) motor->turnByAngle(10); // turn left a bit
        }
      }
      makeNode = false;
    }
  }

  void setPrintLevel(int _val)
  {
    printLevel = _val;
  }
};
