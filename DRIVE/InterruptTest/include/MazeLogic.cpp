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
  inRange,
  wallClose,
  away
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
    if (state == wallClose)
      val = "close";
    else if (state == inRange)
      val = "inRange";
    else if (state == away)
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
  int prevSteps;
  direction dir;

  int moveSpeed = 5, turnSpeed = 5;

  WebBuffer *buf;

public:
  MotorController(int (*_getSteps)(), void (*_setSpeed)(double), void (*_setTurnSpeed)(double), WebBuffer *_buf)
      : getSteps(_getSteps), setSpeed(_setSpeed), setTurnSpeed(_setTurnSpeed), buf(_buf)
  {
    Serial.println("[Motor Control] Class initialised");
  }

  MotorController(int (*_getSteps)(), void (*_setSpeed)(double), void (*_setTurnSpeed)(double))
      : getSteps(_getSteps), setSpeed(_setSpeed), setTurnSpeed(_setTurnSpeed)
  {
    Serial.println("[Motor Control] Class initialised with no Buffer");
  }

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
        if (offsets[i] < 200)
        {
          sens[i].setState(away);
        }
        else if (offsets[i] > 600)
        {
          sens[i].setState(wallClose);
          if (printLevel > 1) {
          Serial.print(sens[i].print() + ",");}
        }
        else
        {
          sens[i].setState(inRange);
           if (printLevel > 1) {
          Serial.print(sens[i].print() + ",");}
        }

        
      }
if (printLevel > 1) {
      Serial.println("");}
    }
  }

  void update()
  {
    LDRoffsets();

    if (sens[2].getState() == wallClose) // front wall close
    {
      motor->stop();
    }
    else if ((sens[0].getState() == inRange && sens[4].getState() == inRange) || (sens[0].getState() == wallClose && sens[4].getState() == wallClose))
    {
      // left and right in range
      motor->moveFW();
    }
    else if (sens[0].getState() == wallClose || sens[1].getState() != away)
    {
      // right too close
      motor->moveTurnLeft();
    }
    else if (sens[4].getState() == wallClose || sens[3].getState() != away)
    {
      // left too close
      motor->moveTurnRight();
    }
  }

  void setPrintLevel(int _val)
  {
    printLevel = _val;
  }
};
