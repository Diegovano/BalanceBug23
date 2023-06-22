#pragma once
#include <Arduino.h>

#include <vector>

// struct to pass to task scheduler for motor data
// type is 0 for angle, 1 for distance
// value is the double variable
typedef struct {
  int type;
  double val;
} motorVals;

class WebBuffer
{
  String ServerEndpoint;
  std::vector<motorVals> buffer;
  unsigned long prevTime;
  int minTime = 100000; // us, 0.1 seconds


public:
  WebBuffer(String endpoint)
  : ServerEndpoint(endpoint)
  {}

  void add(motorVals content)
  {
    buffer.push_back(content);
  }

  void update()
  {
    unsigned long now = micros();
    unsigned long time = now - prevTime;
    prevTime = now;
    if ( time > minTime && buffer.size() > 0 )
    {
      Serial.printf("Sending buffer of size %i", buffer.size());

      xTaskCreate(
        &send,
        "Send Buffer",
        2000,
        this,
        1,
        NULL
      );
    }
  }

  // function runs in separate thread
  static void send(void* bufferInstance)
  {
    WebBuffer* buf = (WebBuffer*) bufferInstance; // dodgy casting to retrieve buffer state; 
    Serial.println("[POST] Start sending");
    HTTPClient http;
    String msgType = "";
    String JSONdata = "[";

    for (motorVals val : buf->buffer)
    {
      if (val.type == 0) msgType = "angle";
      else if (val.val == 1) msgType = "distance";
      JSONdata += "{\"type\":\"" + msgType + "\",\"value\":" + String(val.val) + "}";
    }

    JSONdata += "]";

    http.begin(buf->ServerEndpoint);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Connection","Keep-Alive");

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

enum direction 
{
  FW, BCK, L, R, S
};

class MotorController
{
  int (*getSteps)();
  void (*setSpeed)(double), (*setTurnSpeed)(double);
  int prevSteps;
  direction dir;
  
  int moveSpeed = 10, turnSpeed = 10;

  WebBuffer *buf;
public:
  MotorController(int (*_getSteps)(), void (*_setSpeed)(double),void (*_setTurnSpeed)(double), WebBuffer *_buf)
  : getSteps(_getSteps), setSpeed(_setSpeed), setTurnSpeed(_setTurnSpeed), buf(_buf)
  {}
  
  void computeDistanceTravelled()
  {
    int steps = getSteps();
    int delta_steps = steps - prevSteps;
    prevSteps = getSteps();

    motorVals payload;

    if (dir == FW || dir == BCK){
      double dist = ( double(delta_steps) / STEP_PER_REVOLUTION ) * ( 2 * PI * WHEEL_RADIUS);
      if(dir == FW) Serial.printf("FW by %.2f cm\n", dist);
      else Serial.printf("BCK by %.2f cm\n", dist);

      payload.type = 1;
      payload.val = (dir == FW ? dist : -dist);
      buf->add(payload);
      // payload = "{\"type\":\"distance\",\"value\":" + (prevD == FW ? String(dist) : String(-dist)) + "}";
    } else if (dir == L || dir == R) {
      double angle = delta_steps * (asin((2 * PI * WHEEL_RADIUS) / (WHEEL_CENTRE_OFFSET * STEP_PER_REVOLUTION)) * 180/PI);  // thanks diego
      if(dir == L) Serial.printf("Left by %.2f degrees\n", angle);
      else Serial.printf("Right by %.2f degrees\n", angle);


      payload.type = 0;
      payload.val = (dir == R ? angle : -angle);
      buf->add(payload);

      // payload = "{\"type\":\"angle\",\"value\":" + (prevD == R ? String(angle) : String(-angle)) + "}";
    } else {
      // Serial.printf("Remained still, %i steps\n", steps);
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
  LDR R, FR, F, FL, L;
  int moveSpeed, turnSpeed;
  bool turning;
  MotorController *motor;
  const int minWallThresh = 2000, maxWallThresh = 3500; 
  
public:
  MazeLogic(const int LDRpins[5], MotorController *_motor)
  {
    R = LDR(LDRpins[0]);
    FR = LDR(LDRpins[1]);
    F = LDR(LDRpins[2]);
    FL = LDR(LDRpins[3]);
    L = LDR(LDRpins[4]);
    motor = _motor;
    motor->stop();
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
    Serial.printf("Right:%i,FrontRight%i,Front%i,FrontLeft%i,Left:%i\n", R.getValue(), FR.getValue(), F.getValue(), FL.getValue(), L.getValue());
  }

  void update()
  {
    if ( R.getValue() < maxWallThresh && R.getValue() > minWallThresh )
    {
      Serial.printf("See right wall, move forward\n");
      motor->moveFW();
    } else if ( R.getValue() > maxWallThresh ) {
      Serial.printf("Missed right wall, turn right\n");
      motor->moveTurnRight();
    } else {
      Serial.printf("Too close to right wall, turn left\n");
      motor->moveTurnLeft();
    }
  }
};



