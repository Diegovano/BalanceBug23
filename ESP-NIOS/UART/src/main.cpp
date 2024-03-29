#include <Arduino.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include <Stepper.h>

#define DEBUG 1

// WiFi defines
#define WIFI_SSID "iPhone di Luigi"
#define WIFI_PASSWORD "chungusVBD"
#define SERVER_IP "54.82.44.87"
#define HTTP_PORT "3001"

const char *serverUrl = "http://54.82.44.87:3001/beacon";
const char *triangulationUrl = "http://54.82.44.87:3001/api/flag";

HTTPClient http;

const size_t bufferSize = JSON_OBJECT_SIZE(6);

int pollServer();

// NIOS defines
#define NIOS_RESET_PIN 18
#define NIOS_UART_RX   17
#define NIOS_UART_TX   16

HardwareSerial NIOS(2);
void reset_NIOS();
void setRED(), setBLUE(), setYELLOW();
int findBeacons(int headings[3]);

// ULN2003 Motor Driver Pins
#define IN1 19
#define IN2 22
#define IN3 5
#define IN4 21

#define STEPPER_SPEED 5

const double stepsPerRevolution = 2037.8864;
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

int cameraSpin(double angle);

void setup() {
  Serial.begin(115200);
  Serial.println("ESP-32\nImperial College London EE2 Project Magenta");

  // // WiFi Setup
  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
  }
  Serial.print("Connected to WiFi as");
  Serial.println(WiFi.localIP());
  digitalWrite(LED_BUILTIN, HIGH);

  // NIOS SETUP
  pinMode(NIOS_RESET_PIN, OUTPUT);
  reset_NIOS();

  delay(100);

  NIOS.begin(115200, SERIAL_8N1, 16, 17); 
  NIOS.printf("Hello from ESP\n");
  while (!NIOS.available()) {
    delay(500);
    Serial.println("Waiting for message from NIOS");
  }
  while (NIOS.available()){
    Serial.println("Recevied from NIOS:");
    Serial.println(NIOS.readString());
  }
}

int headings[3];
bool spinCamera = 0;

void loop() {
  
  if(Serial.available()){
    Serial.readString();
    spinCamera = 1;
  }

  if(!spinCamera){
    spinCamera = pollServer() == 1;
  }

  if (spinCamera) {
    // reset_NIOS();
    
    // while (!NIOS.available()) {
    //   delay(500);
    //   Serial.print(".");
    // }
    while (NIOS.available()){
      Serial.print("Connected:");
      Serial.println(NIOS.readString());
    }

    Serial.println("Starting Resectioning!");
    while(cameraSpin(45) == -1){
      Serial.println("[main] Camera spin failed, resetting NIOS");
      reset_NIOS();
      while (!NIOS.available()) {
        delay(500);
        Serial.println(".");
      }
      while (NIOS.available()){
        Serial.println("Recevied from NIOS:");
        Serial.println(NIOS.readString());
      }
    }
    spinCamera = 0;
  }
    
  delay(500);

  // Serial.println("Finding becons");
  // int found = findBeacons(headings);
  // Serial.printf("Found beacons: %x", found);
  // delay(1000);
}

void reset_NIOS() {
  Serial.println("Resetting NIOS");
  digitalWrite(NIOS_RESET_PIN, LOW);
  delay(20);
  digitalWrite(NIOS_RESET_PIN, HIGH);
  delay(20);
  digitalWrite(NIOS_RESET_PIN, LOW);
}

void setRED(){
  // RED
  NIOS.printf("E%08x\n", 0x2D0);
  delay(50);
  NIOS.printf("G%08x\n", 0x040);
  delay(50);
  NIOS.printf("H%08x\n", 0xEE2200);
  delay(50);
  NIOS.printf("T%08x\n", 0x1d3d);
  delay(50);
  NIOS.printf("A%08x\n", 0x1);
  delay(50);
  NIOS.printf("P%08x\n", 10);
  delay(50);
}

void setBLUE(){
  // BLUE
  NIOS.printf("E%08x\n", 0x400);
  delay(50);
  NIOS.printf("G%08x\n", 0x0);
  delay(50);
  NIOS.printf("H%08x\n", 0x0000FF);
  delay(50);
  NIOS.printf("T%08x\n", 0x2db1);
  delay(50);
  NIOS.printf("A%08x\n", 0x1);
  delay(50);
  NIOS.printf("P%08x\n", 10);
  delay(50);
} 

void setYELLOW(){
  // YELLOW
  NIOS.printf("E%08x\n", 0xe80);
  delay(50);
  NIOS.printf("G%08x\n", 0x040);
  delay(50);
  NIOS.printf("H%08x\n", 0xFFFF1f);
  delay(50);
  NIOS.printf("T%08x\n", 0x1d15);
  delay(50);
  NIOS.printf("A%08x\n", 0x5);
  delay(50);
  NIOS.printf("P%08x\n", 10);
  delay(50);
}

// Turn the stepper motor by an angle
// positive is ACCW, negative is CW
void turnAngle(double angle){
  int num_steps = int(angle * stepsPerRevolution / 360);
  #if DEBUG
  Serial.printf("[Stepper Control] Turning %.2f degrees, %i steps\n", angle, num_steps);
  #endif
  myStepper.setSpeed(STEPPER_SPEED);
  myStepper.step(num_steps);
}

int hex2int(char byte) {
  if (byte >= '0' && byte <= '9') byte = byte - '0';
  else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
  else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;    

  return byte;
}

// function that polls the nios to check for the presence of beacons
// writes the pixel value to the array param
// return 3 bits where each bit represents the red, yellow blue (100) means only red was found
int findBeacons(int headings[3], bool detected[3]){
  int found = 0;
  int pixel = 0;

  for(int i = 0; i < 3; i++){
    found = found << 1; // shift left by 1
    char *colour = (char*) malloc(20);
    if (i == 0) strcpy(colour, "red");
    else if (i == 1) strcpy(colour, "yellow");
    else if (i == 2) strcpy(colour, "blue");

    if(!detected[i]){
      // check for yellow
      #if DEBUG
      Serial.printf("Searching for %s \n", colour);
      #endif  

      if (i == 0) setRED();
      else if (i == 1) setYELLOW();
      else if (i == 2) setBLUE();

      delay(300);

      // Serial.println(NIOS.readString()); // buff flush
      // NIOS.readString();
      NIOS.printf("B"); // request Beacons information
      delay(20);
      while(!NIOS.available()){delay(1);} // wait for information to come back

      if(NIOS.available()){
        char in = NIOS.read();
        // find out if the colour was present
        if (in != 'D') {
          Serial.printf("NIOS %s read failed, %c\n", colour, in);
          return -1;
        }

        in = NIOS.read(); // 0 or 1 based on if the beacons was found

        if (in == '1'){
          found = found | 1;
          #if DEBUG
          Serial.printf("%s Beacon Found ", colour);
          #endif
          in = NIOS.read();
          if (in != 'B') {
            Serial.printf("NIOS %s read failed, %c\n",colour,  in);
            return -1;
          }
          pixel = 0;
          // read pixel value
          while((in = NIOS.read()) != '\n'){
            pixel = pixel << 4 | hex2int(in);
            #if DEBUG
            Serial.printf("%i,%c", pixel, in);
            #endif
          }
          #if DEBUG
          Serial.printf(" at position %i\n", pixel);
          #endif

          headings[i] = pixel;
          detected[i] = 1;

          #if DEBUG
          // while((in = NIOS.read()) != '\n'){
          //   Serial.print(in);
          // }
          // Serial.print('\n');
          Serial.print(NIOS.readString());
          #else
          NIOS.readString(); // read to flush buffer;
          #endif
        } else {
          #if DEBUG
          Serial.printf("%s not found, ", colour);
          Serial.println(NIOS.readString());
          #else
          NIOS.readString(); // read to flush buffer;
          #endif
        }
      }
    } else {
      Serial.printf("%s already found\n", colour);
      Serial.println(NIOS.readString());
    }
    free(colour);
  }
  return found;
}

int cameraSpin(double angleStep){
  // first turn the camera -180 to then be able to turn 360 and then -180 again
  turnAngle(-180);

  // find out how many turns are going to happen given the angle step
  int num_turns = 360 / angleStep;

  double curr_angle = -180;
  int found_beacons = 0;
  
  // array that contains the pixel value of the found pixels
  // 0 is red, 1 yellow, 2 blue
  int pixels[3];
  double angles[3]; 
  bool founds[3];

  for (int i = 0; i < 3; i++){
    founds[i] = 0;
    pixels[i] = 0;
    angles[i] = 0;
  }

  // step through that many steps and measure lights
  for (int i = 0; i < num_turns; i++){

    int found = findBeacons(pixels, founds);

    if (found == -1 ){
      Serial.println("[camera Spin] Beacon finding failed");
      turnAngle(-curr_angle);
      return -1;
    }

    if (found & 0b100) {
      angles[0] = curr_angle;
      Serial.print("[camera Spin] found red ");
    }
    if (found & 0b010) {
      angles[1] = curr_angle;
      Serial.print("[camera Spin] found yellow ");
    }
    if (found & 0b001) {
      angles[2] = curr_angle;
      Serial.print("[camera Spin] found blue ");
    }

    Serial.printf("[camera Spin] Found beacons %x: R %i Y %i B %i\n", found, pixels[0], pixels[1], pixels[2]);
    Serial.printf("[camera Spin] Current angle %.2f\n", curr_angle);

    // all beacons have been found
    if (founds[0] && founds[1] && founds[2]) break;

    turnAngle(angleStep);
    curr_angle += angleStep;
  }

  for (int i = 0; i < 3; i++){
    Serial.printf("%x, %i, %.2f\n", founds[i], pixels[i], angles[i]);
  }

  // return to 0 position
  turnAngle(-curr_angle);


  char buffer[100];

  sprintf(buffer, "distanceR=%i&distanceY=%i&distanceB=%i&rotationR=%i&rotationY=%i&rotationB=%i", pixels[0], pixels[1], pixels[2], (int) angles[0], (int) angles[1], (int) angles[2]);

  String getParams = String(buffer);
  String requestUrl = serverUrl;
  requestUrl += '?';
  requestUrl += getParams;

  // Sending beacon values to server
  http.begin(requestUrl);
  // http.addHeader("Content-Type", "application/json");
  int httpResponseCodeBeacon = http.GET();

  if (httpResponseCodeBeacon > 0)
  {
    String response = http.getString();
    Serial.println("HTTP Response code: " + String(httpResponseCodeBeacon));
    Serial.println("Response: " + response);
  }
  else
  {
    Serial.println("Error sending GET request");
  }
  http.end();
  return 0;
}

int pollServer(){
  http.begin(triangulationUrl);
  int httpResponseCodeTriangulation = http.GET();

  if (httpResponseCodeTriangulation != -1)
  {
    String response = http.getString();
    // Serial.println(response);
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);

    if (error)
    {
      #if DEBUG
      Serial.print("Error parsing JSON: ");
      Serial.println(error.c_str());
      #endif
      http.end();
      return -1;
    }
    else
    {
      int flagValue = doc["flag"];
      #if DEBUG
      Serial.print("Flag value: ");
      Serial.println(flagValue);
      #endif
      http.end();
      return flagValue;
    }
  }
  else
  {
    #if DEBUG
    Serial.print("Error: ");
    Serial.println(httpResponseCodeTriangulation);
    #endif
    http.end();
    return -1;
  }  
}