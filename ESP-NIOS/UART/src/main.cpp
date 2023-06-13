#include <Arduino.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include <Stepper.h>

#define DEBUG 0

// WiFi defines
#define WIFI_SSID "Ezra_iPhone"
#define WIFI_PASSWORD "12345678"
#define SERVER_IP "54.82.44.87"
#define HTTP_PORT "3001"

const char* serverUrl = "http://54.82.44.87:3001/beacon";

const size_t bufferSize = JSON_OBJECT_SIZE(6);

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

void cameraSpin(double angle);

void setup() {
  Serial.begin(115200);
  Serial.println("ESP-32\nImperial College London EE2 Project Magenta");

  // WiFi Setup
  // pinMode(LED_BUILTIN, OUTPUT);
  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Serial.println("Connecting to Wi-Fi");

  // while (WiFi.status() != WL_CONNECTED) {
  //   Serial.println(".");
  //   digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  //   delay(500);
  // }
  // Serial.print("Connected to WiFi as");
  // Serial.println(WiFi.localIP());
  // digitalWrite(LED_BUILTIN, HIGH);

  // NIOS SETUP
  pinMode(NIOS_RESET_PIN, OUTPUT);
  reset_NIOS();

  NIOS.begin(115200, SERIAL_8N1, 16, 17); 
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

void loop() {

  int found = findBeacons(headings);

  Serial.printf("Found %i beacons. R %i Y %i B %i\n", found, headings[0], headings[1], headings[2]);
  memset(headings, 0, sizeof(headings));
  delay(1000);
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
  NIOS.printf("E%08x\n", 0x3D0);
  delay(50);
  NIOS.printf("G%08x\n", 0x0);
  delay(50);
  NIOS.printf("H%08x\n", 0xFF0000);
  delay(50);
  NIOS.printf("T%08x\n", 0x1e3d);
  delay(50);
  NIOS.printf("A%08x\n", 0x1);
  delay(50);
  NIOS.printf("P%08x\n", 10);
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
} 

void setYELLOW(){
  // YELLOW
  NIOS.printf("E%08x\n", 0xd80);
  delay(50);
  NIOS.printf("G%08x\n", 0x0);
  delay(50);
  NIOS.printf("H%08x\n", 0xEEFF00);
  delay(50);
  NIOS.printf("T%08x\n", 0x1d15);
  delay(50);
  NIOS.printf("A%08x\n", 0x5);
  delay(50);
  NIOS.printf("P%08x\n", 20);
}

// Turn the stepper motor by an angle
// positive is ACCW, negative is CW
void turnAngle(double angle){
  int num_steps = int(angle * stepsPerRevolution / 360);
  #if DEBUG
  Serial.printf("Turning %.2f degrees, %i steps", angle, num_steps);
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
// returns the number of found beacons and writes the pixel value to the array param
int findBeacons(int headings[3]){
  int found = 0;
  int pixel = 0;


  for(int i = 0; i < 3; i++){
    char *colour = (char*) malloc(20);
    if (i == 0) strcpy(colour, "red");
    else if (i == 1) strcpy(colour, "yellow");
    else if (i == 2) strcpy(colour, "blue");

    // check for yellow
    #if DEBUG
    Serial.printf("Searching for %s\n", colour);
    #endif  

    if (i == 0) setRED();
    else if (i == 1) setYELLOW();
    else if (i == 2) setBLUE();

    delay(100);

    // Serial.println(NIOS.readString()); // buff flush
    // NIOS.readString();
    NIOS.printf("B"); // request Beacons information
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
        found++;
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

    free(colour);
  }
  return found;
}

void cameraSpin(double angleStep){
  // first turn the camera -180 to then be able to turn 360 and then -180 again
  turnAngle(-180);

  // find out how many turns are going to happen given the angle step
  int num_turns = 360 / angleStep;

  double curr_angle = -180;
  int found_beacons = 0;
  
  // array that contains the pixel value of the found pixels
  // 0 is red, 1 yellow, 2 blue
  int headings[3]; 

  // step through that many steps and measure lights
  for (int i = 0; i < num_turns; i++){

    found_beacons += findBeacons(headings);

    // all beacons have been found
    if (found_beacons == 3) break;

    turnAngle(angleStep);
    curr_angle += angleStep;
  }

  // return to 0 position
  turnAngle(-curr_angle);
}