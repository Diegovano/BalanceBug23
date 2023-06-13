
const int LDRPin = 34;
int LDRValue = 0;



void setup() {
  // put your setup code here, to run once:
  pinMode(LDRPin, INPUT);
   Serial.begin(9600);
    delay(1000);
  
  

}

void loop() {
  // put your main code here, to run repeatedly:
  LDRValue = analogRead(LDRPin);
  Serial.println(LDRValue);

  if (LDRValue > 3570 ){
    Serial.println("You are 1cm away from a wall");
  } else if (3441 < LDRValue ){
    Serial.println("You are 2cm away from a wall");
  } else if (3295 < LDRValue ){
    Serial.println("You are 3cm away from a wall");
  } else if (3237 < LDRValue ){
    Serial.println("You are 4cm away from a wall");
  } else if (3180 < LDRValue ){
    Serial.println("You are 5cm away from a wall");
  } else if (3120 < LDRValue ){
    Serial.println("You are 6cm away from a wall");
  } else if (3089 < LDRValue ){
    Serial.println("You are 7cm away from a wall");
  } else if (3076 < LDRValue ){
    Serial.println("You are 8cm away from a wall");
  } else if (3051 < LDRValue ){
    Serial.println("You are 9cm away from a wall");
  } else if (3025 < LDRValue ){
    Serial.println("You are 10cm away from a wall");
  } else if (3011 < LDRValue ){
    Serial.println("You are 11cm away from a wall");
  } else if (2971 < LDRValue ){
    Serial.println("You are 12cm away from a wall");
  } else if (2962 < LDRValue ){
    Serial.println("You are 13cm away from a wall");
  } else if (2923 < LDRValue ){
    Serial.println("You are 14cm away from a wall");
  } else if (2909 < LDRValue ){
    Serial.println("You are 15cm away from a wall");
  } else {
    Serial.println("You are more than 15cm away from a wall");
  }
 
 
 
 
 
 
 
 
 
 
 
 
 
 
  

}