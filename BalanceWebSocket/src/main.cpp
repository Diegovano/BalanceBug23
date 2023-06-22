#include <Arduino.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoWebsockets.h>

#define USE_SERIAL Serial
#define USE_WIFI true
#define WEBSOCKET true

using namespace websockets;

WebsocketsClient client;

#if USE_WIFI
#define WIFI_SSID "Diego-XPS"
#define WIFI_PASSWORD "helloGitHub!"

const String SERVER_IP = "54.82.44.87";
const String HTTP_PORT = "3001";

const String motorEndPoint = "http://" + SERVER_IP + ":" + HTTP_PORT + "/api/motor";

HTTPClient http;
#endif


void setup() {
  Serial.begin(115200);

  #if USE_WIFI
  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
  }
  Serial.print("\nConnected to WiFi as ");
  Serial.println(WiFi.localIP());
  digitalWrite(LED_BUILTIN, HIGH);

  #if WEBSOCKET
  // WEBSOCKET

  Serial.println("Connecting to WebSocket");

  String URL("ws://192.168.0.141:8080/");


  while(!client.available())
  {
    client.connect(URL);
    Serial.print('.');
    delay(500);
  }
  Serial.print("\nConnected to WebSocket!");

  client.onMessage([&](WebsocketsMessage message)
  {
    Serial.print("Got Message: ");
    Serial.println(message.data());
  });

  client.send("Hiiii!");

  #endif
  #endif
}

void loop() {
  client.poll();
}