#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define WIFI_SSID "Ezra_iPhone"
#define WIFI_PASSWORD "12345678"
#define SERVER_IP "54.82.44.87"
#define HTTP_PORT "3001"

const char *serverUrl = "http://54.82.44.87:3001/beacon";
const char *triangulationUrl = "http://54.82.44.87:3001/api/flag";

const size_t bufferSize = JSON_OBJECT_SIZE(6);
const unsigned long pollingInterval = 100;

void setup()
{
  // Wifi connection:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("starting");
}

// Keeping state variable
bool isConnected = false;

void loop()
{
  // Parse values
  StaticJsonDocument<bufferSize> jsonDoc;
  jsonDoc["distance1"] = 3.5;
  jsonDoc["distance2"] = -2;
  jsonDoc["distance3"] = 8;
  jsonDoc["rotation1"] = 1;
  jsonDoc["rotation2"] = 3;
  jsonDoc["rotation3"] = 7;

  // Connection to Wifi
  if (WiFi.status() == WL_CONNECTED && !isConnected)
  {
    Serial.println("Connected");
    digitalWrite(LED_BUILTIN, HIGH);
    isConnected = true;
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(1000);
    isConnected = false;
  }

  // Build query parameters
  String queryParams;
  for (const auto &kvp : jsonDoc.as<JsonObject>())
  {
    if (!queryParams.isEmpty())
    {
      queryParams += "&";
    }
    queryParams += kvp.key().c_str();
    queryParams += "=";
    queryParams += kvp.value().as<String>();
  }

  // Append query parameters to server URL
  String requestUrl = serverUrl;
  if (!queryParams.isEmpty())
  {
    requestUrl += "?";
    requestUrl += queryParams;
  }

  WiFiClient client;
  HTTPClient http;

  // Sending beacon values to server
  http.begin(requestUrl);
  http.addHeader("Content-Type", "application/json");
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
  // Wait before sending the next request
  delay(500);

  http.begin(triangulationUrl);
  int httpResponseCodeTriangulation = http.GET();

  if (httpResponseCodeTriangulation == HTTP_CODE_OK)
  {
    String response = http.getString();
    // Serial.println(response);
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);

    if (error)
    {
      Serial.print("Error parsing JSON: ");
      Serial.println(error.c_str());
    }
    else
    {
      int flagValue = doc["flag"];
      Serial.print("Flag value: ");
      Serial.println(flagValue);
    }
  }
  else
  {
    Serial.print("Error: ");
    Serial.println(httpResponseCodeTriangulation);
  }
  http.end();
  delay(pollingInterval);
}
