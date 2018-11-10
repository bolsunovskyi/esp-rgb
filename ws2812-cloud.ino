#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Syslog.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "FastLED.h"
#define NUM_LEDS 49
CRGB leds[NUM_LEDS];

#define WIFI_SSID "MARVEL"
#define WIFI_PASSWORD "---"
#define WIFI_HOSTNAME "wittycloud-1"

#define SYSLOG_SERVER "192.168.1.5"
#define SYSLOG_PORT 514
#define SYSLOG_DEVICE_HOSTNAME WIFI_HOSTNAME
#define SYSLOG_APP_NAME WIFI_HOSTNAME
#define OTA_PASSWORD "---"

#define MQTT_SERVER "192.168.1.5"
#define MQTT_PORT 1883

#define RED_LED 15
#define GREEN_LED 12
#define BLUE_LED 13

#define STATUS_TOPIC "shouse/tele/wittycloud-1/STATUS"
#define COMMAND_TOPIC "shouse/cmnd/wittycloud-1/COLOR"

WiFiClient espClient;
PubSubClient client(espClient);

WiFiUDP udpClient;
Syslog syslog(udpClient, SYSLOG_SERVER, SYSLOG_PORT, SYSLOG_DEVICE_HOSTNAME, SYSLOG_APP_NAME, LOG_KERN);

uint8_t cR, cG, cB, cBr;

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  WiFi.hostname(WIFI_HOSTNAME);
  ArduinoOTA.setHostname(WIFI_HOSTNAME);

  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";    
    } else { // U_SPIFFS
      type = "filesystem";
    }

    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  syslog.logf(LOG_INFO, "Device ready, ip address: %s", WiFi.localIP().toString().c_str());

  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);

  Serial.setDebugOutput(0);

  FastLED.addLeds<WS2812B, 14, GRB>(leds, NUM_LEDS);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(RED_LED, HIGH);
  delay(1000);

  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(LED_BUILTIN, HIGH);

  cR = 255;
  cG = 255;
  cB = 255;
  cBr = 255;
  FastLED.showColor(CRGB(cR, cG, cB), cBr);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char *msg = new char[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';

  syslog.logf(LOG_INFO, "mqtt message arrived, topic: %s, message: %s", topic, msg);
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(msg);
  const char* state = root["state"];
  if (strcmp(state, "ON") == 0 ) {
    if (root.containsKey("color")) {
      cR = (uint8_t)root["color"]["r"];
      cG = (uint8_t)root["color"]["g"];
      cB = (uint8_t)root["color"]["b"];
    }
    if (root.containsKey("brightness")) {
      cBr = (uint8_t)root["brightness"];
    }
    
    FastLED.showColor(CRGB(cR, cG, cB), cBr);
  } else {
    FastLED.showColor(CRGB(cR, cG, cB), 0);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(WIFI_HOSTNAME)) {
      Serial.println("connected");
      syslog.logf(LOG_INFO, "connected to mqtt");
      
      // Once connected, publish an announcement...
      client.publish(STATUS_TOPIC,"poweron");
      // ... and resubscribe
      client.subscribe(COMMAND_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  ArduinoOTA.handle();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  /*setAll(255, 0, 0); delay(500);
  setAll(0, 255, 0); delay(500);
  setAll(0, 0, 255); delay(500);
  setAll(0, 0, 0); delay(500);*/

  /*FastLED.showColor(CRGB(255, 0, 0), 255);  delay(500);
  FastLED.showColor(CRGB(0, 255, 0), 255);  delay(500);
  FastLED.showColor(CRGB(0, 0, 255), 255);  delay(500);
  FastLED.showColor(CRGB(255, 255, 255), 255);  delay(500);
  FastLED.showColor(CRGB(0, 0, 0), 255);  delay(500);*/

  /*leds[0] = CRGB::Red; FastLED.show(); delay(500);
  leds[0] = CRGB::Green; FastLED.show(); delay(500);
  leds[0] = CRGB::Blue; FastLED.show(); delay(500);
  leds[0] = CRGB::Black; FastLED.show(); delay(500);*/
  
}

