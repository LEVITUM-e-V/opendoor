#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <cstdint>
#include "WiFiType.h"
#include "door_actuator.h"
#include "config.h"

void mqtt_callback(char *topic, uint8_t *payload, unsigned int length);

WiFiClient wifi;
PubSubClient mqtt(MQTT_SERVER, MQTT_PORT, mqtt_callback, wifi);
DoorActuator door(&Serial2, 10);

#pragma GCC diagnostic ignored "-Wunused-function"
static void IRAM_ATTR stall_guard() {
  door.notify_stalled();
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  WiFi.begin(WIFI_SSID, WIFI_PSK);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }

  Serial2.begin(115200); // HW UART drivers
  pinMode(STALL_PIN, INPUT);
  door.setup();
  attachInterrupt(digitalPinToInterrupt(STALL_PIN), stall_guard, RISING);
}

void loop() {

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }


  while (!mqtt.connected()) {
    if (mqtt.connect("door", MQTT_USER, MQTT_PSK, "door/status", 1, true, "offline")) {
      mqtt.subscribe("door/command");
      mqtt.publish("door/status", "online", true);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }

  mqtt.loop();

}

void  mqtt_callback(char *topic, uint8_t *payload, unsigned int length) {
  Serial.println("mqtt callback");
  if (strcmp(topic, "door/command") == 0) {
    String command;
  
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      command += (char)payload[i];
    }
    Serial.println();
    if (strncmp((char*) payload, "open", length) == 0) {
      Serial.println("opening door");
      door.open();
    } else if (strncmp((char*) payload, "close", length) == 0) {
      Serial.println("closing door");
      door.close();
    } else if (strncmp((char*) payload, "home", length) == 0) {
      Serial.println("homing door");
      door.home();
    } else {
      Serial.println("command unknown");
    }
  }
}
