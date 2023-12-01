#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "WiFiType.h"
#include "door_actuator.h"
#include "config.h"

static void mqtt_callback(char *topic, byte *payload, unsigned int length);

WiFiClient wifi;
PubSubClient mqtt(MQTT_SERVER, MQTT_PORT, mqtt_callback, wifi);
DoorActuator door(&Serial2, 10);

#pragma GCC diagnostic ignored "-Wunused-function"
static void IRAM_ATTR stall_guard() {
  door.notify_stalled();
}


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200); // HW UART drivers
  pinMode(STALL_PIN, INPUT);
  while(!Serial2); //wait for hardware serial
  door.setup();
  attachInterrupt(digitalPinToInterrupt(STALL_PIN), stall_guard, RISING);

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

}

static void  mqtt_callback(char *topic, byte *payload, unsigned int length) {
  if (strcmp(topic, "door/command") == 0) {
    if (payload[length-1] != '\0') {
      Serial.println("command payload is not null-terminated.");
    }
    if (strcmp((char*) payload, "open") == 0) {
      door.open();
    } else if (strcmp((char*) payload, "close") == 0) {
      door.close();
    } else if (strcmp((char*) payload, "home") == 0) {
      door.home();
    } else {
      Serial.println("command unknown");
    }
  }
}
