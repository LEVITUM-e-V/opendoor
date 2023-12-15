#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <cstdint>
#include "WiFiType.h"
#include "door_actuator.h"
#include "config.h"


AsyncServer server(PORT);
WiFiClient wifi;
DoorActuator door(&Serial2, 10);

#pragma GCC diagnostic ignored "-Wunused-function"
static void IRAM_ATTR stall_guard() {
  door.notify_stalled();
}

void handleData(void* arg, AsyncClient* client, void* data, size_t len) {
  Serial.print("received ");
  Serial.print(len);
  Serial.println(" bytes");
  client->write(reinterpret_cast<const char*>(data), len);
  client->send();

  char* e = strstr(reinterpret_cast<char*>(data), "\n");
  size_t idx = e - (char*) data;
  
  if (strncmp((char*) data, "open", idx) == 0) {
    client->write("opening door");
    Serial.println("opening door");
    door.open();
  } else if (strncmp((char*) data, "close", idx) == 0) {
    client->write("closing door");
    Serial.println("closing door");
    door.close();
  } else if (strncmp((char*) data, "home", idx) == 0) {
    client->write("homing door");
    Serial.println("homing door");
    door.home();
  } else {
    Serial.println("unknown command");
    client->write("unknown command");
  }
  client->write("done");
  client->close();
}

void handleClient(void *arg, AsyncClient* client) {
  Serial.println("got client");
  client->write("hey\n");
  client->send();
  client->onData(handleData);
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

  server.onClient(handleClient, NULL);
  server.begin();
}

void loop() {
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}
