#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <cstdint>
#include <cstring>
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

  if (len < 1) {
    return;
  }

  char* payload = reinterpret_cast<char*>(data);

  if (payload[len-1] != '\0') {
    payload[len-1] = '\0';
  }
  //remove newline
  char *pch = strstr(payload, "\n");
  if(pch != NULL)
    strncpy(pch, "\0", 1);
  
  if (strncmp(payload, "open", len) == 0) {
    client->write("opening door...");
    Serial.println("opening door");
    door.open();
  } else if (strncmp(payload, "close", len) == 0) {
    client->write("closing door...");
    Serial.println("closing door");
    door.close();
  } else if (strncmp(payload, "home", len) == 0) {
    client->write("homing door...");
    Serial.println("homing door");
    door.home();
  } else if (strncmp(payload, "status", len) == 0) {
    std::optional<DoorPosition> pos = door.get_position();
    client->write("position: ");
    Serial.print("position: ");

    if (!pos.has_value()) {
      client->write("unknown\n");
      Serial.println("unknown");
    } else {
      const char* name = position_name(pos.value());
      client->write(name);
      client->write("\n");
      Serial.println(name);
    }

  } else {
    client->write("unknown command\n");
    Serial.println("unknown command");
  }
  client->write("done\n");
  client->send();
}

void handleClient(void *arg, AsyncClient* client) {
  Serial.println("got client");
  client->write("hey you. I'm a door\n");
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
