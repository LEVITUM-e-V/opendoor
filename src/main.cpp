#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <cstdint>
#include <cstring>
#include "WiFiType.h"
#include "door_actuator.h"
#include "config.h"
#include "esp_timer.h"


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
    auto error = door.open();
    if (error) {
      client->write(error_name(error.value()));
      client->write("\n");
    } else {
      client->write("opening door\n");
    }
  } else if (strncmp(payload, "close", len) == 0) {
    auto error = door.close();
    if (error) {
      client->write(error_name(error.value()));
      client->write("\n");
    } else {
      client->write("closing door\n");
    }
  } else if (strncmp(payload, "home", len) == 0) {
    auto error = door.home();
    if (error) {
      client->write(error_name(error.value()));
      client->write("\n");
    } else {
      client->write("homing door\n");
    }
  } else if (strncmp(payload, "status", len) == 0) {
    DoorState state = door.get_state();
    client->write("state: ");
    client->write(state_name(state));
    client->write("\n");

    auto uptime = esp_timer_get_time();
    if (uptime != 0) {
      char uptime_str[32];
      uptime /= 1000 * 1000 * 60;
      ltoa(uptime, uptime_str, 10);
      client->write("uptime: ");
      client->write(uptime_str);
      client->write(" minutes\n");
    }
  } else if (strncmp(payload, "reboot", len) == 0) {
    ESP.restart();
  } else {
    client->write("unknown command\n");
  }
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

const unsigned long WIFI_RECONNECT_INTERVAL = 30000;
unsigned long wifi_reset_millis = 0;

void loop() {

  unsigned long current_millis = millis();
  if ((WiFi.status() != WL_CONNECTED) && 
      (current_millis - wifi_reset_millis >= WIFI_RECONNECT_INTERVAL)) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    wifi_reset_millis = current_millis;
  }
}
