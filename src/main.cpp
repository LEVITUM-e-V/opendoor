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
DoorActuator door(&Serial2, 20);

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

    client->write("driver version: ");
    char versionStr[10];
    itoa(door._driver.version(), versionStr, 10);
    client->write(versionStr);
    client->write("\n");

    size_t freeHeap = esp_get_free_heap_size();
    char freeHeapStr[32];
    ltoa(freeHeap, freeHeapStr, 10);
    client->write("free heap: ");
    client->write(freeHeapStr);
    client->write(" bytes\n");

    size_t freeRam = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    char freeRamStr[32];
    ltoa(freeRam, freeRamStr, 10);
    client->write("free RAM: ");
    client->write(freeRamStr);
    client->write(" bytes\n");

    wl_status_t wifiStatus = WiFi.status();

    if (wifiStatus == WL_CONNECTED) {
      client->write("SSID: ");
      client->write(WiFi.SSID().c_str());
      client->write("\n");

      client->write("IP address: ");
      client->write(WiFi.localIP().toString().c_str());
      client->write("\n");

      client->write("Subnet Mask: ");
      client->write(WiFi.subnetMask().toString().c_str());
      client->write("\n");

      client->write("Gateway IP: ");
      client->write(WiFi.gatewayIP().toString().c_str());
      client->write("\n");

      client->write("DNS IP: ");
      client->write(WiFi.dnsIP().toString().c_str());
      client->write("\n");

      client->write("BSSID: ");
      client->write(WiFi.BSSIDstr().c_str());
      client->write("\n");

      client->write("MAC Address: ");
      client->write(WiFi.macAddress().c_str());
      client->write("\n");

      int32_t rssi = WiFi.RSSI();
      char rssiStr[32];
      ltoa(rssi, rssiStr, 10);
      client->write("RSSI: ");
      client->write(rssiStr);
      client->write(" dBm\n");

      int32_t channel = WiFi.channel();
      char channelStr[32];
      ltoa(channel, channelStr, 10);
      client->write("Channel: ");
      client->write(channelStr);
      client->write("\n");
    }

  } else if (strncmp(payload, "reboot", len) == 0) {
    ESP.restart();
  } else {
    client->write("unknown command\n");
  }
  client->send();
}

void handleDisconnect(void* arg, AsyncClient* client) {
  delete client;
}

void handleClient(void *arg, AsyncClient* client) {
  Serial.println("got client");
  client->write("hey you. I'm a door\n");
  client->send();
  client->onData(handleData);
  client->onDisconnect(handleDisconnect);
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
