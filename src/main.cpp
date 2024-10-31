#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <cstdint>
#include <cstring>
#include "esp_wifi.h"
#include <iterator>
#include "WiFiType.h"
#include "door_actuator.h"
#include "config.h"
#include "esp_timer.h"
#include "esp_wifi_types.h"

const size_t INITIAL_BUFFER_SIZE = 256;

AsyncServer server(PORT);
WiFiClient wifi;
DoorActuator door(&Serial2, 20);

#pragma GCC diagnostic ignored "-Wunused-function"
static void IRAM_ATTR stall_guard() {
  door.notify_stalled();
}

void processMessage(AsyncClient* client, const String& message) {
  if (message.equals("open")) {
    auto error = door.open();
    if (error) {
      client->write(("error: " + String(error_name(error.value())) + "\n").c_str()); // Send error if any
    } else {
      client->write("opening door...\n");
    }
  } 
  else if (message.equals("close")) {
    auto error = door.close();
    if (error) {
      client->write(("error: " + String(error_name(error.value())) + "\n").c_str());
    } else {
      client->write("closing door...\n");
    }
  } 
  else if (message.equals("home")) {
    auto error = door.home();
    if (error) {
      client->write(("error: " + String(error_name(error.value())) + "\n").c_str());
    } else {
      client->write("homing door...\n");
    }
  } 
  else if (message.equals("status")) {
    DoorState state = door.get_state();
    String response = "state: " + String(state_name(state)) + "\n";

    auto uptime = esp_timer_get_time(); // Get uptime in microseconds
    if (uptime != 0) {
      uptime /= 1000 * 1000 * 60; // Convert to minutes
      response += "uptime: " + String(uptime) + " minutes\n";
    }

    size_t freeHeap = esp_get_free_heap_size();
    response += "free heap: " + String(freeHeap) + " bytes\n";

    size_t freeRam = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    response += "free RAM: " + String(freeRam) + " bytes\n";

    wl_status_t wifiStatus = WiFi.status();
    if (wifiStatus == WL_CONNECTED) {
      response += "SSID: " + String(WiFi.SSID()) + "\n";
      response += "IP address: " + WiFi.localIP().toString() + "\n";
      response += "subnet Mask: " + WiFi.subnetMask().toString() + "\n";
      response += "gateway IP: " + WiFi.gatewayIP().toString() + "\n";
      response += "DNS IP: " + WiFi.dnsIP().toString() + "\n";
      response += "BSSID: " + String(WiFi.BSSIDstr()) + "\n";
      response += "MAC Address: " + String(WiFi.macAddress()) + "\n";

      int32_t rssi = WiFi.RSSI();
      response += "RSSI: " + String(rssi) + " dBm\n";

      int32_t channel = WiFi.channel();
      response += "channel: " + String(channel) + "\n";
    }

    client->write(response.c_str()); // Send the constructed response
  } 
  else if (message.equals("reboot")) {
    client->write("rebooting...\n");
    ESP.restart(); // Restart the ESP device
  } 
  else {
    client->write("unknown command\n"); // Handle unknown commands
  }
}


struct ClientData {
  size_t bufLen = 0;
  size_t bufSize = INITIAL_BUFFER_SIZE;
  char* buffer = nullptr;
};

void handleData(void* arg, AsyncClient* client, void* data, size_t len) {
  ClientData* client_data = static_cast<ClientData*>(arg);

  if (client_data->bufLen + len >= client_data->bufSize) {
    Serial.println("message too big for buffer!");
    client->write("message too long\n");
    client->close();
    return;
  }

  memcpy(client_data->buffer + client_data->bufLen, data, len);
  client_data->bufLen += len;

  char* messageEnd;
  while ((messageEnd = strchr(client_data->buffer, '\n')) != nullptr) {
    *messageEnd = '\0';
    Serial.print("processing message: ");
    Serial.println(client_data->buffer);
    processMessage(client, String(client_data->buffer));
    size_t remainingLen = client_data->bufLen - (messageEnd - client_data->buffer + 1);
    memmove(client_data->buffer, messageEnd + 1, remainingLen);
    client_data->bufLen = remainingLen;
    Serial.print("remaining buffer length: ");
    Serial.println(remainingLen);
  }
}

void handleDisconnect(void* arg, AsyncClient* client) {
  ClientData* client_data = static_cast<ClientData*>(arg);
  free(client_data->buffer);
  free(client_data);
  delete client;
  Serial.println("client disconnected");
}

void handleClient(void *arg, AsyncClient* client) {
  Serial.println("got client");
  ClientData* client_data = static_cast<ClientData*>(malloc(sizeof(ClientData)));
  if (client_data == nullptr) {
    Serial.println("Failed to allocate memory for ClientData!");
    client->close();
    return;
  }
  client_data->bufLen = 0;
  client_data->bufSize = INITIAL_BUFFER_SIZE;
  client_data->buffer = static_cast<char*>(malloc(INITIAL_BUFFER_SIZE));
  if (client_data->buffer == nullptr) {
    Serial.println("Failed to allocate memory for buffer!");
    free(client_data);
    client->close();
    return;
  }
  Serial.println("sending HELO");
  client->write("hey you. I'm a door\n");
  client->send();
  client->onData(handleData, client_data);
  client->onDisconnect(handleDisconnect, client_data);
}

void setup() {
  Serial.begin(115200);

  delay(1000);

  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PSK);
  esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_12M);
  esp_wifi_set_ps(WIFI_PS_NONE);

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
