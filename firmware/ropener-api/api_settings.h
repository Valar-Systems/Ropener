#include <WiFi.h>
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include "AsyncJson.h"
#include "ArduinoJson.h"

#include <ESPmDNS.h>

#define HOSTNAME "val3000"

AsyncWebServer serverAPI(8080);

void connectWifiAP() {
  int connect_timeout;

  WiFi.setHostname(HOSTNAME);
  Serial.println("Connecting WiFi/AP");
  //Try to connect with stored credentials, fire up an access point if they don't work.
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.begin(ssid, password);
  connect_timeout = 28;  //7 seconds
  while (WiFi.status() != WL_CONNECTED && connect_timeout > 0) {
    delay(250);
    Serial.print(".");
    connect_timeout--;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(WiFi.localIP());
    Serial.println("Wifi started");

    if (!MDNS.begin(HOSTNAME)) {  // Sets the hostname to "VAL3000.local"
      Serial.println("Error setting up MDNS responder!");
    }

  } else {
    Serial.println("\nCreating access point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP(HOSTNAME);

    connect_timeout = 20;
    do {
      delay(250);
      Serial.print(",");
      connect_timeout--;
    } while (connect_timeout);
  }
}

void setup_wifi() {

  connectWifiAP();

  serverAPI.on("/position", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("target_percent")) {
      target_percent = request->getParam("target_percent")->value().toInt();
      Serial.print("target_percent: ");
      move_to_percent100ths(target_percent);
    }
    request->send(200, "text/html", "Great Success");
  });

  serverAPI.on("/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
    JsonDocument doc;
    doc["position"] = target_percent;
    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
  });
  serverAPI.begin();
}

void wifiResetButton() {
  if (digitalRead(WIFI_RESET_PIN) == LOW) {    // Button is pressed (LOW due to pull-up)
    delay(3000);                               // delay 3 seconds
    if (digitalRead(WIFI_RESET_PIN) == LOW) {  // Confirm button press
      Serial.println("Reset button pressed. Resetting Wi-Fi...");
      preferences.putString("ssid", "ssid");  // This replaces the stored wifi network with a random value
      preferences.putString("pass", "pass");  // This replaces the stored wifi network with a random value
      ESP.restart();
    }
  }
}
