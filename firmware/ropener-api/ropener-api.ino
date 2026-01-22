// Libraries required:
// esp32 core by Espressiff Systems: v3.3.5
// ESPUI: 2.2.4
// TMCStepper: 0.7.3
// ESPAsyncWebServer: v3.9.4 https://github.com/ESP32Async/ESPAsyncWebServer
// AsyncTCP: v3.4.10 https://github.com/ESP32Async/AsyncTCP


#include <Arduino.h>
#include "memory.h"
#include "motor_control.h"
#include "api_settings.h"
#include "espui_settings.h"
#include "reset_button.h"
#include <ArduinoOTA.h>  // For enabling over-the-air updates


void setup() {

  Serial.begin(115200);
  Serial.println("BEGIN SETUP");

  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // ESP32 can use any pins to Serial

  delay(100);

  preferences.begin("local", false);
  load_preferences();
  setup_motors();
  setup_wifi();
  ESPUIsetup();

  ArduinoOTA.begin();

  Serial.println("COMPLETE SETUP");
}

void loop() {
  dnsServer.processNextRequest();  // Process request for ESPUI
  ArduinoOTA.handle();             // Handles a code update request
  wifiResetButton();

  if (btn1Press == true & btn2Press == true) {

    Serial.println("change direction of motor");

  } else if (btn1Press == true) {

    if (is_moving) {
      stop_flag = true;
      delay(1000);  //delay to not trigger movement again
    } else {
      // add debounce timer
      delay(500);  //Lazy debounce
      Serial.println("START MOTOR CLOSE");
      move_to_percent100ths(0);
    }
    btn1Press = false;
  } else if (btn2Press == true) {
    if (is_moving) {
      stop_flag = true;
      delay(1000);  //delay to not trigger movement again
    } else {
      // add debounce timer
      delay(500);  //Lazy debounce
      Serial.println("START MOTOR CLOSE");
      move_to_percent100ths(100);
    }
    btn2Press = false;
  } else {
    vTaskDelay(10);
  }
}