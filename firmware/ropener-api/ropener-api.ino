/* TO DO
*
* long press main buttons to override positions. Move until button pressed 
* Press both buttons same time to set max position
*
* short press btn3 to change direction
* long press btn3 to reset wifi
*/


#include <Arduino.h>
//#include "button_settings.h"
#include <Button.h>



#include "memory.h"
#include "motor_control.h"
#include "api_settings.h"
#include "espui_settings.h"

#include <ArduinoOTA.h>  // For enabling over-the-air updates


static void btn1PressDownCb(void *button_handle, void *usr_data) {
  Serial.println("Button pressed down");
  if (is_moving) {
    stop_flag = true;
  }
}
static void btn1SingleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button single click");
  if (is_moving) {
    stop_flag = true;
  } else {
    move_to_percent100ths(0);
  }
}
static void btn1DoubleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button double click");
  //
}
static void btn1LongPressStartCb(void *button_handle, void *usr_data) {
  Serial.println("Button long press click");
  // Override position. Turn motor until button is pressed. (Set to close position once button is pressed. If this is the close direction?)
}

static void btn2PressDownCb(void *button_handle, void *usr_data) {
  Serial.println("Button pressed down");
  if (is_moving) {
    stop_flag = true;
  }
}
static void btn2SingleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button single click");
  if (is_moving) {
    stop_flag = true;
  } else {
    move_to_percent100ths(100);
  }
}
static void btn2DoubleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button double click");
}
static void btn2LongPressStartCb(void *button_handle, void *usr_data) {
  Serial.println("Button long press click");
}


static void btn3SingleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button single click");
  // Set to close position
  motor_position = maximum_motor_position;
  target_percent = 100;
  ESPUI.updateSlider(positionSlider, target_percent);
  Serial.print("set close position: ");
  Serial.println(target_percent);
}
static void btn3DoubleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button double click");
  // Change direction
  if (opening_direction == 0) {
    opening_direction = 1;
    preferences.putInt("open_dir", opening_direction);
    driver.shaft(true);
  } else {
    Serial.print("Inactive");
    opening_direction = 0;
    preferences.putInt("open_dir", opening_direction);
    driver.shaft(false);
  }
}
static void btn3LongPressStartCb(void *button_handle, void *usr_data) {
  Serial.println("Button long press click");
  //Reset wifi
  preferences.putString("ssid", "NOT_SET");
  preferences.putString("pass", "NOT_SET");
  preferences.end();
  ESP.restart();
}


void setup() {

  Serial.begin(115200);
  Serial.println("BEGIN SETUP");

  Button btn1 = Button(BUTTON_1_PIN, false);    //BUTTON_1_PIN
  Button btn2 = Button(BUTTON_2_PIN, false);    //BUTTON_1_PIN
  Button btn3 = Button(WIFI_RESET_PIN, false);  //BUTTON_1_PIN

  btn1.attachPressDownEventCb(&btn1PressDownCb, NULL);
  btn1.attachSingleClickEventCb(&btn1SingleClickCb, NULL);
  btn1.attachDoubleClickEventCb(&btn1DoubleClickCb, NULL);
  btn1.attachLongPressStartEventCb(&btn1LongPressStartCb, NULL);

  btn2.attachPressDownEventCb(&btn2PressDownCb, NULL);
  btn2.attachSingleClickEventCb(&btn2SingleClickCb, NULL);
  btn2.attachDoubleClickEventCb(&btn2DoubleClickCb, NULL);
  btn2.attachLongPressStartEventCb(&btn2LongPressStartCb, NULL);

  btn3.attachSingleClickEventCb(&btn3SingleClickCb, NULL);
  btn3.attachDoubleClickEventCb(&btn3DoubleClickCb, NULL);
  btn3.attachLongPressStartEventCb(&btn3LongPressStartCb, NULL);

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
  delay(10);
}