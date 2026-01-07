uint16_t openLabel;
uint16_t closeLabel;
uint16_t labelWifi;
uint16_t wifi_ssid_text, wifi_pass_text;
uint16_t speedMax;
uint16_t accelMax;
String display_wifi;

void numberMaxStepsCall(Control* sender, int type) {
  maximum_motor_position = sender->value.toInt();
  preferences.putInt("max_motor_pos", maximum_motor_position);
  Serial.print("maximum_motor_position: ");
  Serial.println(maximum_motor_position);
}

void numberSpeedCall(Control* sender, int type) {
  max_speed = sender->value.toInt();
  preferences.putInt("max_speed", max_speed);
  //setSpeedInHz(max_speed);
  Serial.print("max_speed: ");
  Serial.println(max_speed);
}

void numberAccelCall(Control* sender, int type) {
  accel = sender->value.toInt();
  preferences.putInt("accel", accel);
  //setAcceleration(accel);
  Serial.print("accel: ");
  Serial.println(accel);
}

void numberCurrentCall(Control* sender, int type) {
  current = sender->value.toInt();
  preferences.putInt("current", current);
  Serial.print("current: ");
  Serial.println(current);
}

void numberStallCall(Control* sender, int type) {
  Serial.println(sender->value);
  stall = sender->value.toInt();
  driver.SGTHRS(stall);
  preferences.putInt("stall", stall);
  Serial.print("stall: ");
  Serial.println(stall);
}

void numberTravelCall(Control* sender, int type) {
  Serial.println(sender->value);
  travel_distance = sender->value.toInt();
  preferences.putInt("travel_dist", travel_distance);
  Serial.print("travel_distance: ");
  Serial.println(travel_distance);

  if (is_cm == true) {                                                    // convert cm to steps
    maximum_motor_position = (travel_distance / circumference_cm) * 200;  // 200 steps per revolution
    Serial.print("maximum_motor_position: ");
    Serial.println(maximum_motor_position);
    preferences.putInt("max_motor_pos", maximum_motor_position);

  } else {                                                                // convert inches to steps
    maximum_motor_position = (travel_distance / circumference_in) * 200;  // 200 steps per revolution
    Serial.print("maximum_motor_position: ");
    Serial.println(maximum_motor_position);
    preferences.putInt("max_motor_pos", maximum_motor_position);
  }
}

void sliderPosition(Control* sender, int type) {
  Serial.println(sender->value);
  target_percent = sender->value.toInt();
  target_position = (maximum_motor_position / 100) * target_percent;
  move_to_percent100ths(target_percent);
}

void buttonSetCloseCall(Control* sender, int type) {
  switch (type) {
    case B_DOWN:
      Serial.println("Button Pressed");
      motor_position = maximum_motor_position;
      target_percent = 100;

      ESPUI.updateSlider(positionSlider, target_percent);

      Serial.print("set close position: ");
      Serial.println(target_percent);
      break;
  }
}

void switchChangeDirectionCall(Control* sender, int value) {
  switch (value) {
    case S_ACTIVE:
      Serial.print("Active");
      opening_direction = 1;
      preferences.putInt("open_dir", opening_direction);
      driver.shaft(true);
      break;

    case S_INACTIVE:
      Serial.print("Inactive");
      opening_direction = 0;
      preferences.putInt("open_dir", opening_direction);
      driver.shaft(false);
      break;
  }
  Serial.print(" ");
  Serial.println(sender->id);
}

void switchUnitsCall(Control* sender, int value) {
  switch (value) {
    case S_ACTIVE:
      Serial.print("is_cm");
      is_cm = 1;
      preferences.putInt("is_cm", is_cm);
      break;

    case S_INACTIVE:
      Serial.print("!is_cm");
      is_cm = 0;
      preferences.putInt("is_cm", is_cm);
      break;
  }
  Serial.print(" ");
  Serial.println(sender->id);
}

void textNetworkCall(Control* sender, int type) {
}

void textPasswordCall(Control* sender, int type) {
}

void buttonSaveNetworkCall(Control* sender, int type) {
  if (type == B_UP) {
    Serial.println("Button Pressed");
    String ssid = ESPUI.getControl(wifi_ssid_text)->value;
    String pass = ESPUI.getControl(wifi_pass_text)->value;
    preferences.putString("ssid", ssid);
    preferences.putString("pass", pass);
    ESP.restart();
  }
}

void buttonClearNetworkCall(Control* sender, int type) {
  if (type == B_UP) {
    preferences.putString("ssid", "NOT_SET");
    preferences.putString("pass", "NOT_SET");
    preferences.end();
    ESP.restart();
  }
}

void ESPUIsetup() {

  uint16_t tab1 = ESPUI.addControl(ControlType::Tab, "Positioning", "Positioning");
  uint16_t tab2 = ESPUI.addControl(ControlType::Tab, "Settings", "Settings");
  uint16_t tab3 = ESPUI.addControl(ControlType::Tab, "WiFi", "WiFi");
  uint16_t tab4 = ESPUI.addControl(ControlType::Tab, "API", "API");

  //Tab1: Positioning
  positionSlider = ESPUI.addControl(ControlType::Slider, "Position. 100 = Closed. 0 = Opened", String(target_percent), ControlColor::Alizarin, tab1, &sliderPosition); //Slider: Move to position
  ESPUI.addControl(ControlType::Min, "", "0", ControlColor::None, positionSlider);
  ESPUI.addControl(ControlType::Max, "", "100", ControlColor::None, positionSlider);


  // //Tab2: Settings
  ESPUI.addControl(ControlType::Separator, "Home Position", "", ControlColor::Peterriver, tab2);
  //Button: Set Zero
  ESPUI.addControl(ControlType::Switcher, "Change Direction", String(opening_direction), ControlColor::Dark, tab2, &switchChangeDirectionCall);
  ESPUI.addControl(ControlType::ButtonESPUI, "Set Close Position", "Set", ControlColor::Dark, tab2, &buttonSetCloseCall);
  ESPUI.addControl(ControlType::Separator, "Motor Setting", "", ControlColor::Peterriver, tab2);

  Serial.println("travel_distance: ");
  Serial.println(travel_distance);
  ESPUI.addControl(ControlType::Number, "Max Distance:", String(travel_distance), ControlColor::Peterriver, tab2, &numberTravelCall);
  ESPUI.addControl(ControlType::Switcher, "Inches/Centimeters", String(is_cm), ControlColor::Dark, tab2, &switchUnitsCall);

  // //Number: Current
  // speedMax = ESPUI.addControl(ControlType::Number, "Speed", String(max_speed), ControlColor::Alizarin, tab2, &numberSpeedCall);
  // accelMax = ESPUI.addControl(ControlType::Number, "Acceleration", String(accel), ControlColor::Alizarin, tab2, &numberAccelCall);
  // uint16_t currentMax = ESPUI.addControl(ControlType::Number, "Current(mA)", String(current), ControlColor::Alizarin, tab2, &numberCurrentCall);
  // ESPUI.addControl(ControlType::Min, "", "400", ControlColor::None, currentMax);
  // ESPUI.addControl(ControlType::Max, "", "2000", ControlColor::None, currentMax);
  // uint16_t stallMax = ESPUI.addControl(ControlType::Number, "Stall", String(stall), ControlColor::Carrot, tab2, &numberStallCall);  //Number: Stall
  // ESPUI.addControl(ControlType::Min, "", "0", ControlColor::None, stallMax);
  // ESPUI.addControl(ControlType::Max, "", "255", ControlColor::None, stallMax);


  //Tab3: WiFi
  ESPUI.addControl(ControlType::Separator, "Set Wifi", "", ControlColor::None, tab3);
  wifi_ssid_text = ESPUI.addControl(ControlType::Text, "Network", String(ssid), ControlColor::Emerald, tab3, &textNetworkCall);        //Text: Network
  wifi_pass_text = ESPUI.addControl(ControlType::Text, "Password", String(password), ControlColor::Emerald, tab3, &textPasswordCall);  //Text: Password
  ESPUI.addControl(ControlType::ButtonESPUI, "Save Settings", "SAVE", ControlColor::Emerald, tab3, &buttonSaveNetworkCall);                 //Button: Save


  //Tab4: API
  String API_IP = WiFi.localIP().toString();

  String apiPosition;
  apiPosition.reserve(150);
  apiPosition = "http://";
  apiPosition += API_IP;
  apiPosition += ":8080/position?target_percent=";
  apiPosition += target_percent;
  ESPUI.addControl(ControlType::Label, "Move to Position", apiPosition, ControlColor::Turquoise, tab4);

  String apiSettings;
  apiSettings.reserve(150);
  apiSettings = "http://";
  apiSettings += API_IP;
  apiSettings += ":8080/settings";
  ESPUI.addControl(ControlType::Label, "Check Settings", apiSettings, ControlColor::Turquoise, tab4);

  ESPUI.sliderContinuous = false;
  ESPUI.begin("Valar Systems");

  ESPUI.updateSlider(positionSlider, target_percent);
}
