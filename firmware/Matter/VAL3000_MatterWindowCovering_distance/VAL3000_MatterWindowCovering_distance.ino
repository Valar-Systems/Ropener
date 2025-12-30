// Requires esp32 v3.3.5

// Matter Manager
#include <Matter.h>
// #if !CONFIG_ENABLE_CHIPOBLE
// // if the device can be commissioned using BLE, WiFi is not used - save flash space
// #include <WiFi.h>
// #endif
#include <Preferences.h>
#include <Button.h>

#include "memory.h"

MatterWindowCovering WindowBlinds;

#include "motor_control.h"

#include <ArduinoOTA.h>  // For enabling over-the-air updates


bool set_distance;


// Button callbacks

// Button1 = Top left button
// Button2 = Top Right button
// Button3 = Bottom button

// Stop movement, if moving, when button down is pressed
static void btn1PressDownCb(void *button_handle, void *usr_data) {
  Serial.println("Button pressed down");
  if (is_moving) {
    stop_flag = true;
    if (set_distance) {
      disable_driver();
      driver.VACTUAL(STOP_MOTOR_VELOCITY);
      is_moving = false;
      pressdown = false;
      pressdown_timer = millis() + 1000;  //start timer
      motor_position = 0;
      preferences.putInt("motor_pos", motor_position);
      WindowBlinds.setTargetLiftPercent100ths(motor_position * 100);
      set_distance = false;
    }
  }
}

// Move to full close position
static void btn1SingleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button1 single click");
  if (pressdown) {
    if (is_moving) {
      stop_flag = true;
    } else {
      fullClose();
    }
  }
}

// Move until stop button is pressed
static void btn1DoubleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button1 double click");
  // Create function for this?
  // Set velocity full
  //
  is_moving = true;
  is_closing = true;
  set_distance = true;
  enable_driver();
  driver.VACTUAL(CLOSE_VELOCITY);
}

// If not moving, sets zero position
static void btn1LongPressStartCb(void *button_handle, void *usr_data) {
  Serial.println("Button1 long press click");

  if (!is_moving) {
  }
  // Override position. Turn motor until button is pressed. (Set to close position once button is pressed. If this is the close direction?)
}










static void btn2PressDownCb(void *button_handle, void *usr_data) {
  Serial.println("Button2 pressed down");
  if (is_moving) {
    stop_flag = true;
    if (set_distance) {
      disable_driver();
      driver.VACTUAL(STOP_MOTOR_VELOCITY);
      is_moving = false;
      maximum_motor_position = motor_position;
      preferences.putInt("motor_pos", motor_position);
      preferences.putInt("max_motor_pos", motor_position);
      set_distance = false;

      pressdown = false;
      pressdown_timer = millis() + 1000;  //start timer to ignore release for 1 second
      int targetLiftPercent = 100;
      WindowBlinds.setTargetLiftPercent100ths(targetLiftPercent * 100);
    }
  }
}

// Move to full Open position
static void btn2SingleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button2 single click");
  if (pressdown) {
    if (is_moving) {
      stop_flag = true;
    } else {
      fullOpen();
    }
  }
}




static void btn2DoubleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button2 double click");
  // Move the motor until pressed to stop. Will override position

  // Pass bool to fulOpen to avoid
  // fullOpen(false);

  is_moving = true;
  set_distance = true;
  enable_driver();
  driver.VACTUAL(OPEN_VELOCITY);
}




static void btn2LongPressStartCb(void *button_handle, void *usr_data) {
  Serial.println("Button2 long press click");
}












static void btn3SingleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button2 single click");
}

// Changes the opening direction of Button1 and Button2
static void btn3DoubleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button3 double click");
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
  Serial.println("Button3 long press click");
  //Reset matter
  Serial.println("Decommissioning the Window Covering Matter Accessory. It shall be commissioned again.");
  WindowBlinds.setLiftPercentage(0);  // close the covering
  Matter.decommission();
}



void setup() {
  Serial.begin(115200);

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

  preferences.begin("preferencess", false);

  //load_preferences();

  // motor_position = preferences.getInt("motor_pos", 0);  // Loads saved motor position

  // travel_distance = 20;  // Inches. Change this value to change the distance to openS

  // // 200 steps per revolution

  // int revolutions = travel_distance / circumference_in;  // How many times the motor need to spin to reach 20 inches
  // int steps_per_revolution = 200;
  // maximum_motor_position = revolutions * steps_per_revolution;  //
  // uint8_t lastLiftPercent = ((float)motor_position / (float)maximum_motor_position) * 100;

  setup_motors();

  ArduinoOTA.begin();

  // Initialize Matter EndPoint
  // matterPref.begin("MatterPrefs", false);
  // default lift percentage is 100% (fully open) if not stored before
  uint8_t lastLiftPercent = preferences.getUChar(liftPercentPrefKey, 100);

  // Initialize window covering with BLIND_LIFT type
  WindowBlinds.begin(lastLiftPercent, MatterWindowCovering::DRAPERY);

  // Configure installed limits for lift
  WindowBlinds.setInstalledOpenLimitLift(MIN_LIFT);
  WindowBlinds.setInstalledClosedLimitLift(MAX_LIFT);

  // Initialize current positions based on percentages and installed limits
  uint16_t openLimitLift = WindowBlinds.getInstalledOpenLimitLift();
  uint16_t closedLimitLift = WindowBlinds.getInstalledClosedLimitLift();
  currentLiftPercent = lastLiftPercent;
  if (openLimitLift < closedLimitLift) {
    currentLift = openLimitLift + ((closedLimitLift - openLimitLift) * lastLiftPercent) / 100;
  } else {
    currentLift = openLimitLift - ((openLimitLift - closedLimitLift) * lastLiftPercent) / 100;
  }

  Serial.printf(
    "Window Covering limits configured: Lift [%d-%d cm]\r\n", WindowBlinds.getInstalledOpenLimitLift(),
    WindowBlinds.getInstalledClosedLimitLift());
  Serial.printf("Initial positions: Lift=%d cm (%d%%)\r\n", currentLift, currentLiftPercent);

  // Set callback functions
  WindowBlinds.onOpen(fullOpen);
  WindowBlinds.onClose(fullClose);
  WindowBlinds.onGoToLiftPercentage(goToLiftPercentage);
  WindowBlinds.onStop(stopMotor);

  // Generic callback for Lift change
  WindowBlinds.onChange([](uint8_t liftPercent, uint8_t tiltPercent) {
    Serial.printf("Window Covering changed: Lift=%d%%, Tilt=%d%%\r\n", liftPercent, tiltPercent);
    //visualizeWindowBlinds(liftPercent, tiltPercent);
    return true;
  });

  // Matter beginning - Last step, after all EndPoints are initialized
  Matter.begin();
  // This may be a restart of a already commissioned Matter accessory
  if (Matter.isDeviceCommissioned()) {
    Serial.println("Matter Node is commissioned and connected to the network. Ready for use.");
    Serial.printf("Initial state: Lift=%d%%\r\n", WindowBlinds.getLiftPercentage());
    // Update visualization based on initial state
  }
}



void loop() {
  ArduinoOTA.handle();  // Handles a code update request

  // Keeps the down button from triggering single click on release
  if (millis() >= pressdown_timer) {
    pressdown = true;
  }

  // Check Matter Window Covering Commissioning state, which may change during execution of loop()
  if (!Matter.isDeviceCommissioned()) {
    Serial.println("");
    Serial.println("Matter Node is not commissioned yet.");
    Serial.println("Initiate the device discovery in your Matter environment.");
    Serial.println("Commission it to your Matter hub with the manual pairing code or QR code");
    Serial.printf("Manual pairing code: %s\r\n", Matter.getManualPairingCode().c_str());
    Serial.printf("QR code URL: %s\r\n", Matter.getOnboardingQRCodeUrl().c_str());
    // waits for Matter Window Covering Commissioning.
    uint32_t timeCount = 0;
    while (!Matter.isDeviceCommissioned()) {
      delay(100);
      if ((timeCount++ % 50) == 0) {  // 50*100ms = 5 sec
        Serial.println("Matter Node not commissioned yet. Waiting for commissioning.");
      }
    }
    Serial.printf("Initial state: Lift=%d%%\r\n", WindowBlinds.getLiftPercentage());
    // Update visualization based on initial state

    Serial.println("Matter Node is commissioned and connected to the network. Ready for use.");
  }
}
