// Requires esp32 v3.3.5

// ========================================
// LOGGING CONFIGURATION
// Comment out to disable serial logging (saves memory)
// ========================================
#define LOGGING_ENABLED

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

// #include <ArduinoOTA.h>  // Disabled to save memory during commissioning

// ========================================
// LOCAL STATE VARIABLES
// ========================================

#define PRESSDOWN_DELAY 1500

// Button state management (moved from memory.h - only used in this file)
static bool pressdown = false;             // Press-down state flag
static uint32_t pressdown_timer = 0;       // Press-down timer

// Distance setting mode flag
static bool set_distance = false;          // True when setting distance via double-click

// ========================================
// MOTOR INITIALIZATION HELPER
// ========================================

/**
 * @brief Initialize motor system (called after commissioning)
 *
 * This function is called automatically when the device is commissioned.
 * It sets up the motor driver and creates the position watcher task.
 */
void initialize_motor_system() {
  if (motor_initialized) {
    return;  // Already initialized
  }

  #ifdef LOGGING_ENABLED
    Serial.println("Initializing motor system...");
  #endif

  setup_motors();
  xTaskCreate(position_watcher_task, "position_watcher_task", 4192, NULL, 1, &position_watcher_task_handler);
  motor_initialized = true;
  WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::STALL);

  #ifdef LOGGING_ENABLED
    Serial.println("Motor system initialized successfully.");
    Serial.println("Device is now fully operational.");
  #endif
}


// Button callbacks

// Button1 = Top left button
// Button2 = Top Right button
// Button3 = Bottom button

// Stop movement, if moving, when button down is pressed
static void btn1PressDownCb(void *button_handle, void *usr_data) {
  #ifdef LOGGING_ENABLED
    Serial.println("Button pressed down");
  #endif
  if (is_moving) {
    #ifdef LOGGING_ENABLED
      Serial.println("Stop Flag");
    #endif
    stop_flag = true;

    pressdown = false;
    pressdown_timer = millis() + PRESSDOWN_DELAY;  //start timer to ignore release for 1 second

    if (set_distance) {
      disable_driver();
      driver.VACTUAL(STOP_MOTOR_VELOCITY);

      motor_position = 0;
      preferences.putInt(PREF_MOTOR_POS, motor_position);

      currentLiftPercent = 0;
      WindowBlinds.setLiftPercentage(currentLiftPercent);

      set_distance = false;

      #ifdef LOGGING_ENABLED
        Serial.print("Motor position: ");
        Serial.println(motor_position);
      #endif
    }
    is_moving = false;  // Is this needed?
  }
}

// Move to full close position
static void btn1SingleClickCb(void *button_handle, void *usr_data) {
  #ifdef LOGGING_ENABLED
    Serial.println("Button1 single click");
    Serial.print("motor_position: ");
    Serial.println(motor_position);
  #endif

  // Safety check: Don't operate motors if not initialized
  if (!motor_initialized) {
    #ifdef LOGGING_ENABLED
      Serial.println("Motors not initialized yet. Please commission device first.");
    #endif
    return;
  }

  if (pressdown) {
    #ifdef LOGGING_ENABLED
      Serial.println("pressdown");
    #endif
    if (is_moving) {
      stop_flag = true;
    } else {
      fullClose();
    }
  }
}

// Move until stop button is pressed
static void btn1DoubleClickCb(void *button_handle, void *usr_data) {
  #ifdef LOGGING_ENABLED
    Serial.println("Button1 double click");
  #endif

  // Safety check: Don't operate motors if not initialized
  if (!motor_initialized) {
    #ifdef LOGGING_ENABLED
      Serial.println("Motors not initialized yet. Please commission device first.");
    #endif
    return;
  }

  // Create function for this?
  // Set velocity full
  //

  is_closing = true;
  set_distance = true;
  // create function for this?
  enable_driver();
  driver.VACTUAL(CLOSE_VELOCITY);
  is_moving = true;
}

// Sets zero position
static void btn1LongPressStartCb(void *button_handle, void *usr_data) {
  #ifdef LOGGING_ENABLED
    Serial.println("Button1 long press click");
  #endif

  // Safety check: Don't operate motors if not initialized
  if (!motor_initialized) {
    #ifdef LOGGING_ENABLED
      Serial.println("Motors not initialized yet. Please commission device first.");
    #endif
    return;
  }

  motor_position = 0;
  preferences.putInt(PREF_MOTOR_POS, motor_position);

  currentLiftPercent = 100;            // 0
  WindowBlinds.setLiftPercentage(99);  // Updates Matter to 100 percent closed position
  delay(100);
  WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::STALL);

  #ifdef LOGGING_ENABLED
    Serial.print("Motor position: ");
    Serial.println(motor_position);
  #endif
}


// BUTTON 2
static void btn2PressDownCb(void *button_handle, void *usr_data) {
  #ifdef LOGGING_ENABLED
    Serial.println("Button2 pressed down");
  #endif
  if (is_moving) {
    #ifdef LOGGING_ENABLED
      Serial.println("Stop Flag");
    #endif
    stop_flag = true;

    pressdown = false;
    pressdown_timer = millis() + PRESSDOWN_DELAY;  //start timer to ignore release for 1 second

    if (set_distance) {
      disable_driver();
      driver.VACTUAL(STOP_MOTOR_VELOCITY);

      maximum_motor_position = motor_position;
      preferences.putInt(PREF_MOTOR_POS, motor_position);
      preferences.putInt(PREF_MAX_MOTOR_POS, motor_position);
      #ifdef LOGGING_ENABLED
        Serial.print("Motor position: ");
        Serial.println(motor_position);
      #endif

      set_distance = false;

      // pressdown = false;
      // pressdown_timer = millis() + PRESSDOWN_DELAY;  //start timer to ignore release for 1 second

      #ifdef LOGGING_ENABLED
        Serial.println("Updating Matter");
      #endif
      currentLiftPercent = 0;                              // 0 percent open = 100 percent closed
      WindowBlinds.setLiftPercentage(currentLiftPercent);  // Updates Matter to 0 percent position
      delay(100);
      WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::STALL);

      //Convert distance to centimeters
      int revolutions;
      revolutions = motor_position / 200;  // may equal zero

      MAX_LIFT = revolutions * 3.7699;  // may equal zero
      //motor_position / 200 = motor revolutions

      // 3.7699 cm per revolution
      //WindowBlinds.setInstalledClosedLimitLift(MAX_LIFT);
    }
    is_moving = false;  // is this required
  }
}

// Move to full Open position
static void btn2SingleClickCb(void *button_handle, void *usr_data) {
  #ifdef LOGGING_ENABLED
    Serial.println("Button2 single click");
    Serial.print("Motor position: ");
    Serial.println(motor_position);
  #endif

  // Safety check: Don't operate motors if not initialized
  if (!motor_initialized) {
    #ifdef LOGGING_ENABLED
      Serial.println("Motors not initialized yet. Please commission device first.");
    #endif
    return;
  }

  if (pressdown) {  // not working
    #ifdef LOGGING_ENABLED
      Serial.println("pressdown");
    #endif
    if (is_moving) {
      stop_flag = true;
    } else {
      fullOpen();
    }
  }
}


static void btn2DoubleClickCb(void *button_handle, void *usr_data) {
  #ifdef LOGGING_ENABLED
    Serial.println("Button2 double click");
  #endif

  // Safety check: Don't operate motors if not initialized
  if (!motor_initialized) {
    #ifdef LOGGING_ENABLED
      Serial.println("Motors not initialized yet. Please commission device first.");
    #endif
    return;
  }

  // Move the motor until pressed to stop. Will override position

  // Pass bool to fulOpen to avoid
  // fullOpen(false);

  is_closing = false;
  set_distance = true;

  enable_driver();
  driver.VACTUAL(OPEN_VELOCITY);
  is_moving = true;
}


static void btn2LongPressStartCb(void *button_handle, void *usr_data) {
  #ifdef LOGGING_ENABLED
    Serial.println("Button2 long press click");
  #endif
}


static void btn3SingleClickCb(void *button_handle, void *usr_data) {
  #ifdef LOGGING_ENABLED
    Serial.println("Button3 single click");
  #endif
}

// Changes the opening direction of Button1 and Button2
static void btn3DoubleClickCb(void *button_handle, void *usr_data) {
  #ifdef LOGGING_ENABLED
    Serial.println("Button3 double click");
  #endif

  // Safety check: Don't operate motors if not initialized
  if (!motor_initialized) {
    #ifdef LOGGING_ENABLED
      Serial.println("Motors not initialized yet. Please commission device first.");
    #endif
    return;
  }

  // Change direction
  if (opening_direction == 0) {
    opening_direction = 1;
    preferences.putInt(PREF_OPEN_DIR, opening_direction);
    driver.shaft(true);
  } else {
    #ifdef LOGGING_ENABLED
      Serial.print("Inactive");
    #endif
    opening_direction = 0;
    preferences.putInt(PREF_OPEN_DIR, opening_direction);
    driver.shaft(false);
  }
}

static void btn3LongPressStartCb(void *button_handle, void *usr_data) {
  #ifdef LOGGING_ENABLED
    Serial.println("Button3 long press click");
    Serial.println("Decommissioning the Window Covering Matter Accessory. It shall be commissioned again.");
  #endif

  //Reset matter
  WindowBlinds.setLiftPercentage(0);  // close the covering
  Matter.decommission();

  // Reset motor initialization flag so motors will be re-initialized after re-commissioning
  motor_initialized = false;

  #ifdef LOGGING_ENABLED
    Serial.println("Motor system will be re-initialized after next commissioning.");
  #endif
}



void setup() {

  #ifdef LOGGING_ENABLED
    Serial.begin(115200);
  #endif

  Button btn1 = Button(BUTTON_1_PIN, false);    //BUTTON_1_PIN
  Button btn2 = Button(BUTTON_2_PIN, false);    //BUTTON_2_PIN
  Button btn3 = Button(WIFI_RESET_PIN, false);  //WIFI_RESET_PIN

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

  load_preferences();

  // motor_position = preferences.getInt("motor_pos", 0);  // Loads saved motor position

  // travel_distance = 20;  // Inches. Change this value to change the distance to openS

  // // 200 steps per revolution

  // int revolutions = travel_distance / circumference_in;  // How many times the motor need to spin to reach 20 inches
  // int steps_per_revolution = 200;
  // maximum_motor_position = revolutions * steps_per_revolution;  //
  // uint8_t lastLiftPercent = ((float)motor_position / (float)maximum_motor_position) * 100;

  // DEFERRED MOTOR INITIALIZATION - Motors will be initialized after first successful commissioning
  // This prevents motor initialization from interfering with Matter commissioning
  // setup_motors();
  // xTaskCreate(position_watcher_task, "position_watcher_task", 4192, NULL, 1, &position_watcher_task_handler);

  #ifdef LOGGING_ENABLED
    Serial.println("Check 1");                                  // Print the free heap memory in bytes
    Serial.println(ESP.getFreeHeap());                          // Print the free heap memory in bytes
    UBaseType_t freeStack = uxTaskGetStackHighWaterMark(NULL);  // Periodically check and print the stack high water mark (minimum free stack)
    Serial.printf("Loop task high water mark (min free stack): %u bytes\n", freeStack);
  #endif

  // Initialize Matter EndPoint
  // default lift percentage is 100% (fully closed) if not stored before
  uint8_t lastLiftPercent = preferences.getUChar(PREF_LIFT_PERCENT, 100);

  // Initialize window covering with BLIND_LIFT type
  WindowBlinds.begin(lastLiftPercent, MatterWindowCovering::DRAPERY);

  // Configure installed limits for lift
  WindowBlinds.setInstalledOpenLimitLift(MIN_LIFT);
  WindowBlinds.setInstalledClosedLimitLift(MAX_LIFT);

  // Set current lift percentage
  currentLiftPercent = lastLiftPercent;

  #ifdef LOGGING_ENABLED
    Serial.printf(
      "Window Covering limits configured: Lift [%d-%d cm]\r\n",
      WindowBlinds.getInstalledOpenLimitLift(),
      WindowBlinds.getInstalledClosedLimitLift());
    Serial.printf("Initial lift percentage: %d%%\r\n", currentLiftPercent);
  #endif

  // Set callback functions
  WindowBlinds.onOpen(fullOpen);
  WindowBlinds.onClose(fullClose);
  WindowBlinds.onGoToLiftPercentage(goToLiftPercentage);
  WindowBlinds.onStop(stopMotor);

  // Generic callback for Lift change
  // This callback also fires when device first connects after commissioning
  WindowBlinds.onChange([](uint8_t liftPercent, uint8_t tiltPercent) {
    #ifdef LOGGING_ENABLED
      Serial.printf("Window Covering changed: Lift=%d%%, Tilt=%d%%\r\n", liftPercent, tiltPercent);
    #endif

    // Auto-initialize motors when device becomes commissioned
    // This callback fires when the device first connects to the Matter network
    if (!motor_initialized && Matter.isDeviceCommissioned()) {
      initialize_motor_system();
    }

    return true;
  });

  // Matter beginning - Last step, after all EndPoints are initialized
  Matter.begin();

  // Check if device is already commissioned (e.g., after a restart)
  if (Matter.isDeviceCommissioned()) {
    #ifdef LOGGING_ENABLED
      Serial.println("Matter Node is commissioned and connected to the network. Ready for use.");
      Serial.printf("Initial state: Lift=%d%%\r\n", WindowBlinds.getLiftPercentage());
    #endif

    // Device is already commissioned, initialize motors immediately
    initialize_motor_system();
  }
  else {
    #ifdef LOGGING_ENABLED
      Serial.println("Matter Node is not commissioned yet.");
      Serial.println("Motor system will be initialized automatically after commissioning.");
      Serial.printf("Manual pairing code: %s\r\n", Matter.getManualPairingCode().c_str());
      Serial.printf("QR code URL: %s\r\n", Matter.getOnboardingQRCodeUrl().c_str());
    #endif
  }
}


unsigned long previousMillis = 0;
const long interval = 3000;  // 3 second

unsigned long previousMillis2 = 0;
const long interval2 = 10000;  // 2 second

void loop() {
  //ArduinoOTA.handle();  // Handles a code update request

  // Motor initialization is now handled automatically via:
  // 1. setup() - if device is already commissioned
  // 2. WindowBlinds.onChange() callback - when device first connects after commissioning
  // No polling needed!

  // Keeps the down button from triggering single click on release
  if (millis() >= pressdown_timer) {
    pressdown = true;
  }

  #ifdef LOGGING_ENABLED
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;                             // Save the time of the last event
      Serial.println(ESP.getFreeHeap());                          // Print the free heap memory in bytes
      UBaseType_t freeStack = uxTaskGetStackHighWaterMark(NULL);  // Periodically check and print the stack high water mark (minimum free stack)
      Serial.printf("Loop task high water mark (min free stack): %u bytes\n", freeStack);
    }
  #endif




  // Serial.print("is_moving: ");
  // Serial.println(is_moving);
  // delay(100);


  #ifdef LOGGING_ENABLED
    unsigned long currentMillis2 = millis();
    if (currentMillis2 - previousMillis2 >= interval2) {
      previousMillis2 = currentMillis2;  // Save the time of the last event

      if (!Matter.isDeviceCommissioned()) {
        Serial.println("Matter Node is not commissioned yet.");
        //Serial.printf("Manual pairing code: %s\r\n", Matter.getManualPairingCode().c_str());
        //Serial.printf("QR code URL: %s\r\n", Matter.getOnboardingQRCodeUrl().c_str());
      }
      else
      {
        Serial.println("Matter Node is commissioned and connected to the network. Ready for use.");
        if (!motor_initialized && Matter.isDeviceCommissioned())
        {
          initialize_motor_system();
        }
      }
    }
  #endif

  // Check Matter Window Covering Commissioning state, which may change during execution of loop()

  //   Serial.println("Initiate the device discovery in your Matter environment.");
  //   Serial.println("Commission it to your Matter hub with the manual pairing code or QR code");
  //   Serial.printf("Manual pairing code: %s\r\n", Matter.getManualPairingCode().c_str());
  //   Serial.printf("QR code URL: %s\r\n", Matter.getOnboardingQRCodeUrl().c_str());
  //   // waits for Matter Window Covering Commissioning.
  //   uint32_t timeCount = 0;
  //   while (!Matter.isDeviceCommissioned()) { // Don't use while loop like this
  //     delay(100);
  //     if ((timeCount++ % 50) == 0) {  // 50*100ms = 5 sec
  //       Serial.println("Matter Node not commissioned yet. Waiting for commissioning.");
  //     }
  //   }

  //   Serial.printf("Initial state: Lift=%d%%\r\n", WindowBlinds.getLiftPercentage());
  //   // Update visualization based on initial state

  //   Serial.println("Matter Node is commissioned and connected to the network. Ready for use.");

  //   delay(2000)
  // }
}
