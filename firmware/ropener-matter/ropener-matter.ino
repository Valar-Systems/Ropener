// Requires esp32 v3.3.5

// Partition: Huge APP

/* CONSIDERATIONS
*
* The ESP32-C3 only has 400KB of SRAM.
* Usings Matter on Arduino uses a large amount of SRAM when commissioning.
* The TMCStepper library uses a large amount of SRAM as well.
* If you load the TMCStepper library before Matter commissioning, it will crash.
* Therefore, the TMCStepper library is only loaded after successful Matter Commissioning.
* NOTE: There is a 30 second delay after commissioning before the motors become initialized. This is required, or else the Matter Hub will not connect. 
*     Even though the ESP32 thinks it's commissioned, the process is not complete on the hub until 20-30 seconds later. The delay is required due to low SRAM
* Advise using ESP-IDF to avoid this issue as Matter is optimized much better on it.
*
*/


// ========================================
// LOGGING CONFIGURATION
// Comment out to disable serial logging (saves memory)
// Will NOT commission in Matter if logging is enabled
// Also set USB CDC on Boot to OFF
// ========================================
//#define LOGGING_ENABLED

// Matter Manager
#include <Matter.h>
#include <Preferences.h>
#include <Button.h>

#include "memory.h"

MatterWindowCovering WindowBlinds;

#include "motor_control.h"

// ========================================
// LOCAL STATE VARIABLES
// ========================================

#define PRESSDOWN_DELAY 1500

// Button state management (moved from memory.h - only used in this file)
static bool pressdown = false;        // Press-down state flag
static uint32_t pressdown_timer = 0;  // Press-down timer

// Distance setting mode flag
static bool set_distance = false;  // True when setting distance via double-click

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


// Stop movement, if moving, when button down is pressed
static void btn1PressDownCb(void *button_handle, void *usr_data) {
#ifdef LOGGING_ENABLED
  Serial.println("Button pressed down");
#endif
  if (is_moving) {
#ifdef LOGGING_ENABLED
    Serial.println("Stop Flag - IMMEDIATE STOP");
#endif

    stop_flag = true;
    //is_moving = false; // Will trigger position watcher task

    pressdown = false;
    pressdown_timer = millis() + PRESSDOWN_DELAY;  //start timer to ignore release for 1 second

    if (set_distance) {

      // stop the motor 
      disable_driver();
      driver.VACTUAL(STOP_MOTOR_VELOCITY);

      motor_position = 0;
      preferences.putInt(PREF_MOTOR_POS, motor_position);

      WindowBlinds.setLiftPercentage(100);  // Closed position
      delay(200);
      WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::STALL);
      delay(200);

      set_distance = false;
      is_moving = false;

#ifdef LOGGING_ENABLED
      Serial.print("Motor position: ");
      Serial.println(motor_position);
#endif
    }

    // Update Matter state immediately
    //WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::STALL); // call this in the loop
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

  // Only respond to single click if pressdown is true (not during/after a press down stop)
  if (pressdown) {
#ifdef LOGGING_ENABLED
    Serial.println("pressdown");
#endif
    if (is_moving) {
      stop_flag = true;
    } else {

      WindowBlinds.setTargetLiftPercent100ths(100 * 100);  // Update the move-to position in Matter
      goToLiftPercentage(100);
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

  is_closing = true;
  set_distance = true;
  is_moving = true;

  WindowBlinds.setTargetLiftPercent100ths(100 * 100);  // Update the move-to position in Matter

  enable_driver();
  driver.VACTUAL(CLOSE_VELOCITY);
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

  motor_position = maximum_motor_position;
  preferences.putInt(PREF_MOTOR_POS, motor_position);

  // WindowBlinds.setTargetLiftPercent100ths(100 * 100);  // Update the move-to position in Matter
  // delay(500);
  WindowBlinds.setLiftPercentage(100);  // Fully closed is 100 percent
  delay(500);
  WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::STALL);
  delay(500);

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
    Serial.println("Stop Flag - IMMEDIATE STOP");
#endif

    stop_flag = true;
    //is_moving = false;

    pressdown = false;
    pressdown_timer = millis() + PRESSDOWN_DELAY;  //start timer to ignore release for 1 second

    if (set_distance) {

      //  stop the motor
      disable_driver();
      driver.VACTUAL(STOP_MOTOR_VELOCITY);

      maximum_motor_position = abs(motor_position);
      preferences.putInt(PREF_MAX_MOTOR_POS, motor_position);

      motor_position = 0;  // When fully open, motor position is actually 0
      preferences.putInt(PREF_MOTOR_POS, motor_position);

#ifdef LOGGING_ENABLED
      Serial.print("Motor position: ");
      Serial.println(motor_position);
#endif

      set_distance = false;
      is_moving = false;
      
      // pressdown = false;
      // pressdown_timer = millis() + PRESSDOWN_DELAY;  //start timer to ignore release for 1 second

      // WindowBlinds.setTargetLiftPercent100ths(0 * 100);  // Update the move-to position in Matter
      // delay(500);
      WindowBlinds.setLiftPercentage(0);  // Update Matter to 0 percent position which is full open
      delay(500);
      WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::STALL);

      //Convert distance to centimeters
      float revolutions;
      revolutions = (float)maximum_motor_position / 100;  // 100 pulses per revolution. This gives us the number of revolutions
      MAX_LIFT = (float)revolutions * 3.7699f;            // The curtain will move 3.7699 cm per revolution
      preferences.putInt(PREF_MAX_LIFT, MAX_LIFT);
      WindowBlinds.setInstalledClosedLimitLift(MAX_LIFT);

#ifdef LOGGING_ENABLED
      Serial.println("Updating Matter");
      Serial.println("revolutions: ");
      Serial.println(revolutions);
      Serial.println("MAX_LIFT: ");
      Serial.println(MAX_LIFT);
#endif
    } else {
      // If not in set_distance mode, still update Matter state
      WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::STALL);
    }
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

  // Only respond to single click if pressdown is true (not during/after a press down stop)
  if (pressdown) {
#ifdef LOGGING_ENABLED
    Serial.println("pressdown");
#endif
    if (is_moving) {
      stop_flag = true;
    } else {
      WindowBlinds.setTargetLiftPercent100ths(0 * 100);  // Update the move-to position in Matter
      goToLiftPercentage(0);
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

  is_closing = false;
  set_distance = true;
  is_moving = true;

  motor_position = 0;  // Set current motor position to 0 to begin counting

  WindowBlinds.setTargetLiftPercent100ths(0 * 100);  // Update the move-to position in Matter

  enable_driver();
  driver.VACTUAL(OPEN_VELOCITY);
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
  } else if (opening_direction == 1) {
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
  WindowBlinds.setLiftPercentage(100);
  Matter.decommission();
  delay(500);
  ESP.restart();  // Restart to de-initialize the motor
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
      Serial.println("initialize_motor_system() 1-1");
      initialize_motor_system();
    }

    return true;
  });

  // Matter beginning - Last step, after all EndPoints are initialized
  Matter.begin();

  // Check if device is already commissioned (e.g., after a restart)
  if (Matter.isDeviceCommissioned()) {
#ifdef LOGGING_ENABLED
    Serial.println("1: Matter Node is commissioned and connected to the network. Ready for use.");
    Serial.printf("Initial state: Lift=%d%%\r\n", WindowBlinds.getLiftPercentage());
#endif

    // Device is already commissioned, initialize motors immediately
    Serial.println("initialize_motor_system() 2-1");
    delay(20000);  // Delay required for Matter to connect before initializing motor system. Otherwise low SRAM causes crash
    initialize_motor_system();
  } else {
#ifdef LOGGING_ENABLED
    Serial.println("Matter Node is not commissioned yet.");
    Serial.println("Motor system will be initialized automatically after commissioning.");
    Serial.printf("Manual pairing code: %s\r\n", Matter.getManualPairingCode().c_str());
    Serial.printf("QR code URL: %s\r\n", Matter.getOnboardingQRCodeUrl().c_str());
#endif
  }
}

unsigned long previousMillis = 0;
const long interval = 5000;  // 3 second

void loop() {

  // Keeps the down button from triggering single click on release
  if (millis() >= pressdown_timer) {
    pressdown = true;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Save the time of the last event

    if (!Matter.isDeviceCommissioned()) {
#ifdef LOGGING_ENABLED
      Serial.println("Matter Node is not commissioned yet.");
      Serial.printf("Manual pairing code: %s\r\n", Matter.getManualPairingCode().c_str());
      Serial.printf("QR code URL: %s\r\n", Matter.getOnboardingQRCodeUrl().c_str());
#endif
    } else {
      //Serial.println("2: Matter Node is commissioned and connected to the network. Ready for use.");
      if (!motor_initialized && Matter.isDeviceCommissioned()) {
        //Serial.println("initialize_motor_system() 3-1");
        delay(30000);  // Delay initializing motor for 30 seconds after Matter device commissioned. For an unknown reason, the Hub takes times to create this device and the delay is required to prevent crash due to low SRAM on ESP32-C3
        initialize_motor_system();
      }
    }
  }
}
