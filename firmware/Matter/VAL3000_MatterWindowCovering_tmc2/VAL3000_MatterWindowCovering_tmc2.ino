// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Matter Manager
#include <Matter.h>
// #if !CONFIG_ENABLE_CHIPOBLE
// // if the device can be commissioned using BLE, WiFi is not used - save flash space
// #include <WiFi.h>
// #endif
#include <Preferences.h>
#include <Button.h>
#include <TMCStepper.h>
#include <ArduinoOTA.h>  // For enabling over-the-air updates

#include "memory.h"
#include "motor_control.h"


#define BUTTON_1_PIN GPIO_NUM_4
#define BUTTON_2_PIN GPIO_NUM_3
#define WIFI_RESET_PIN GPIO_NUM_7

#define ENABLE_PIN 8
#define RX_PIN 5
#define TX_PIN 6
#define STALLGUARD_PIN 1
#define INDEX_PIN 0

#define DRIVER_ADDRESS 0b00  // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f        // R_SENSE for current calc.

TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);

#define CLOSE_VELOCITY 600
#define OPEN_VELOCITY -600
#define STOP_MOTOR_VELOCITY 0

#define PRESSDOWN_DELAY 2000

// List of Matter Endpoints for this Node
// Window Covering Endpoint
MatterWindowCovering WindowBlinds;

void position_watcher_task(void *parameter);
TaskHandle_t position_watcher_task_handler = NULL;

// // CONFIG_ENABLE_CHIPOBLE is enabled when BLE is used to commission the Matter Network
// #if !CONFIG_ENABLE_CHIPOBLE
// // WiFi is manually set and started
// const char *ssid = "Tony";            // Change this to your WiFi SSID
// const char *password = "spockntony";  // Change this to your WiFi password
// #endif



// Stop movement, if moving, when button down is pressed
static void btn1PressDownCb(void *button_handle, void *usr_data) {
  Serial.println("Button pressed down");

  // Triggers single click which starts motor again
  if (is_moving) {

    disable_driver();
    driver.VACTUAL(STOP_MOTOR_VELOCITY);
    is_moving = false;
    pressdown = false;
    pressdown_timer = millis() + PRESSDOWN_DELAY;  //start timer

    if (set_distance) {
      int targetLiftPercent = 0;
      WindowBlinds.setTargetLiftPercent100ths(targetLiftPercent * 100);
      set_distance = false;
    }
  }
}

// Move to full close position
static void btn1SingleClickCb(void *button_handle, void *usr_data) {

  if (pressdown) {
    Serial.println("Button1 single click");
    digitalWrite(ENABLE_PIN, 0);
    driver.VACTUAL(CLOSE_VELOCITY);
    is_moving = true;
  }

  uint8_t targetLiftPercent = currentLiftPercent;
  // go to the closest next 20% or move 20% more
  if ((targetLiftPercent % 20) != 0) {
    targetLiftPercent = ((targetLiftPercent / 20) + 1) * 20;
  } else {
    targetLiftPercent += 20;
  }
  if (targetLiftPercent > 100) {
    targetLiftPercent = 0;
  }
  Serial.printf("User button released. Setting lift to %d%%\r\n", targetLiftPercent);
  WindowBlinds.setTargetLiftPercent100ths(targetLiftPercent * 100);
}

// Move until stop button is pressed
static void btn1DoubleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button1 double click");

  if (pressdown) {
    is_moving = true;
    is_closing = true;
    set_distance = true;
    digitalWrite(ENABLE_PIN, 0);
    driver.VACTUAL(CLOSE_VELOCITY);
  }
}

// If not moving, sets zero position
static void btn1LongPressStartCb(void *button_handle, void *usr_data) {
  Serial.println("Button1 long press click");
}

static void btn2PressDownCb(void *button_handle, void *usr_data) {
  Serial.println("Button2 pressed down");
  if (is_moving) {

    digitalWrite(ENABLE_PIN, 1);
    driver.VACTUAL(STOP_MOTOR_VELOCITY);
    is_moving = false;
    maximum_motor_position = motor_position;
    pressdown = false;
    pressdown_timer = millis() + PRESSDOWN_DELAY;  //start timer to ignore release for 1 second

    if (set_distance) {
      int targetLiftPercent = 100;
      WindowBlinds.setTargetLiftPercent100ths(targetLiftPercent * 100);
      set_distance = false;
    }
  }
}

// Move to full Open position
static void btn2SingleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button2 single click");
  if (pressdown) {
    digitalWrite(ENABLE_PIN, 0);
    driver.VACTUAL(OPEN_VELOCITY);
  }
}

static void btn2DoubleClickCb(void *button_handle, void *usr_data) {

  if (pressdown) {
    Serial.println("Button2 double click");
    is_moving = true;
    set_distance = true;
    digitalWrite(ENABLE_PIN, 0);
    driver.VACTUAL(OPEN_VELOCITY);
  }
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
}

static void btn3LongPressStartCb(void *button_handle, void *usr_data) {
  Serial.println("Button3 long press click");
  //Reset matter
  Serial.println("Decommissioning the Window Covering Matter Accessory. It shall be commissioned again.");
  WindowBlinds.setLiftPercentage(0);  // close the covering
  Matter.decommission();
}



// Interrupt tracks the position of the stepper motor using the index pin
void IRAM_ATTR index_interrupt(void) {

  if (is_closing == true) {
    motor_position++;
  } else {
    motor_position--;
  }

  // Ensure motor position stays within bounds
  if (motor_position < 0) {
    motor_position = 0;
  } else if (motor_position > maximum_motor_position) {
    motor_position = maximum_motor_position;
  }
}

int getMotorPosition() {
  return motor_position;
}

/* Enables power stage of TMC */
void enable_driver() {
  digitalWrite(ENABLE_PIN, 0);
}

/* Disabled power stage of TMC */
void disable_driver() {
  digitalWrite(ENABLE_PIN, 1);
}



// Window Covering Callbacks
bool fullOpen() {
  // This is where you would trigger your motor to go to full open state
  uint16_t openLimit = WindowBlinds.getInstalledOpenLimitLift();
  currentLift = openLimit;
  currentLiftPercent = 100;

  Serial.printf("Opening window covering to full open (position: %d cm)\r\n", currentLift);
  if (motor_position < 0) {
    motor_position = 0;
  }
  printf("motor_position close: %lu\n", motor_position);              // TESTING
  printf("target_position close: %lu\n", target_position);            // TESTING
  printf("max_motor_position close: %lu\n", maximum_motor_position);  // TESTING
  printf("target_percent close: %lu\n", target_percent);              // TESTING

  stop_flag = false;
  is_closing = false;
  is_moving = true;
  
  xTaskCreate(position_watcher_task, "position_watcher_task", 4096, NULL, 1, &position_watcher_task_handler);
  
  enable_driver();
  // Add for loop for acceleration
  driver.VACTUAL(OPEN_VELOCITY);

  return true;
}

bool fullClose() {
  // This is where you would trigger your motor to go to full close state
  // For simulation, we update instantly
  uint16_t closedLimit = WindowBlinds.getInstalledClosedLimitLift();
  currentLift = closedLimit;
  currentLiftPercent = 0;
  Serial.printf("Closing window covering to full close (position: %d cm)\r\n", currentLift);

  if (motor_position > maximum_motor_position) {
    motor_position = maximum_motor_position;
  }

  // printf("motor_position close: %lu\n", motor_position);    // TESTING
  // printf("target_position close: %lu\n", target_position);  // TESTING

  // printf("max_motor_position close: %lu\n", maximum_motor_position);  // TESTING
  // printf("target_percent close: %lu\n", target_percent);  // TESTING

  stop_flag = false;
  is_closing = true;
  is_moving = true;

  xTaskCreate(position_watcher_task, "position_watcher_task", 4096, NULL, 1, &position_watcher_task_handler);

  enable_driver();
  // Add for loop for acceleration
  driver.VACTUAL(CLOSE_VELOCITY);

  return true;
}

bool goToLiftPercentage(uint8_t liftPercent) {
  // update Lift operational state
  if (liftPercent > currentLiftPercent) {
    // Set operational status to OPEN
    WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::MOVING_UP_OR_OPEN);
  }
  if (liftPercent < currentLiftPercent) {
    // Set operational status to CLOSE
    WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::MOVING_DOWN_OR_CLOSE);
  }

  // This is where you would trigger your motor to go towards liftPercent
  // For simulation, we update instantly
  // Calculate absolute position based on installed limits
  uint16_t openLimit = WindowBlinds.getInstalledOpenLimitLift();
  uint16_t closedLimit = WindowBlinds.getInstalledClosedLimitLift();

  // Linear interpolation: 0% = openLimit, 100% = closedLimit
  if (openLimit < closedLimit) {
    currentLift = openLimit + ((closedLimit - openLimit) * liftPercent) / 100;
  } else {
    currentLift = openLimit - ((openLimit - closedLimit) * liftPercent) / 100;
  }
  currentLiftPercent = liftPercent;
  Serial.printf("Moving lift to %d%% (position: %d cm)\r\n", currentLiftPercent, currentLift);

  // Update CurrentPosition to reflect actual position (setLiftPercentage now only updates CurrentPosition)
  WindowBlinds.setLiftPercentage(currentLiftPercent);

  // Set operational status to STALL when movement is complete
  WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::STALL);

  // Store state
  preferences.putUChar(liftPercentPrefKey, currentLiftPercent);

  return true;
}



void setup() {
  // Initialize the USER BUTTON (Boot button) GPIO
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


  Serial.begin(115200);
  delay(100);

  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // ESP32 can use any pins to Serial
  delay(100);

  setup_motors();

  // Initialize Matter EndPoint
  preferences.begin("preferencess", false);
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

  // This keeps the down button from triggering single click on release
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

bool stopMotor() {
  // Motor can be stopped while moving cover toward current target
  Serial.println("Stopping window covering motor");
  stop_flag = true;
  return true;
}

/* Stops motor */
void stop() {
  disable_driver();
  printf("stop(): disable_driver\n");
  driver.VACTUAL(STOP_MOTOR_VELOCITY);
  printf("stop(): driver.VACTUAL(STOP_MOTOR_VELOCITY)\n");
}

void position_watcher_task(void *parameter) {

  int loop1 = 0;

  while (true) {

    while (is_moving) {

      Serial.println(motor_position);
      loop1++;

      // check if button was pressed
      if (stop_flag) {
        stop();
        stop_flag = false;
        printf("position_watcher: button pressed stop == true\n");
        delay(1000);
        goto notify_and_suspend;
      }

      // Check for stall flag //
      if (stall_flag) {
        stop();
        stall_flag = false;
        printf("position_watcher: stall_flag == true\n");
        goto notify_and_suspend;
      }

      // /* Check if Position reached */
      if (is_closing) {
        if (motor_position >= target_position) {
          printf("position_watcher STOPPING because target_position: %u <= motor_position: %u\n", (unsigned int)target_position, (unsigned int)motor_position);
          stop();
          goto notify_and_suspend;
        }
      } else {
        if (motor_position <= target_position) {
          printf("position_watcher_task STOPPING because target_position: %u >= motor_position: %u\n", (unsigned int)target_position, (unsigned int)motor_position);
          stop();
          goto notify_and_suspend;
        }
      }

      // Update position in Matter every once is a while
      if (loop1 >= 100) {
        currentLiftPercent = ((float)motor_position / (float)maximum_motor_position) * 100;
        WindowBlinds.setLiftPercentage(currentLiftPercent);
        loop1 = 0;
      }

      delay(20);
    }

notify_and_suspend:
    is_moving = false;
    currentLiftPercent = ((float)motor_position / (float)maximum_motor_position) * 100;

    // Update CurrentPosition to reflect actual position (setLiftPercentage now only updates CurrentPosition)
    WindowBlinds.setLiftPercentage(currentLiftPercent);

    // Set operational status to STALL when movement is complete
    WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::STALL);

    // Store state
    preferences.putUChar(liftPercentPrefKey, currentLiftPercent);
    preferences.putInt("motor_pos", motor_position);

    vTaskDelete(NULL);
  }
}


uint8_t PWM_grad;

// put your setup code here, to run once:
void setup_motors() {

  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STALLGUARD_PIN, INPUT);
  pinMode(INDEX_PIN, INPUT);

  //attachInterrupt(STALLGUARD_PIN, stall_interrupt, RISING);
  attachInterrupt(INDEX_PIN, index_interrupt, RISING);


  if (opening_direction == 1) {
    driver.shaft(true);
  } else {
    driver.shaft(false);
  }

  /* General Registers */
  driver.I_scale_analog(false);
  driver.internal_Rsense(false);
  driver.en_spreadCycle(false);
  driver.index_otpw(false);
  driver.index_step(true);
  driver.pdn_disable(true);
  driver.mstep_reg_select(true);
  driver.multistep_filt(true);

  // NODECONF Registers
  driver.senddelay(6);

  // Factory Registers
  driver.ottrim(0);

  // Velocity Dependent Control
  driver.ihold(0);
  driver.iholddelay(1);  // Set I_HOLD_DELAY to 1 to 15 for smooth standstill current decay
  driver.TPOWERDOWN(20);
  driver.TPWMTHRS(0);
  driver.VACTUAL(0);

  driver.irun(20);  // Max current

  driver.TCOOLTHRS(80);  // 20 to turn off coolstep to test vibrations//600 ok//

  // Stallguard or CoolStep NOT currently supported. Coming soon

  driver.SGTHRS(120);  //
  // Once SGTHRS has been determined, use 1/16*SGTHRS+1
  // as a starting point for SEMIN.

  // Working values
  // SGTHRS = 100 / semin = 5 // not great, sometimes skips
  // 120/6 working great

  driver.semin(6);  // 6 is great. 5 causes skipped steps. Sometimes 6, always 7 causes current to remain high.
  driver.seup(0);   // 0 works great

  driver.semax(0);  // 0-15 // 0 to 2 recommended
  driver.sedn(0);
  driver.seimin(1);

  // CHOPCONF – Chopper Configuration
  driver.diss2vs(0);
  driver.diss2g(0);
  driver.dedge(0);
  driver.intpol(1);

  driver.mres(8);  // 8 = FULLSTEP mode. 200 pulses per revolution.
  driver.vsense(0);
  driver.tbl(2);
  driver.hend(0);
  driver.hstrt(4);
  driver.toff(5);

  // PWMCONF – Voltage PWM Mode StealthChop
  driver.pwm_lim(12);
  driver.pwm_reg(8);    // Try 2
  driver.freewheel(1);  // 1= Freewheel mode. 3 = Coil Short HS. Only short the coil when motor is NOT moving. Use for physical security
  driver.pwm_autograd(1);
  driver.pwm_autoscale(1);
  driver.pwm_freq(1);
  driver.pwm_grad(PWM_grad);  // Test different initial values. Use scope.
  driver.pwm_ofs(36);
}
