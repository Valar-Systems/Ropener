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

#define ENABLE_PIN 8
#define RX_PIN 5
#define TX_PIN 6
#define STALLGUARD_PIN 1
#define INDEX_PIN 0

#define BUTTON_1_PIN GPIO_NUM_4
#define BUTTON_2_PIN GPIO_NUM_3
#define WIFI_RESET_PIN GPIO_NUM_7

#define buttonPin GPIO_NUM_4
// List of Matter Endpoints for this Node
// Window Covering Endpoint
MatterWindowCovering WindowBlinds;

// // CONFIG_ENABLE_CHIPOBLE is enabled when BLE is used to commission the Matter Network
// #if !CONFIG_ENABLE_CHIPOBLE
// // WiFi is manually set and started
// const char *ssid = "Tony";            // Change this to your WiFi SSID
// const char *password = "spockntony";  // Change this to your WiFi password
// #endif

// it will keep last Lift state stored, using Preferences
Preferences matterPref;
const char *liftPercentPrefKey = "LiftPercent";


// Button control
uint32_t button_time_stamp = 0;                // debouncing control
bool button_state = false;                     // false = released | true = pressed
const uint32_t debounceTime = 250;             // button debouncing time (ms)
const uint32_t decommissioningTimeout = 5000;  // keep the button pressed for 5s, or longer, to decommission

// Window covering limits
// Lift limits in centimeters (physical position)
const uint16_t MAX_LIFT = 200;  // Maximum lift position (fully open)
const uint16_t MIN_LIFT = 0;    // Minimum lift position (fully closed)

// Current window covering state
// These will be initialized in setup() based on installed limits and saved percentages
uint16_t currentLift = 0;  // Lift position in cm
uint8_t currentLiftPercent = 100;

// Window Covering Callbacks
bool fullOpen() {
  // This is where you would trigger your motor to go to full open state
  // For simulation, we update instantly
  uint16_t openLimit = WindowBlinds.getInstalledOpenLimitLift();
  currentLift = openLimit;
  currentLiftPercent = 100;
  Serial.printf("Opening window covering to full open (position: %d cm)\r\n", currentLift);

  // Update CurrentPosition to reflect actual position (setLiftPercentage now only updates CurrentPosition)
  WindowBlinds.setLiftPercentage(currentLiftPercent);

  // Set operational status to STALL when movement is complete
  WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::STALL);

  // Store state
  matterPref.putUChar(liftPercentPrefKey, currentLiftPercent);

  return true;
}

bool fullClose() {
  // This is where you would trigger your motor to go to full close state
  // For simulation, we update instantly
  uint16_t closedLimit = WindowBlinds.getInstalledClosedLimitLift();
  currentLift = closedLimit;
  currentLiftPercent = 0;
  Serial.printf("Closing window covering to full close (position: %d cm)\r\n", currentLift);

  // Update CurrentPosition to reflect actual position (setLiftPercentage now only updates CurrentPosition)
  WindowBlinds.setLiftPercentage(currentLiftPercent);

  // Set operational status to STALL when movement is complete
  WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::STALL);

  // Store state
  matterPref.putUChar(liftPercentPrefKey, currentLiftPercent);

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
  matterPref.putUChar(liftPercentPrefKey, currentLiftPercent);

  return true;
}

bool stopMotor() {
  // Motor can be stopped while moving cover toward current target
  Serial.println("Stopping window covering motor");

  // Update CurrentPosition to reflect actual position when stopped
  // (setLiftPercentage now only update CurrentPosition)
  WindowBlinds.setLiftPercentage(currentLiftPercent);

  // Set operational status to STALL for both lift
  WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::STALL);

  return true;
}


static void btn3SingleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button3 single click");
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

void setup() {
  // Initialize the USER BUTTON (Boot button) GPIO
  Button btn3 = Button(WIFI_RESET_PIN, false);  //WIFI_RESET_PIN

  btn3.attachSingleClickEventCb(&btn3SingleClickCb, NULL);
  btn3.attachDoubleClickEventCb(&btn3DoubleClickCb, NULL);
  btn3.attachLongPressStartEventCb(&btn3LongPressStartCb, NULL);

  Serial.begin(115200);
  Serial.println("Starting");

// CONFIG_ENABLE_CHIPOBLE is enabled when BLE is used to commission the Matter Network
#if !CONFIG_ENABLE_CHIPOBLE
  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");
  Serial.println(ssid);
  // Manually connect to WiFi
  WiFi.begin(ssid, password);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\r\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  delay(500);
#endif

  // Initialize Matter EndPoint
  matterPref.begin("MatterPrefs", false);
  // default lift percentage is 100% (fully open) if not stored before
  uint8_t lastLiftPercent = matterPref.getUChar(liftPercentPrefKey, 100);

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

// unsigned long previousMillis = 0;
// const long interval = 3000;  // 3 second

void loop() {
  // Check Matter Window Covering Commissioning state, which may change during execution of loop()
  // unsigned long currentMillis = millis();
  // if (currentMillis - previousMillis >= interval) {
  //   previousMillis = currentMillis;                             // Save the time of the last event
  //   Serial.println(ESP.getFreeHeap());                          // Print the free heap memory in bytes
  //   UBaseType_t freeStack = uxTaskGetStackHighWaterMark(NULL);  // Periodically check and print the stack high water mark (minimum free stack)
  //   Serial.printf("Loop task high water mark (min free stack): %u bytes\n", freeStack);
  // }

}
