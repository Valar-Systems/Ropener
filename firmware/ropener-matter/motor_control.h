// ========================================
// MOTOR CONTROL HEADER
// ========================================

#include <TMCStepper.h>

// ========================================
// HARDWARE PIN DEFINITIONS
// ========================================

// TMC2209 Motor Driver Pins
#define ENABLE_PIN 8                       // Motor driver enable pin (LOW = enabled)
#define RX_PIN 5                           // UART RX for TMC2209 communication
#define TX_PIN 6                           // UART TX for TMC2209 communication
#define STALLGUARD_PIN 1                   // StallGuard interrupt pin
#define INDEX_PIN 0                        // Index/step pulse interrupt pin

// Button Pins
#define BUTTON_1_PIN GPIO_NUM_4            // Button 1 (Close control)
#define BUTTON_2_PIN GPIO_NUM_3            // Button 2 (Open control)
#define WIFI_RESET_PIN GPIO_NUM_7          // Button 3 (Direction/Decommission)

// ========================================
// TMC2209 DRIVER CONFIGURATION
// ========================================

#define DRIVER_ADDRESS 0b00                // TMC2209 address (MS1=0, MS2=0)
#define R_SENSE 0.11f                      // Current sense resistor value

// Motor velocity constants (steps/second)
#define OPEN_VELOCITY 600                  // Velocity for opening movement
#define CLOSE_VELOCITY -600                // Velocity for closing movement (negative)
#define STOP_MOTOR_VELOCITY 0              // Velocity to stop motor

// ========================================
// FUNCTION PROTOTYPES
// ========================================

void move_close(void);
void move_open(void);
void stop(void);
void position_watcher_task(void *parameter);

// ========================================
// GLOBAL OBJECTS
// ========================================

// FreeRTOS task handle for position monitoring
TaskHandle_t position_watcher_task_handler = NULL;

// TMC2209 stepper driver instance
TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);

void IRAM_ATTR stall_interrupt() {
  stall_flag = true;
}

// Interrupt tracks the position of the stepper motor using the index pin
void IRAM_ATTR index_interrupt(void) {

  if (is_closing) {
    motor_position--;
  } else {
    motor_position++;
  }

  // Ensure motor position stays within bounds
  // if (motor_position < 0) {
  //   motor_position = 0;
  // } else if (motor_position > maximum_motor_position) {
  //   motor_position = maximum_motor_position;
  // }
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

// ========================================
// WINDOW COVERING CALLBACKS
// ========================================




// Window Covering Callbacks
bool fullOpen() {
  // Safety check: Don't operate motors if not initialized
  if (!motor_initialized) {
    #ifdef LOGGING_ENABLED
      Serial.println("fullOpen() called but motors not initialized yet. Ignoring.");
    #endif
    return false;
  }

  target_position = maximum_motor_position;
  printf("fullOpen: %lu\n", target_position);

  // Clamp motor position to valid range
  if (motor_position > maximum_motor_position) {
    motor_position = maximum_motor_position;
  }
  if (motor_position < 0) {
    motor_position = 0;
  }

  #ifdef LOGGING_ENABLED
    printf("target_position open: %lu\n", target_position);
    printf("motor_position open: %lu\n", motor_position);
    printf("max_motor_position open: %lu\n", maximum_motor_position);
  #endif

  stop_flag = false;
  is_closing = false;
  is_moving = true;

  if (target_position == motor_position) {
    #ifdef LOGGING_ENABLED
      Serial.println("target_position == motor_position");
    #endif
  } else {
    WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::MOVING_UP_OR_OPEN);
    vTaskResume(position_watcher_task_handler);
    enable_driver();
    driver.VACTUAL(OPEN_VELOCITY);
  }

  return true;
}

bool fullClose() {
  // Safety check: Don't operate motors if not initialized
  if (!motor_initialized) {
    #ifdef LOGGING_ENABLED
      Serial.println("fullClose() called but motors not initialized yet. Ignoring.");
    #endif
    return false;
  }

  target_position = 0;
  printf("fullClose: %lu\n", target_position);

  // Clamp motor position to valid range
  if (motor_position > maximum_motor_position) {
    motor_position = maximum_motor_position;
  }
  if (motor_position < 0) {
    motor_position = 0;
  }

  #ifdef LOGGING_ENABLED
    printf("target_position close: %lu\n", target_position);
    printf("motor_position close: %lu\n", motor_position);
    printf("max_motor_position close: %lu\n", maximum_motor_position);
  #endif

  stop_flag = false;
  is_closing = true;
  is_moving = true;

  if (target_position == motor_position) {
    #ifdef LOGGING_ENABLED
      Serial.println("target_position == motor_position");
    #endif
  } else {
    WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::MOVING_DOWN_OR_CLOSE);
    vTaskResume(position_watcher_task_handler);
    enable_driver();
    driver.VACTUAL(CLOSE_VELOCITY);
  }

  return true;
}

bool goToLiftPercentage(uint8_t liftPercent) {
  // Safety check: Don't operate motors if not initialized
  if (!motor_initialized) {
    #ifdef LOGGING_ENABLED
      Serial.println("goToLiftPercentage() called but motors not initialized yet. Ignoring.");
    #endif
    return false;
  }

  printf("goToLiftPercentage: %lu\n", liftPercent);

  // Stop current movement if in progress
  if (is_moving) {
    stop_flag = true;
    delay(1000);
  }

  // Calculate target position from percentage
  target_position = (liftPercent / 100.0) * maximum_motor_position;

  // Clamp motor position to valid range
  if (motor_position > maximum_motor_position) {
    motor_position = maximum_motor_position;
  }
  if (motor_position < 0) {
    motor_position = 0;
  }

  #ifdef LOGGING_ENABLED
    printf("target_position: %lu\n", target_position);
    printf("motor_position: %lu\n", motor_position);
    printf("max_motor_position: %lu\n", maximum_motor_position);
  #endif


  if (target_position == motor_position) {
    printf("Not moving the window because it is already at the desired position\n");
    return true;
  } else if (target_position < motor_position) {
    #ifdef LOGGING_ENABLED
      Serial.println(" goToLiftPercentage CLOSING");
    #endif

    stop_flag = false;
    is_closing = true;
    is_moving = true;

    vTaskResume(position_watcher_task_handler);
    delay(100);
    enable_driver();
    driver.VACTUAL(CLOSE_VELOCITY);

    WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::MOVING_DOWN_OR_CLOSE);

  } else if (target_position > motor_position) {
    #ifdef LOGGING_ENABLED
      Serial.println("goToLiftPercentage OPENING");
    #endif

    stop_flag = false;
    is_closing = false;
    is_moving = true;

    vTaskResume(position_watcher_task_handler);
    delay(100);
    enable_driver();
    driver.VACTUAL(OPEN_VELOCITY);

    WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::MOVING_UP_OR_OPEN);
  }

  return true;
}

bool stopMotor() {
  // Safety check: Don't operate motors if not initialized
  if (!motor_initialized) {
    #ifdef LOGGING_ENABLED
      Serial.println("stopMotor() called but motors not initialized yet. Ignoring.");
    #endif
    return false;
  }

  // Motor can be stopped while moving cover toward current target
  #ifdef LOGGING_ENABLED
    Serial.println("Stopping window covering motor");
  #endif
  stop_flag = true;
  return true;
}

/* Stops motor */
void stop() {
  disable_driver();
  driver.VACTUAL(STOP_MOTOR_VELOCITY);
  #ifdef LOGGING_ENABLED
    printf("stop(): Motor stopped\n");
  #endif
}

// ========================================
// POSITION WATCHER TASK
// ========================================

/**
 * @brief FreeRTOS task that monitors motor position and stops at target
 *
 * This task runs continuously, suspended when not needed. It monitors the motor
 * position and stops movement when target is reached or stop conditions occur.
 */
void position_watcher_task(void *parameter) {
  #ifdef LOGGING_ENABLED
    Serial.println("position_watcher_task CREATED");
  #endif

  int loop_counter = 0;


  while (true) {
    vTaskSuspend(NULL);

    #ifdef LOGGING_ENABLED
      Serial.print("is_moving: ");
      Serial.println(is_moving);
    #endif

    while (is_moving) {
      loop_counter++;

      // Check if stop button was pressed
      if (stop_flag) {
        stop();
        stop_flag = false;
        #ifdef LOGGING_ENABLED
          printf("position_watcher: Stop requested\n");
        #endif
        delay(1000);
        goto notify_and_suspend;
      }

      // Check for stall condition
      if (stall_flag) {
        stop();
        stall_flag = false;
        #ifdef LOGGING_ENABLED
          printf("position_watcher: Stall detected\n");
        #endif
        goto notify_and_suspend;
      }

      // Check if target position reached
      if (is_closing) {
        if (motor_position <= target_position) {
          #ifdef LOGGING_ENABLED
            printf("position_watcher: Target reached (closing) - pos: %u, target: %u\n",
                   (unsigned int)motor_position, (unsigned int)target_position);
          #endif
          stop();
          goto notify_and_suspend;
        }
      } else {
        if (motor_position >= target_position) {
          #ifdef LOGGING_ENABLED
            printf("position_watcher: Target reached (opening) - pos: %u, target: %u\n",
                   (unsigned int)motor_position, (unsigned int)target_position);
          #endif
          stop();
          goto notify_and_suspend;
        }
      }

      // Periodic position logging
      if (loop_counter >= 20) {
        #ifdef LOGGING_ENABLED
          Serial.println(motor_position);
        #endif
        loop_counter = 0;
      }

      delay(20);
    }

notify_and_suspend:
    #ifdef LOGGING_ENABLED
      Serial.println("Movement complete - updating state");
    #endif
    is_moving = false;

    // Calculate current lift percentage (inverted for Matter standard: 0% = open, 100% = closed)
    int currentLiftPercent = 100 - (((float)motor_position / (float)maximum_motor_position) * 100.0);

    #ifdef LOGGING_ENABLED
      printf("Final state - Position: %lu, Lift%%: %lu, Target: %lu, Max: %lu\n",
             motor_position, currentLiftPercent, target_position, maximum_motor_position);
    #endif

    // Update Matter state
    WindowBlinds.setLiftPercentage(currentLiftPercent);
    WindowBlinds.setOperationalState(MatterWindowCovering::LIFT, MatterWindowCovering::STALL);

    // Save state to preferences
    preferences.putUChar(PREF_LIFT_PERCENT, currentLiftPercent);
    preferences.putInt(PREF_MOTOR_POS, motor_position);
  }
}

// ========================================
// MOTOR SETUP FUNCTION
// ========================================

/**
 * @brief Initialize TMC2209 motor driver and configure all parameters
 *
 * Sets up GPIO pins, interrupts, and configures the TMC2209 driver with
 * optimized settings for StealthChop operation.
 */
void setup_motors() {
  // Configure GPIO pins
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STALLGUARD_PIN, INPUT);
  pinMode(INDEX_PIN, INPUT);

  // Attach interrupt handlers
  attachInterrupt(STALLGUARD_PIN, stall_interrupt, RISING);
  attachInterrupt(INDEX_PIN, index_interrupt, RISING);

  // Set motor shaft direction
  driver.shaft(opening_direction == 1);

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

  driver.TCOOLTHRS(80);

  // StallGuard and CoolStep configuration
  driver.SGTHRS(120);  // StallGuard threshold (120 works well with semin=6)
  driver.semin(6);     // CoolStep minimum current (6 optimal - prevents skipped steps)
  driver.seup(0);      // Current increment steps
  driver.semax(0);     // Maximum current (0-2 recommended)
  driver.sedn(0);      // Current decrement steps
  driver.seimin(1);    // Minimum current flag

  // CHOPCONF – Chopper Configuration
  driver.diss2vs(0);
  driver.diss2g(0);
  driver.dedge(0);
  driver.intpol(1);

  driver.mres(8);      // Microstep resolution: 8 = FULLSTEP (200 pulses/revolution)
  driver.vsense(0);    // Voltage sense
  driver.tbl(2);       // Blank time
  driver.hend(0);      // Hysteresis end
  driver.hstrt(4);     // Hysteresis start
  driver.toff(5);      // Off time

  // PWMCONF – Voltage PWM Mode StealthChop
  driver.pwm_lim(12);       // PWM limit
  driver.pwm_reg(8);        // PWM regulation
  driver.freewheel(1);      // Freewheel mode (1 = normal, 3 = coil short for security)
  driver.pwm_autograd(1);   // Automatic gradient adaptation
  driver.pwm_autoscale(1);  // Automatic amplitude scaling
  driver.pwm_freq(1);       // PWM frequency
  driver.pwm_grad(PWM_GRAD); // PWM gradient
  driver.pwm_ofs(36);       // PWM offset
}