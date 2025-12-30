uint16_t positionLabel;

#include <TMCStepper.h>

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#define ENABLE_PIN 8
#define RX_PIN 5
#define TX_PIN 6
#define STALLGUARD_PIN 1
#define INDEX_PIN 0

#define BUTTON_1_PIN GPIO_NUM_3
#define BUTTON_2_PIN GPIO_NUM_4
#define WIFI_RESET_PIN GPIO_NUM_7

#define DRIVER_ADDRESS 0b00  // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f        // R_SENSE for current calc.

#define CLOSE_VELOCITY 600
#define OPEN_VELOCITY -600
#define STOP_MOTOR_VELOCITY 0

// function prototypes
void move_close(void);
void move_open(void);
void move_open(void);
void stop(void);

void position_watcher_task(void *parameter);
TaskHandle_t position_watcher_task_handler = NULL;

TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);

void IRAM_ATTR stall_interrupt() {
  stall_flag = true;
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

void setCloseCall() {
  Serial.println("Button Pressed");
  motor_position = maximum_motor_position;
  target_percent = 100;
  Serial.print("set close position: ");
  Serial.println(target_percent);
}


/*
// Function that commands motor to move to position 
void move_to_percent100ths(uint16_t percent100ths) {
  printf("move_to_percent100ths(): %i\n", percent100ths);

  switch (percent100ths) {
    case 0:
      target_position = 0;
      break;
    case 100:
      target_position = maximum_motor_position;
      break;
    default:
      target_position = (percent100ths / 100.0) * maximum_motor_position;
      break;
  }

  //printf("target_position(): %i\n", target_position);
  //printf("motor_position(): %i\n", motor_position);
  //printf("maximum_motor_position(): %i\n", maximum_motor_position);

  if (target_position == motor_position) {
    printf("Not moving the window because it is already at the desired position\n");
    printf("target_position: %li\n", target_position);
    printf("motor_position: %li\n", motor_position);
    return;
  } else if (target_position > motor_position || percent100ths == 100) {
    move_close();
  } else if (target_position < motor_position || percent100ths == 0) {
    move_open();
  }
}



void move_close() {
  printf("move_close()\n");

  if (motor_position < 0) {
    motor_position = 0;
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
}





void move_open() {
  printf("move_open()\n");

  if (motor_position < 0) {
    motor_position = maximum_motor_position;
  }

  printf("motor_position close: %lu\n", motor_position);    // TESTING
  printf("target_position close: %lu\n", target_position);  // TESTING

  printf("max_motor_position close: %lu\n", maximum_motor_position);  // TESTING
  printf("target_percent close: %lu\n", target_percent);              // TESTING

  stop_flag = false;
  is_closing = false;
  is_moving = true;

  xTaskCreate(position_watcher_task, "position_watcher_task", 4096, NULL, 1, &position_watcher_task_handler);

  enable_driver();
  // Add for loop for acceleration
  driver.VACTUAL(OPEN_VELOCITY);
}


*/


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




// Window Covering Callbacks
bool fullOpen() {
  // This is where you would trigger your motor to go to full open state
  // For simulation, we update instantly
  uint16_t openLimit = WindowBlinds.getInstalledOpenLimitLift();
  currentLift = openLimit;
  currentLiftPercent = 100;
  Serial.printf("Opening window covering to full open (position: %d cm)\r\n", currentLift);


  /////////////////
  if (motor_position < 0) {
    motor_position = 0;
  }

  printf("motor_position close: %lu\n", motor_position);    // TESTING
  printf("target_position close: %lu\n", target_position);  // TESTING

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
  
  //if close_until_stop == false
  xTaskCreate(position_watcher_task, "position_watcher_task", 4096, NULL, 1, &position_watcher_task_handler);
  // else
  // Different position watcher task without positioning. Waits until stop_flag is pressed, then sets position to 0
  //xTaskCreate(position_watcher_task_end, "position_watcher_task", 4096, NULL, 1, &position_watcher_task_handler);



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

  target_position = (liftPercent / 100.0) * maximum_motor_position;

  if (target_position == motor_position) {
    printf("Not moving the window because it is already at the desired position\n");
    printf("target_position: %li\n", target_position);
    printf("motor_position: %li\n", motor_position);
    return true;
  } else if (target_position > motor_position || liftPercent == 100) {
    fullOpen();
  } else if (target_position < motor_position || liftPercent == 0) {
    fullClose();
  }


  // This is where you would trigger your motor to go towards liftPercent
  // For simulation, we update instantly
  // Calculate absolute position based on installed limits
  // uint16_t openLimit = WindowBlinds.getInstalledOpenLimitLift();
  // uint16_t closedLimit = WindowBlinds.getInstalledClosedLimitLift();

  // // Linear interpolation: 0% = openLimit, 100% = closedLimit
  // if (openLimit < closedLimit) {
  //   currentLift = openLimit + ((closedLimit - openLimit) * liftPercent) / 100;
  // } else {
  //   currentLift = openLimit - ((openLimit - closedLimit) * liftPercent) / 100;
  // }
  // currentLiftPercent = liftPercent;
  // Serial.printf("Moving lift to %d%% (position: %d cm)\r\n", currentLiftPercent, currentLift);

  return true;
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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// User return statement?
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
        if(loop1 >= 100){
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

// put your setup code here, to run once:
void setup_motors() {

  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STALLGUARD_PIN, INPUT);
  pinMode(INDEX_PIN, INPUT);

  attachInterrupt(STALLGUARD_PIN, stall_interrupt, RISING);
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

  driver.irun(20);       // Max current

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