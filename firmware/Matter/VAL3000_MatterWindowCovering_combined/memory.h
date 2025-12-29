#include <Preferences.h>

uint16_t positionSlider;

bool stall_flag = false;

bool stop_motor = false;
bool opening_direction;

int travel_distance;
float diameter_pulley_cm = 1.2f;  // Only change the value if updating the pulley size
float diameter_pulley_in = diameter_pulley_cm * 0.3937f;

float circumference_cm = diameter_pulley_cm * PI;
float circumference_in = diameter_pulley_in * PI;

int current;
int stall;
int accel;
int max_speed;
int tcools;

bool is_cm;

bool is_closing;
bool is_moving = false;
bool stop_flag = false;
bool overtemp_flag = false;

uint8_t PWM_grad;
uint32_t target_position;
int32_t motor_position;
uint32_t maximum_motor_position;
uint8_t target_percent;



///////////


  //current = 1000;
  //stall = 10;
  //accel = 10000;
  //max_speed = 30000;
  //opening_direction = 0;
  //is_cm = 0;

  

  //motor_position = 0; // load from preferences



//////////



// Filter anti-rebond (debouncer)
#define DEBOUNCE_TIME 250
volatile uint32_t DebounceTimer = 0;

Preferences preferences;


// List of Matter Endpoints for this Node
// Window Covering Endpoint

// it will keep last Lift state stored, using Preferences
//Preferences matterPref;
const char *liftPercentPrefKey = "LiftPercent";

// set your board USER BUTTON pin here
const uint8_t buttonPin = BOOT_PIN;  // Set your pin here. Using BOOT Button.

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




void load_preferences() {

  Serial.println("LOADING PREFERENCES");

  maximum_motor_position = preferences.getInt("max_motor_pos", 2695); // defaults to 20 inches
  motor_position = preferences.getInt("motor_pos", 0);
  current = preferences.getLong("current", 1000);
  stall = preferences.getInt("stall", 10);
  accel = preferences.getInt("accel", 10000);
  max_speed = preferences.getInt("max_speed", 30000);
  opening_direction = preferences.getInt("open_dir", 0);
  is_cm = preferences.getInt("is_cm", 0);
  travel_distance = preferences.getInt("travel_dist", 20);
  target_percent = ((float)motor_position / (float)maximum_motor_position) * 100;

  Serial.println("FINISHED LOADING PREFERENCES");
}
