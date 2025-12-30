// it will keep last Lift state stored, using Preferences
Preferences preferences;
const char *liftPercentPrefKey = "LiftPercent";

// set your board USER BUTTON pin here
const uint8_t buttonPin = GPIO_NUM_4;  // Set your pin here. Using BOOT Button.

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

bool is_moving = false;
bool pressdown = false;
int pressdown_timer;

bool set_distance;
bool is_closing;
bool stop_flag = false;
bool opening_direction;

uint32_t maximum_motor_position = 2000;
int32_t motor_position = 0;

uint32_t target_position;
uint8_t target_percent;

bool stall_flag = false;
