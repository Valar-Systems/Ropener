#include <Preferences.h>

// ========================================
// MOTOR STATE VARIABLES
// ========================================

// Motor position tracking
volatile int motor_position = 0;           // Current motor position (steps)
int target_position = 0;                   // Target motor position (steps)
int maximum_motor_position = 500;          // Maximum motor position (steps) - loaded from preferences

// Motor movement state
volatile bool is_closing = false;          // Direction flag: true = closing, false = opening
volatile bool is_moving = false;           // Movement flag: true = motor is moving
volatile bool stop_flag = false;           // Stop request flag
volatile bool stall_flag = false;          // Stallguard interrupt flag

// Motor initialization state (deferred until after commissioning)
bool motor_initialized = false;

// Motor direction configuration
bool opening_direction = 0;                // Motor shaft direction: 0 = normal, 1 = reversed

// ========================================
// MOTOR HARDWARE CONFIGURATION
// ========================================

// TMC2209 PWM configuration (StealthChop)
#define PWM_GRAD 0                         // PWM gradient for StealthChop



// ========================================
// MATTER WINDOW COVERING STATE
// ========================================

// Preferences storage
Preferences preferences;

// Preference keys (using #define to save RAM)
#define PREF_LIFT_PERCENT "LiftPercent"
#define PREF_MAX_MOTOR_POS "max_motor_pos"
#define PREF_MOTOR_POS "motor_pos"
#define PREF_OPEN_DIR "open_dir"
#define PREF_TRAVEL_DIST "travel_dist"

// Window covering limits (physical position in centimeters)
uint16_t MAX_LIFT = 200;                   // Maximum lift position (fully open) - can be updated
const uint16_t MIN_LIFT = 0;               // Minimum lift position (fully closed)

// Current window covering state
uint8_t currentLiftPercent = 100;          // Current lift percentage (0-100, Matter standard)


// ========================================
// PREFERENCE LOADING FUNCTION
// ========================================

/**
 * @brief Load saved preferences from flash memory
 *
 * Loads motor position, maximum position, and direction from non-volatile storage.
 * Calculates current lift percentage based on loaded motor position.
 */
void load_preferences() {
  #ifdef LOGGING_ENABLED
    Serial.println("Loading preferences from flash...");
  #endif

  // Load motor configuration
  maximum_motor_position = preferences.getInt(PREF_MAX_MOTOR_POS, 500);  // Default: 500 steps (~20 inches)
  motor_position = preferences.getInt(PREF_MOTOR_POS, 0);                // Default: 0 (closed position)
  opening_direction = preferences.getInt(PREF_OPEN_DIR, 0);              // Default: 0 (normal direction)

  // Calculate current percentage based on loaded position (inverted for Matter standard)
  if (maximum_motor_position > 0) {
    currentLiftPercent = 100 - (((float)motor_position / (float)maximum_motor_position) * 100.0f);
  } else {
    currentLiftPercent = 100;  // Default to fully closed
  }

  #ifdef LOGGING_ENABLED
    Serial.println("Preferences loaded successfully.");
    Serial.printf("  Motor position: %d / %d steps\n", motor_position, maximum_motor_position);
    Serial.printf("  Opening direction: %d\n", opening_direction);
    Serial.printf("  Current lift: %d%%\n", currentLiftPercent);
  #endif
}
