#include "opendbc/safety/declarations.h"

bool get_longitudinal_allowed(void) {
  return controls_allowed && !gas_pressed_prev;
}

// Safety checks for longitudinal actuation
bool longitudinal_accel_checks(int desired_accel, const LongitudinalLimits limits) {
  bool accel_valid = get_longitudinal_allowed() && !safety_max_limit_check(desired_accel, limits.max_accel, limits.min_accel);
  bool accel_inactive = desired_accel == limits.inactive_accel;
  return !(accel_valid || accel_inactive);
}

bool longitudinal_speed_checks(int desired_speed, const LongitudinalLimits limits) {
  return !get_longitudinal_allowed() && (desired_speed != limits.inactive_speed);
}

bool longitudinal_transmission_rpm_checks(int desired_transmission_rpm, const LongitudinalLimits limits) {
  bool transmission_rpm_valid = get_longitudinal_allowed() && !safety_max_limit_check(desired_transmission_rpm, limits.max_transmission_rpm, limits.min_transmission_rpm);
  bool transmission_rpm_inactive = desired_transmission_rpm == limits.inactive_transmission_rpm;
  return !(transmission_rpm_valid || transmission_rpm_inactive);
}

bool longitudinal_gas_checks(int desired_gas, const LongitudinalLimits limits) {
  bool gas_valid = get_longitudinal_allowed() && !safety_max_limit_check(desired_gas, limits.max_gas, limits.min_gas);
  bool gas_inactive = desired_gas == limits.inactive_gas;
  return !(gas_valid || gas_inactive);
}

bool longitudinal_brake_checks(int desired_brake, const LongitudinalLimits limits) {
  bool violation = false;
  violation |= !get_longitudinal_allowed() && (desired_brake != 0);
  violation |= desired_brake > limits.max_brake;
  return violation;
}

// E2E longitudinal safety checks with ISO 15622 limits
// Used for end-to-end model outputs in Chill mode and Experimental mode
bool e2e_longitudinal_accel_checks(int desired_accel, bool is_chill_mode, float model_confidence) {
  if (!get_longitudinal_allowed()) {
    // When not allowed, only zero acceleration is permitted
    return desired_accel != 0;
  }
  
  // Select limits based on mode and confidence
  int max_accel, min_accel;
  
  if (model_confidence < 0.3f) {
    // Low confidence: use conservative limits
    max_accel = E2E_LOW_CONFIDENCE_MAX_ACCEL_MPS2;
    min_accel = E2E_LOW_CONFIDENCE_MIN_ACCEL_MPS2;
  } else if (is_chill_mode) {
    // Chill mode: ISO 15622 comfortable limits
    max_accel = ISO15622_CHILL_MAX_ACCEL_MPS2;
    min_accel = ISO15622_CHILL_MIN_ACCEL_MPS2;
  } else {
    // Standard/Experimental mode: ISO 15622 normal limits
    max_accel = ISO15622_MAX_ACCEL_MPS2;
    min_accel = ISO15622_MIN_ACCEL_MPS2;
  }
  
  // Check if acceleration is within safe bounds
  return !safety_max_limit_check(desired_accel, max_accel, min_accel);
}

// Jerk limit check for E2E mode
// Ensures smooth transitions between acceleration commands
bool e2e_longitudinal_jerk_check(int desired_accel, int prev_accel, bool is_chill_mode) {
  int jerk = desired_accel - prev_accel;
  int max_jerk = is_chill_mode ? ISO15622_CHILL_MAX_JERK_MPS3 : ISO15622_MAX_JERK_MPS3;
  
  // Jerk limit check (absolute value)
  return (jerk > -max_jerk) && (jerk < max_jerk);
}
