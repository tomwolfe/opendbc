#include "opendbc/safety/declarations.h"

// ISO 15622:2018 ACC limits for longitudinal control
// These limits ensure safe acceleration/deceleration for passenger comfort and safety
static const float ISO_LONG_ACCEL_MAX = 2.0;    // m/s^2 - maximum acceleration
static const float ISO_LONG_DECEL_MAX = 3.5;    // m/s^2 - maximum deceleration (braking)
static const float ISO_LONG_JERK_MAX = 5.0;     // m/s^3 - maximum jerk

// E2E policy safety buffer - additional margin for neural network outputs
static const float E2E_ACCEL_SAFETY_MARGIN = 0.3;  // m/s^2 - extra margin for E2E uncertainty

bool get_longitudinal_allowed(void) {
  return controls_allowed && !gas_pressed_prev;
}

// Safety checks for longitudinal actuation
bool longitudinal_accel_checks(int desired_accel, const LongitudinalLimits limits) {
  bool accel_valid = get_longitudinal_allowed() && !safety_max_limit_check(desired_accel, limits.max_accel, limits.min_accel);
  bool accel_inactive = desired_accel == limits.inactive_accel;
  return !(accel_valid || accel_inactive);
}

// E2E Policy acceleration gating - ensures neural network outputs comply with ISO 15622
// This is critical for Full E2E driving where the model directly outputs acceleration commands
bool e2e_longitudinal_accel_checks(float desired_accel, float current_accel, float prev_accel, uint32_t dt_us) {
  bool violation = false;
  
  // Only check when controls are allowed
  if (!controls_allowed) {
    return desired_accel != 0;  // Must be zero when controls not allowed
  }
  
  // Convert to m/s^2 if needed (assuming input is already in m/s^2)
  float accel_cmd = desired_accel;
  
  // 1. ISO 15622 absolute acceleration limits
  // Acceleration must be within safe bounds for passenger comfort
  if (accel_cmd > (ISO_LONG_ACCEL_MAX + E2E_ACCEL_SAFETY_MARGIN)) {
    violation = true;
  }
  if (accel_cmd < -(ISO_LONG_DECEL_MAX + E2E_ACCEL_SAFETY_MARGIN)) {
    violation = true;
  }
  
  // 2. Jerk limit - rate of change of acceleration
  // Prevents sudden changes that could cause discomfort or instability
  if (dt_us > 0) {
    float dt_sec = dt_us / 1000000.0;
    float jerk = (accel_cmd - prev_accel) / dt_sec;
    
    if (jerk > ISO_LONG_JERK_MAX) {
      violation = true;
    }
    if (jerk < -ISO_LONG_JERK_MAX) {
      violation = true;
    }
  }
  
  // 3. Consistency check with current vehicle acceleration
  // E2E model command should not deviate too far from actual vehicle dynamics
  // This catches potential model hallucinations or distribution shift
  static const float MAX_ACCEL_ERROR = 8.0;  // m/s^2 - maximum allowed error
  float accel_error = accel_cmd - current_accel;
  if (accel_error > MAX_ACCEL_ERROR || accel_error < -MAX_ACCEL_ERROR) {
    // Large discrepancy - model may be out of distribution
    violation = true;
  }
  
  return violation;
}

// Clip E2E acceleration command to ISO 15622 limits
// Returns the clipped acceleration value
float e2e_clip_acceleration(float desired_accel, float prev_accel, uint32_t dt_us) {
  float accel_cmd = desired_accel;
  
  // Apply absolute limits
  if (accel_cmd > ISO_LONG_ACCEL_MAX) {
    accel_cmd = ISO_LONG_ACCEL_MAX;
  }
  if (accel_cmd < -ISO_LONG_DECEL_MAX) {
    accel_cmd = -ISO_LONG_DECEL_MAX;
  }
  
  // Apply jerk limit
  if (dt_us > 0) {
    float dt_sec = dt_us / 1000000.0;
    float max_accel_change = ISO_LONG_JERK_MAX * dt_sec;
    
    float min_accel = prev_accel - max_accel_change;
    float max_accel = prev_accel + max_accel_change;
    
    if (accel_cmd > max_accel) {
      accel_cmd = max_accel;
    }
    if (accel_cmd < min_accel) {
      accel_cmd = min_accel;
    }
  }
  
  return accel_cmd;
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
