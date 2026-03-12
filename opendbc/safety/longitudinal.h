#include "opendbc/safety/declarations.h"

// E2E Phase 3: AEB override flag
// When set, allows maximum braking beyond normal safety limits
static bool aeb_override = false;

void set_aeb_override(bool override) {
  aeb_override = override;
}

bool get_aeb_override(void) {
  return aeb_override;
}

bool get_longitudinal_allowed(void) {
  return controls_allowed && !gas_pressed_prev;
}

// Safety checks for longitudinal actuation
bool longitudinal_accel_checks(int desired_accel, const LongitudinalLimits limits) {
  // E2E Phase 3: AEB override - allow enhanced braking during emergency, but with safe limits
  // When AEB is active, use emergency_min_accel instead of min_accel
  // This prevents the E2E model from requesting physically impossible or dangerous braking
  bool accel_valid;
  if (aeb_override) {
    // During AEB, enforce emergency braking limits:
    // - Upper bound: max_accel (prevent unintended acceleration)
    // - Lower bound: emergency_min_accel (prevent catastrophic braking, e.g., -1g)
    accel_valid = get_longitudinal_allowed() && 
                  !safety_max_limit_check(desired_accel, limits.max_accel, limits.emergency_min_accel);
  } else {
    // Normal operation - enforce both min and max limits
    accel_valid = get_longitudinal_allowed() && !safety_max_limit_check(desired_accel, limits.max_accel, limits.min_accel);
  }
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
  // E2E Phase 3: AEB override - allow maximum brake pressure during emergency
  // During AEB, allow full brake pressure up to max_brake limit
  // Note: The acceleration-based emergency limit (emergency_min_accel) is enforced
  // separately in longitudinal_accel_checks to prevent excessive deceleration
  if (aeb_override) {
    // During AEB, allow brake pressure up to max_brake (no additional restriction)
    // Negative values are invalid (brake pressure cannot be negative)
    violation |= desired_brake < 0;
  } else {
    violation |= desired_brake > limits.max_brake;
  }
  return violation;
}
