#include "opendbc/safety/declarations.h"

// ISO 15622:2018 ACC safety limits
// Maximum allowed acceleration: 2.5 m/s^2 (comfortable)
// Maximum allowed deceleration: -5.0 m/s^2 (emergency braking threshold)
// Maximum allowed jerk: 5.0 m/s^3 (comfort limit)
#define ISO_MAX_ACCEL 250  // 2.5 m/s^2 in cm/s^2
#define ISO_MIN_ACCEL -500 // -5.0 m/s^2 in cm/s^2
#define ISO_MAX_JERK 500   // 5.0 m/s^3 in cm/s^3

// E2E safety floor: limit rate of change to prevent hallucination-induced sudden maneuvers
#define E2E_MAX_ACCEL_RATE 300  // 3.0 m/s^2 per second limit
#define E2E_MIN_ACCEL_RATE -400 // -4.0 m/s^2 per second limit

bool get_longitudinal_allowed(void) {
  return controls_allowed && !gas_pressed_prev;
}

// Safety checks for longitudinal actuation
// Phase 4: Enhanced checks for E2E "black-box" protection
bool longitudinal_accel_checks(int desired_accel, const LongitudinalLimits limits) {
  bool accel_valid = get_longitudinal_allowed() && !safety_max_limit_check(desired_accel, limits.max_accel, limits.min_accel);
  bool accel_inactive = desired_accel == limits.inactive_accel;
  
  // Phase 4: E2E Safety - Enforce ISO acceleration limits regardless of car-specific limits
  // This protects against E2E model hallucinations requesting extreme acceleration
  bool iso_violation = (desired_accel > ISO_MAX_ACCEL) || (desired_accel < ISO_MIN_ACCEL);
  
  // Phase 4: E2E Safety - Rate limiting for acceleration changes
  // Prevents sudden jerks that could be caused by model instability
  int accel_delta = desired_accel - vehicle_accel_prev;
  bool jerk_violation = (accel_delta > E2E_MAX_ACCEL_RATE) || (accel_delta < E2E_MIN_ACCEL_RATE);
  
  // Update previous acceleration for rate limiting
  if (get_longitudinal_allowed() || accel_inactive) {
    vehicle_accel_prev = desired_accel;
  }
  
  return !(accel_valid || accel_inactive) || iso_violation || jerk_violation;
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
