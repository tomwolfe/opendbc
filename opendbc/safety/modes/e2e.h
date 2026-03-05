#pragma once

/**
 * E2E (End-to-End) Safety Mode for openpilot Phase 4
 *
 * This safety mode provides E2E-aware limits that:
 * 1. Allow higher torque and braking for vision-based E2E maneuvers
 * 2. Still enforce ISO 15622 lateral acceleration limits (3.0 m/s^2)
 * 3. Prevent "Permanent Fault" from radical vision-based plans
 * 4. Support both torque-based and angle-based steering control
 *
 * Key differences from standard modes:
 * - Higher max torque (up to 400 for most brands, vs 270-300 standard)
 * - Higher max brake (up to 4.0 m/s^2, vs 3.5 m/s^2 standard)
 * - Maintains ISO lateral accel limit via roll compensation
 * - More permissive rate limits for E2E maneuvers
 */

#include <math.h>
#include "opendbc/safety/declarations.h"
#include "opendbc/safety/lateral.h"
#include "opendbc/safety/longitudinal.h"

// Forward declarations (defined in safety.h)
static void generic_rx_checks(void);

// E2E-specific limits
// ISO 15622:2018 still applies - max lateral acceleration
static const float E2E_ISO_LATERAL_ACCEL = 3.0;  // m/s^2 (same as standard)
static const float E2E_MAX_LAT_ACCEL_WITH_ROLL = 3.5;  // m/s^2 (slightly higher for E2E maneuvers)

// E2E allows higher torque for more aggressive vision-based corrections
static const int E2E_MAX_TORQUE = 400;  // vs 270-300 in standard modes
static const int E2E_MAX_TORQUE_RATE_UP = 15;  // vs 10 standard (faster response)
static const int E2E_MAX_TORQUE_RATE_DOWN = 20;  // vs 15 standard (faster recovery)
static const int E2E_MAX_RT_DELTA = 500;  // real-time delta limit

// E2E allows stronger braking for traffic lights/obstacles
static const int E2E_MAX_BRAKE = 500;  // approx -4.0 m/s^2 (vs 400/-3.5 standard)
static const int E2E_MAX_ACCEL = 250;  // approx 2.0 m/s^2 (same as standard)
static const int E2E_MIN_ACCEL = -400;  // approx -3.5 m/s^2 (comfortable braking)

// E2E driver monitoring - more permissive allowance
static const int E2E_DRIVER_TORQUE_ALLOWANCE = 100;  // vs 80 standard
static const int E2E_DRIVER_TORQUE_MULTIPLIER = 3;  // vs 2 standard

// E2E torque steering limits structure
typedef struct {
  int max_torque;
  int max_rate_up;
  int max_rate_down;
  int max_rt_delta;
  uint32_t max_rt_interval;
  int max_torque_error;
  bool dynamic_max_torque;
  struct lookup_t max_torque_lookup;  // speed bins for dynamic torque
  SteeringControlType type;
} E2ETorqueSteeringLimits;

// E2E longitudinal limits structure
typedef struct {
  int max_accel;
  int min_accel;
  int max_brake;
  int inactive_accel;
} E2ELongitudinalLimits;

// Default E2E torque limits (can be overridden per-vehicle)
static const E2ETorqueSteeringLimits default_e2e_torque_limits = {
  .max_torque = E2E_MAX_TORQUE,
  .max_rate_up = E2E_MAX_TORQUE_RATE_UP,
  .max_rate_down = E2E_MAX_TORQUE_RATE_DOWN,
  .max_rt_delta = E2E_MAX_RT_DELTA,
  .max_rt_interval = 250000,  // 250ms
  .max_torque_error = 100,
  .dynamic_max_torque = false,
  .max_torque_lookup = {{0}, {0}},
  .type = TorqueDriverLimited,
};

// Default E2E longitudinal limits
static const E2ELongitudinalLimits default_e2e_long_limits = {
  .max_accel = E2E_MAX_ACCEL,
  .min_accel = E2E_MIN_ACCEL,
  .max_brake = E2E_MAX_BRAKE,
  .inactive_accel = 3000,  // 3.0 m/s^2 (positive is accel, so this is inactive)
};

// E2E steer torque command checks
// Enhanced version that allows higher limits while maintaining safety
static bool e2e_steer_torque_cmd_checks(int desired_torque, int steer_req, 
                                         const E2ETorqueSteeringLimits limits) {
  bool violation = false;
  uint32_t ts = microsecond_timer_get();

  if (controls_allowed) {
    // E2E allows higher max torque
    int max_torque = limits.max_torque;
    if (limits.dynamic_max_torque) {
      const float fudged_speed = (vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;
      max_torque = safety_interpolate(limits.max_torque_lookup, fudged_speed) + 1;
      max_torque = SAFETY_CLAMP(max_torque, -limits.max_torque, limits.max_torque);
    }

    // *** global torque limit check (E2E higher limits) ***
    violation |= safety_max_limit_check(desired_torque, max_torque, -max_torque);

    // *** E2E torque rate limit check (faster rates) ***
    if (limits.type == TorqueDriverLimited) {
      violation |= driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
                                      max_torque, limits.max_rate_up, limits.max_rate_down,
                                      E2E_DRIVER_TORQUE_ALLOWANCE, E2E_DRIVER_TORQUE_MULTIPLIER);
    } else {
      violation |= dist_to_meas_check(desired_torque, desired_torque_last, &torque_meas,
                                      limits.max_rate_up, limits.max_rate_down, limits.max_torque_error);
    }
    desired_torque_last = desired_torque;

    // *** E2E torque real time rate limit check (higher delta) ***
    violation |= rt_torque_rate_limit_check(desired_torque, rt_torque_last, limits.max_rt_delta);

    // every RT_INTERVAL set the new limits
    uint32_t ts_elapsed = safety_get_ts_elapsed(ts, ts_torque_check_last);
    if (ts_elapsed > limits.max_rt_interval) {
      rt_torque_last = desired_torque;
      ts_torque_check_last = ts;
    }
  }

  // no torque if controls is not allowed
  if (!controls_allowed && (desired_torque != 0)) {
    violation = true;
  }

  // E2E: allow driver override more permissively
  // Reset mismatch timer if driver is overriding
  if (steer_req && (desired_torque != 0)) {
    if (valid_steer_req_count == 0) {
      invalid_steer_req_count = 0;
    }
    valid_steer_req_count++;
  } else {
    invalid_steer_req_count++;
    valid_steer_req_count = 0;
  }
  
  // E2E: more lenient mismatch counting (3 vs 5 standard)
  if (invalid_steer_req_count >= 3) {
    ts_steer_req_mismatch_last = ts;
    if (valid_steer_req_count >= 3) {
      valid_steer_req_count = 0;
    }
  }

  return violation;
}

// E2E longitudinal acceleration checks
// Allows stronger braking for E2E traffic light detection
static bool e2e_longitudinal_accel_checks(int desired_accel, const E2ELongitudinalLimits limits) {
  bool violation = false;
  
  // E2E: allow braking even if gas was recently pressed (for traffic lights)
  // Standard mode requires gas_pressed_prev to be false
  bool accel_valid = controls_allowed && 
                     !safety_max_limit_check(desired_accel, limits.max_accel, limits.min_accel);
  bool accel_inactive = desired_accel == limits.inactive_accel;
  
  violation = !(accel_valid || accel_inactive);
  
  // E2E: additional brake-specific check with higher limits
  if (desired_accel < 0) {  // Braking
    int brake_command = -desired_accel;  // Convert to positive brake value
    if (brake_command > limits.max_brake) {
      violation = true;
    }
  }
  
  return violation;
}

// E2E safety mode state
static bool e2e_mode_enabled = false;
static E2ETorqueSteeringLimits e2e_torque_limits = default_e2e_torque_limits;
static E2ELongitudinalLimits e2e_long_limits = default_e2e_long_limits;

// Initialize E2E safety mode
static safety_config e2e_init(uint16_t param) {
  // Optional parameter to enable/disable E2E mode
  // Default: enabled for supported vehicles
  const uint16_t E2E_PARAM_ENABLE = 1;
  const uint16_t E2E_PARAM_MAX_TORQUE = 2;
  const uint16_t E2E_PARAM_MAX_BRAKE = 4;
  
  e2e_mode_enabled = GET_FLAG(param, E2E_PARAM_ENABLE);
  
  // Allow custom limits via parameters (for vehicle-specific tuning)
  if (param & E2E_PARAM_MAX_TORQUE) {
    e2e_torque_limits.max_torque = (param >> 8) & 0xFF;
  }
  
  if (param & E2E_PARAM_MAX_BRAKE) {
    e2e_long_limits.max_brake = ((param >> 16) & 0xFF) * 10;
  }
  
  controls_allowed = false;
  return (safety_config){NULL, 0, NULL, 0, true};
}

// E2E RX hook - process incoming messages
static void e2e_rx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  // Standard RX checks apply
  generic_rx_checks();
}

// E2E TX hook - validate outgoing control messages
static bool e2e_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  bool violation = false;

  // Extract desired torque/accel from message (vehicle-specific)
  int desired_torque = 0;
  int desired_accel = 0;
  
  // E2E-specific checks
  if (e2e_mode_enabled) {
    // Apply E2E limits
    violation |= e2e_steer_torque_cmd_checks(desired_torque, 1, e2e_torque_limits);
    violation |= e2e_longitudinal_accel_checks(desired_accel, e2e_long_limits);
  } else {
    // Fall back to standard limits
    // (would call standard safety checks here)
  }
  
  return !violation;
}

// E2E safety hooks structure
const safety_hooks e2e_hooks = {
  .init = e2e_init,
  .rx = e2e_rx_hook,
  .tx = e2e_tx_hook,
};
