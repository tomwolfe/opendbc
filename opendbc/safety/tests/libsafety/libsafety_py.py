import os
from cffi import FFI

from opendbc.safety import LEN_TO_DLC

libsafety_dir = os.path.dirname(os.path.abspath(__file__))
libsafety_fn = os.path.join(libsafety_dir, "libsafety.so")

ffi = FFI()

ffi.cdef("""
typedef struct {
  unsigned char fd : 1;
  unsigned char bus : 3;
  unsigned char data_len_code : 4;
  unsigned char rejected : 1;
  unsigned char returned : 1;
  unsigned char extended : 1;
  unsigned int addr : 29;
  unsigned char checksum;
  unsigned char data[64];
} CANPacket_t;
""", packed=True)
class CANPacket:
  pass

ffi.cdef("""
bool safety_rx_hook(CANPacket_t *msg);
bool safety_tx_hook(CANPacket_t *msg);
int safety_fwd_hook(int bus_num, int addr);
int set_safety_hooks(uint16_t mode, uint16_t param);

void set_controls_allowed(bool c);
bool get_controls_allowed(void);
bool get_longitudinal_allowed(void);
void set_alternative_experience(int mode);
int get_alternative_experience(void);
void set_relay_malfunction(bool c);
bool get_relay_malfunction(void);
bool get_gas_pressed_prev(void);
void set_gas_pressed_prev(bool);
bool get_brake_pressed_prev(void);
bool get_regen_braking_prev(void);
bool get_steering_disengage_prev(void);
bool get_acc_main_on(void);
float get_vehicle_speed_min(void);
float get_vehicle_speed_max(void);
int get_current_safety_mode(void);
int get_current_safety_param(void);

void set_torque_meas(int min, int max);
int get_torque_meas_min(void);
int get_torque_meas_max(void);
void set_torque_driver(int min, int max);
int get_torque_driver_min(void);
int get_torque_driver_max(void);
void set_desired_torque_last(int t);
void set_rt_torque_last(int t);
void set_desired_angle_last(int t);
int get_desired_angle_last();

// E2E Phase 3: AEB override functions
void set_aeb_override(bool override);
bool get_aeb_override(void);
void set_angle_meas(int min, int max);
int get_angle_meas_min(void);
int get_angle_meas_max(void);

bool get_cruise_engaged_prev(void);
void set_cruise_engaged_prev(bool engaged);
bool get_vehicle_moving(void);
void set_timer(uint32_t t);

void safety_tick_current_safety_config();
bool safety_config_valid();

void init_tests(void);

void set_honda_fwd_brake(bool c);
bool get_honda_fwd_brake(void);
void set_honda_alt_brake_msg(bool c);
void set_honda_bosch_long(bool c);
int get_honda_hw(void);

// E2E Phase 3: Longitudinal safety check wrappers for testing
typedef struct {
  int max_accel;
  int min_accel;
  int inactive_accel;
  int emergency_min_accel;
  int max_gas;
  int min_gas;
  int inactive_gas;
  int max_brake;
  int max_transmission_rpm;
  int min_transmission_rpm;
  int inactive_transmission_rpm;
  int inactive_speed;
} TestLongitudinalLimits;

bool longitudinal_accel_checks_wrapper(int desired_accel, TestLongitudinalLimits test_limits);
""")

class LibSafety:
  def longitudinal_accel_checks(self, desired_accel, limits):
    """Wrapper for longitudinal_accel_checks that accepts a dict of limits."""
    test_limits = ffi.new('TestLongitudinalLimits *')
    test_limits[0].max_accel = limits['max_accel']
    test_limits[0].min_accel = limits['min_accel']
    test_limits[0].inactive_accel = limits['inactive_accel']
    test_limits[0].emergency_min_accel = limits['emergency_min_accel']
    test_limits[0].max_gas = limits.get('max_gas', 0)
    test_limits[0].min_gas = limits.get('min_gas', 0)
    test_limits[0].inactive_gas = limits.get('inactive_gas', 0)
    test_limits[0].max_brake = limits.get('max_brake', 0)
    test_limits[0].max_transmission_rpm = limits.get('max_transmission_rpm', 0)
    test_limits[0].min_transmission_rpm = limits.get('min_transmission_rpm', 0)
    test_limits[0].inactive_transmission_rpm = limits.get('inactive_transmission_rpm', 0)
    test_limits[0].inactive_speed = limits.get('inactive_speed', 0)
    return self.longitudinal_accel_checks_wrapper(desired_accel, test_limits[0])

libsafety: LibSafety = ffi.dlopen(libsafety_fn)

def make_CANPacket(addr: int, bus: int, dat):
  ret = ffi.new('CANPacket_t *')
  ret[0].extended = 1 if addr >= 0x800 else 0
  ret[0].addr = addr
  ret[0].data_len_code = LEN_TO_DLC[len(dat)]
  ret[0].bus = bus
  ret[0].data = bytes(dat)
  return ret
