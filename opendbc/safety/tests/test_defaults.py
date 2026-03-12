#!/usr/bin/env python3
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py


class TestDefaultRxHookBase(common.SafetyTest):
  FWD_BUS_LOOKUP = {}

  def test_rx_hook(self):
    # default rx hook allows all msgs
    for bus in range(4):
      for addr in self.SCANNED_ADDRS:
        self.assertTrue(self._rx(common.make_msg(bus, addr, 8)), f"failed RX {addr=}")


class TestNoOutput(TestDefaultRxHookBase):
  TX_MSGS = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)
    self.safety.init_tests()


class TestSilent(TestNoOutput):
  """SILENT uses same hooks as NOOUTPUT"""

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.silent, 0)
    self.safety.init_tests()


class TestAllOutput(TestDefaultRxHookBase):
  # Allow all messages
  TX_MSGS = [[addr, bus] for addr in common.SafetyTest.SCANNED_ADDRS
             for bus in range(4)]

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.allOutput, 0)
    self.safety.init_tests()

  def test_spam_can_buses(self):
    # asserts tx allowed for all scanned addrs
    for bus in range(4):
      for addr in self.SCANNED_ADDRS:
        should_tx = [addr, bus] in self.TX_MSGS
        self.assertEqual(should_tx, self._tx(common.make_msg(bus, addr, 8)), f"allowed TX {addr=} {bus=}")

  def test_default_controls_not_allowed(self):
    # controls always allowed
    self.assertTrue(self.safety.get_controls_allowed())

  def test_tx_hook_on_wrong_safety_mode(self):
    # No point, since we allow all messages
    pass


class TestAllOutputPassthrough(TestAllOutput):
  FWD_BLACKLISTED_ADDRS = {}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.allOutput, 1)
    self.safety.init_tests()


class TestE2EAEB(common.SafetyTest):
  """
  E2E Phase 3: Tests for End-to-End Automatic Emergency Braking.

  These tests verify that the Panda safety layer allows enhanced braking
  during AEB events while maintaining strict safety bounds to prevent
  catastrophic deceleration requests from the E2E model.
  
  Safety Requirements:
  - Normal operation: braking limited to min_accel (e.g., -3.5 m/s^2)
  - AEB operation: braking limited to emergency_min_accel (e.g., -9.8 m/s^2 = -1g)
  - AEB must never allow acceleration beyond max_accel
  - AEB must prevent physically impossible braking (< -1g)
  """

  # Test limits (in mm/s^2, matching C constants)
  MAX_ACCEL = 200      # 0.2 m/s^2
  MIN_ACCEL = -3500    # -3.5 m/s^2 (normal braking limit)
  EMERGENCY_MIN_ACCEL = -9800  # -9.8 m/s^2 (1g emergency braking limit)
  INACTIVE_ACCEL = 0

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.silent, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    # Define test limits matching typical vehicle parameters
    self.limits = {
      'max_accel': self.MAX_ACCEL,
      'min_accel': self.MIN_ACCEL,
      'emergency_min_accel': self.EMERGENCY_MIN_ACCEL,
      'inactive_accel': self.INACTIVE_ACCEL,
    }

  def test_aeb_override_functions_exist(self):
    """Verify AEB override functions are available."""
    # Test setting AEB override
    self.safety.set_aeb_override(True)
    self.assertTrue(self.safety.get_aeb_override())

    # Test clearing AEB override
    self.safety.set_aeb_override(False)
    self.assertFalse(self.safety.get_aeb_override())

  def test_normal_accel_within_limits(self):
    """Verify normal acceleration commands within limits are allowed."""
    self.safety.set_aeb_override(False)
    
    # Test max acceleration (should be allowed)
    result = self.safety.longitudinal_accel_checks(self.MAX_ACCEL, self.limits)
    self.assertFalse(result, f"Max accel {self.MAX_ACCEL} should be allowed")
    
    # Test moderate braking (should be allowed)
    result = self.safety.longitudinal_accel_checks(-2000, self.limits)
    self.assertFalse(result, "Moderate braking (-2.0 m/s^2) should be allowed")
    
    # Test at min_accel limit (should be allowed)
    result = self.safety.longitudinal_accel_checks(self.MIN_ACCEL, self.limits)
    self.assertFalse(result, f"Min accel {self.MIN_ACCEL} should be allowed")

  def test_normal_accel_violations(self):
    """Verify normal acceleration violations are caught."""
    self.safety.set_aeb_override(False)
    
    # Test exceeding max acceleration (should be rejected)
    result = self.safety.longitudinal_accel_checks(self.MAX_ACCEL + 100, self.limits)
    self.assertTrue(result, f"Exceeding max accel should be rejected")
    
    # Test exceeding min acceleration (harder braking than allowed, should be rejected)
    result = self.safety.longitudinal_accel_checks(self.MIN_ACCEL - 100, self.limits)
    self.assertTrue(result, f"Exceeding min accel (harder braking) should be rejected")
    
    # Test catastrophic braking without AEB (should be rejected)
    result = self.safety.longitudinal_accel_checks(-12000, self.limits)
    self.assertTrue(result, "Catastrophic braking (-12 m/s^2) should be rejected without AEB")

  def test_aeb_enhanced_braking_allowed(self):
    """Verify AEB allows enhanced braking up to emergency limit."""
    self.safety.set_aeb_override(True)
    
    # Test braking beyond normal limit but within emergency limit (should be allowed)
    result = self.safety.longitudinal_accel_checks(-5000, self.limits)
    self.assertFalse(result, "Braking at -5.0 m/s^2 should be allowed during AEB")
    
    result = self.safety.longitudinal_accel_checks(-7000, self.limits)
    self.assertFalse(result, "Braking at -7.0 m/s^2 should be allowed during AEB")
    
    # Test at emergency limit (should be allowed)
    result = self.safety.longitudinal_accel_checks(self.EMERGENCY_MIN_ACCEL, self.limits)
    self.assertFalse(result, f"Emergency braking at {self.EMERGENCY_MIN_ACCEL} (-9.8 m/s^2) should be allowed")

  def test_aeb_prevents_catastrophic_braking(self):
    """CRITICAL: Verify AEB prevents braking beyond emergency limit."""
    self.safety.set_aeb_override(True)
    
    # Test braking beyond emergency limit (should be rejected)
    result = self.safety.longitudinal_accel_checks(-11000, self.limits)
    self.assertTrue(result, "Braking at -11 m/s^2 (>1g) should be REJECTED even during AEB")
    
    # Test extreme catastrophic braking (should be rejected)
    result = self.safety.longitudinal_accel_checks(-20000, self.limits)
    self.assertTrue(result, "Catastrophic braking (-20 m/s^2) should be REJECTED even during AEB")
    
    # Test just beyond emergency limit (should be rejected)
    result = self.safety.longitudinal_accel_checks(self.EMERGENCY_MIN_ACCEL - 100, self.limits)
    self.assertTrue(result, "Braking just beyond emergency limit should be rejected")

  def test_aeb_prevents_acceleration(self):
    """Verify AEB still prevents unintended acceleration during emergency."""
    self.safety.set_aeb_override(True)
    
    # Test exceeding max acceleration during AEB (should be rejected)
    result = self.safety.longitudinal_accel_checks(self.MAX_ACCEL + 100, self.limits)
    self.assertTrue(result, "Exceeding max accel during AEB should be rejected")
    
    # Extreme acceleration should always be rejected
    result = self.safety.longitudinal_accel_checks(5000, self.limits)
    self.assertTrue(result, "Extreme acceleration (5 m/s^2) should be rejected during AEB")

  def test_aeb_boundary_conditions(self):
    """Test boundary conditions at the emergency limit."""
    self.safety.set_aeb_override(True)
    
    # Exactly at emergency limit (should be allowed)
    result = self.safety.longitudinal_accel_checks(self.EMERGENCY_MIN_ACCEL, self.limits)
    self.assertFalse(result, "Exactly at emergency limit should be allowed")
    
    # One unit beyond emergency limit (should be rejected)
    result = self.safety.longitudinal_accel_checks(self.EMERGENCY_MIN_ACCEL - 1, self.limits)
    self.assertTrue(result, "One unit beyond emergency limit should be rejected")
    
    # One unit within emergency limit (should be allowed)
    result = self.safety.longitudinal_accel_checks(self.EMERGENCY_MIN_ACCEL + 1, self.limits)
    self.assertFalse(result, "One unit within emergency limit should be allowed")

  def test_inactive_accel_always_allowed(self):
    """Verify inactive acceleration (0) is always allowed."""
    # Test without AEB
    self.safety.set_aeb_override(False)
    result = self.safety.longitudinal_accel_checks(self.INACTIVE_ACCEL, self.limits)
    self.assertFalse(result, "Inactive accel should be allowed without AEB")
    
    # Test with AEB
    self.safety.set_aeb_override(True)
    result = self.safety.longitudinal_accel_checks(self.INACTIVE_ACCEL, self.limits)
    self.assertFalse(result, "Inactive accel should be allowed with AEB")

  def test_controls_allowed_required(self):
    """Verify longitudinal commands require controls_allowed."""
    self.safety.set_controls_allowed(False)
    self.safety.set_aeb_override(True)
    
    # Even with AEB, controls must be allowed
    result = self.safety.longitudinal_accel_checks(-5000, self.limits)
    self.assertTrue(result, "Longitudinal commands should be rejected when controls not allowed")


if __name__ == "__main__":
  unittest.main()
