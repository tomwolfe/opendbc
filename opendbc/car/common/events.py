"""
Common event logic for all car brands.

This module contains the logic for generating common vehicle events/alerts
that apply to all brands. Brand-specific events should be implemented in
the respective brand's CarInterface.get_events() method.
"""

from opendbc.car import DT_CTRL, structs
from opendbc.car.interfaces import MAX_CTRL_SPEED

GearShifter = structs.CarState.GearShifter
ButtonType = structs.CarState.ButtonEvent.Type


class CarEventProcessor:
  """
  Processes car state and generates common events.
  
  This class handles all the common event logic that was previously
  in openpilot's CarSpecificEvents class. Brand-specific event logic
  should be implemented in each brand's CarInterface.get_events() method.
  """
  
  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.steering_unpressed = 0
    self.no_steer_warning = False
    self.silent_steer_warning = True
    self.low_speed_alert = False

  def update(self, CS: structs.CarState, CS_prev: structs.CarState,
             CC: structs.CarControl, brand_events: list[str]) -> list[str]:
    """
    Update and return all events (common + brand-specific).
    
    Args:
      CS: Current CarState
      CS_prev: Previous CarState
      CC: Current CarControl
      brand_events: List of brand-specific event names from CarInterface.get_events()
    
    Returns:
      List of all event names (strings)
    """
    if self.CP.brand in ('body', 'mock'):
      return []
    
    events = self.create_common_events(CS, CS_prev, CC)
    events.extend(brand_events)
    return events

  def create_common_events(self, CS: structs.CarState, CS_prev: structs.CarState,
                           CC: structs.CarControl) -> list[str]:
    """
    Create common events that apply to all car brands.
    
    Returns:
      List of event names (strings)
    """
    events = []

    # TODO: cleanup the honda-specific logic - move to honda interface
    pcm_enable = self.CP.pcmCruise and self.CP.brand != 'honda'
    # TODO: on some hyundai cars, the cancel button is also the pause/resume button,
    # so only use it for cancel when running openpilot longitudinal
    allow_button_cancel = self.CP.brand != 'hyundai'

    if CS.doorOpen:
      events.append("doorOpen")
    if CS.seatbeltUnlatched:
      events.append("seatbeltNotLatched")
    if CS.gearShifter != GearShifter.drive and CS.gearShifter not in self._get_drivable_gears():
      events.append("wrongGear")
    if CS.gearShifter == GearShifter.reverse:
      events.append("reverseGear")
    if not CS.cruiseState.available:
      events.append("wrongCarMode")
    if CS.espDisabled:
      events.append("espDisabled")
    if CS.espActive:
      events.append("espActive")
    if CS.stockFcw:
      events.append("stockFcw")
    if CS.stockAeb:
      events.append("stockAeb")
    if CS.stockLkas:
      events.append("stockLkas")
    if CS.vEgo > MAX_CTRL_SPEED:
      events.append("speedTooHigh")
    if CS.cruiseState.nonAdaptive:
      events.append("wrongCruiseMode")
    if CS.brakeHoldActive and self.CP.openpilotLongitudinalControl:
      events.append("brakeHold")
    if CS.parkingBrake:
      events.append("parkBrake")
    if CS.accFaulted:
      events.append("accFaulted")
    if CS.steeringPressed:
      events.append("steerOverride")
    if CS.steeringDisengage and not CS_prev.steeringDisengage:
      events.append("steerDisengage")
    if CS.brakePressed and CS.standstill:
      events.append("preEnableStandstill")
    if CS.gasPressed:
      events.append("gasPressedOverride")
    if CS.vehicleSensorsInvalid:
      events.append("vehicleSensorsInvalid")
    if CS.invalidLkasSetting:
      events.append("invalidLkasSetting")
    if CS.lowSpeedAlert:
      events.append("belowSteerSpeed")
    if CS.buttonEnable:
      events.append("buttonEnable")

    # Handle cancel button presses
    for b in CS.buttonEvents:
      # Disable on rising and falling edge of cancel for both stock and OP long
      # TODO: only check the cancel button with openpilot longitudinal on all brands to match panda safety
      if b.type == ButtonType.cancel and (allow_button_cancel or not self.CP.pcmCruise):
        events.append("buttonCancel")

    # Handle permanent and temporary steering faults
    self.steering_unpressed = 0 if CS.steeringPressed else self.steering_unpressed + 1
    if CS.steerFaultTemporary:
      if CS.steeringPressed and (not CS_prev.steerFaultTemporary or self.no_steer_warning):
        self.no_steer_warning = True
      else:
        self.no_steer_warning = False

        # if the user overrode recently, show a less harsh alert
        if self.silent_steer_warning or CS.standstill or self.steering_unpressed < int(1.5 / DT_CTRL):
          self.silent_steer_warning = True
          events.append("steerTempUnavailableSilent")
        else:
          events.append("steerTempUnavailable")
    else:
      self.no_steer_warning = False
      self.silent_steer_warning = False
    if CS.steerFaultPermanent:
      events.append("steerUnavailable")

    # we engage when pcm is active (rising edge)
    # enabling can optionally be blocked by the car interface
    if pcm_enable:
      if CS.cruiseState.enabled and not CS_prev.cruiseState.enabled and not CS.blockPcmEnable:
        events.append("pcmEnable")
      elif not CS.cruiseState.enabled:
        events.append("pcmDisable")

    return events

  def _get_drivable_gears(self) -> tuple:
    """Get drivable gears for the current car."""
    from opendbc.car.car_helpers import interfaces
    CI = interfaces.get(self.CP.carFingerprint)
    if CI and hasattr(CI, 'DRIVABLE_GEARS'):
      return CI.DRIVABLE_GEARS
    return ()
