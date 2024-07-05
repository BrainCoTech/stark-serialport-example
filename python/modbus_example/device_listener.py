from modbus_client_utils import *

class DeviceListener(StarkDeviceListener):    
  def on_error(self, error):
    SKLog.error(f"Error: {error.message}")

  def on_serialport_cfg(self, cfg):
    SKLog.info(f"Serialport cfg: {cfg}")

  def on_hand_type(self, hand_type):
    SKLog.info(f"Hand type: {hand_type}")

  def on_force_level(self, force_level):
    SKLog.info(f"Force level: {force_level}")

  def on_motorboard_info(self, info):
    SKLog.info(f"Motorboard info: {info}")  

  def on_voltage(self, voltage):
    SKLog.info(f"Voltage: {voltage}")

  def on_button_event(self, event):
    SKLog.info(f"Button event: {event}")

  def on_led_info(self, led_info):
    SKLog.info(f"Led info: {led_info}")

  def on_turbo_mode(self, turbo_mode):
    SKLog.info(f"in Turbo mode: {turbo_mode}")

  def on_turbo_conf(self, turbo_interval, turbo_duration):
    SKLog.info(f"Turbo conf: {turbo_interval}, {turbo_duration}")

  def on_auto_calibration(self, auto_calibration_enabled):
    SKLog.info(f"Auto calibration: {auto_calibration_enabled}")

  def on_finger_currents(self, finger_currents):
    SKLog.info(f"Finger currents: {finger_currents}")  

  def on_finger_positions(self, finger_positions):
    SKLog.info(f"Finger positions: {finger_positions}")

  def on_finger_speeds(self, finger_speeds):
    SKLog.info(f"Finger speeds: {finger_speeds}")

  def on_motor_status(self, statuses):
    SKLog.info(f"Motor status: {statuses}")  

  def on_action_sequence(self, action_sequence):
    SKLog.info(f"Action sequence: {action_sequence}")

  def on_stall_durations(self, stall_durations):
    SKLog.info(f"Stall durations: {stall_durations}")
  
  def on_stall_currents(self, stall_currents):
    SKLog.info(f"Stall currents: {stall_currents}")
  
  def on_finger_pwms(self, finger_pwms):
    SKLog.info(f"Finger pwms: {finger_pwms}")
      