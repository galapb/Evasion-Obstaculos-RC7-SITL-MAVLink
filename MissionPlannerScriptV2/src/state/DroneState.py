"""
Module: DroneState.py
Description: Defines the DroneState class and its sub-classes to hold the drone's state.
"""

class ControlState:
    """Holds control-related state data."""
    def __init__(self):
        self.arm_ = False
        self.deploy1_ = False
        self.deploy2_ = False
        self.activeControl_ = 0
        self.activeControlPrev_ = 0
        self.metaCamera_ = 0
        self.metaCommand1_ = 0
        self.metaCommand2_ = 0
        self.gcsCamera_ = 0
        self.gcsCommand1_ = 0
        self.gcsCommand2_ = 0
        self.camControl_ = 0
        self.nnToggle_ = 0
        self.nn_ = 0
        self.smartDeploy_ = 0
        self.auxDroneOnMission_=False
        self.masterHearthBeatReceived_=False
        self.auxHearthBeatReceived_=False

class RCChannelsState:
    """Holds RC channel data."""
    def __init__(self):
        self.rcChannelPitch_ = 13
        self.rcChannelYaw_ = 14
        self.rcChannelCam_ = 12
        self.rcChannelDeploy1_ = 6
        self.rcChannelDeploy2_ = 7
        self.deploy12Value_ = 0
        self.deploy34Value_ = 0
        self.safetyValue_ = 0
        self.cameraValue_ = 5
        self.investigationValue_ = 0

class TelemetryState:
    """Holds telemetry data."""
    def __init__(self):
        self.pitch_ = 0
        self.roll_ = 0
        self.yaw_ = 0
        self.groundspeed_ = 0
        self.verticalSpeed_ = 0
        self.latitude_ = 0
        self.longitude_ = 0
        self.altitude_ = 0
        self.heading_ = 0
        self.accelZ_ = 0

class BatteryState:
    """Holds battery information."""
    def __init__(self):
        self.voltageValue_ = 0
        self.currentValue_ = 0
        self.batteryRemaining_ = 0

class GimbalState:
    """Holds gimbal state data."""
    def __init__(self):
        self.gimbalGain_ = 25.1
        self.gimbalPitchNeutral_ = 1825
        self.gimbalYawNeutral_ = 1675
        self.pwmPitchMin_ = 1010
        self.pwmPitchMax_ = 2018
        self.pwmYawMin_ = 1010
        self.pwmYawMax_ = 2018
        self.pwmPitch_ = 0
        self.pwmYaw_ = 0
        self.pitchPrev_ = 0.0
        self.yawPrev_ = 0.0
        self.cameraPrev_ = 7
        self.deploy12_ = 0
        self.deploy34_ = 0
        self.deploy12Prev_ = 0
        self.deploy34Prev_ = 0

class JoystickState:
    """Holds joystick state data."""
    def __init__(self):
        self.joystickButtonPrev_ = 1
        self.joystickX_ = 0.0
        self.joystickY_ = 0.0
        self.joystickZ_ = 0.0
        self.joystickButton_ = 0
        self.temperature_ = 0.0
        self.exponentialEnabled = False

class AuxiliaryDroneState:
    """Holds auxiliary drone control variables"""
    def __init__(self):
        self.auxIsInvestigating = False

class DroneState:
    """Aggregates all state information for the drone."""
    def __init__(self):
        self.id_ = 13
        self.control_ = ControlState()
        self.rc_channels_ = RCChannelsState()
        self.masterDroneTelemetry_ = TelemetryState()
        self.auxDroneTelemetry_ = TelemetryState()
        self.battery_ = BatteryState()
        self.gimbal_ = GimbalState()
        self.joystick_ = JoystickState()
        self.auxDroneState_ = AuxiliaryDroneState()