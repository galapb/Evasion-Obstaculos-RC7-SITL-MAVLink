import time
import logging
from pymavlink import mavutil
import threading


class MavlinkMasterDroneWriterInterface:

    def __init__(self, state, connection):
        self.lockSend = threading.Lock()
        self.state_ = state
        self.logger = logging.getLogger("MavLink Writer Interface")
        self.mavlinkConnection_ = connection
        self.current_pitch_ = 0.0  # Current pitch angle in degrees.
        self.current_yaw_ = 0.0  # Current yaw angle in degrees.
        self.max_rate_pitch_ = 10.0
        self.max_rate_yaw_ = 10.0

    def set_gimbal_speed(self, joystick_azimuth, joystick_elevation, dt):
        with self.lockSend:
            if not self.state_.control_.masterHearthBeatReceived_:
                return
            delta_pitch = joystick_elevation * self.max_rate_pitch_ * dt
            delta_yaw = joystick_azimuth * self.max_rate_yaw_ * dt
            mount_mode = 2

            self.current_pitch_ += delta_pitch
            self.current_yaw_ += delta_yaw

            self.mavlinkConnection_.mav.command_long_send(
                self.mavlinkConnection_.target_system,  # target_system
                self.mavlinkConnection_.target_component,  # target_component
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,  # command
                0,  # confirmation
                self.current_pitch_,  # param1: pitch angle in degrees
                0,  # param2: roll angle (set to 0)
                self.current_yaw_,  # param3: yaw angle in degrees
                0, 0, 0,  # param4, param5, param6 (unused)
                mount_mode  # param7: mount mode (typically 2)
            )

    def SendRCChannelPWM(self, deploy1, deploy2, camera, pitch, yaw):
        with self.lockSend:
            if not self.state_.control_.masterHearthBeatReceived_:
                return
            self.mavlinkConnection_.mav.rc_channels_override_send(
                self.mavlinkConnection_.target_system,
                self.mavlinkConnection_.target_component,
                0, 0, 0, 0, 0,
                deploy1, deploy2, 0, pitch, 0, yaw, camera, 0, 0, 0, 0
            )
            # self.logger.info(
            #     f"Sent MAVLink PWM: deploy1={deploy1}, deploy2={deploy2}, camera={camera}, pitch={pitch}, yaw={yaw}")

    def HandleVfrHud(self, msg):
        with self.lockSend:
            self.state_.masterDroneTelemetry_.groundspeed_ = msg.groundspeed
            self.state_.masterDroneTelemetry_.altitude_ = msg.alt
            self.state_.masterDroneTelemetry_.heading_ = msg.heading
            self.logger.info(f"Received MAVLink VFR_HUD: Altitude={msg.alt}, Heading={msg.heading}")

    def SetServoValue(self, channel, value):
        with self.lockSend:
            if not self.state_.control_.masterHearthBeatReceived_:
                return
            self.mavlinkConnection_.mav.command_long_send(
                self.mavlinkConnection_.target_system,
                self.mavlinkConnection_.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                channel, value, 0, 0, 0, 0, 0
            )
            self.logger.info(f"Sent MAVLink SET SERVO: channel={channel}, value={value}")
