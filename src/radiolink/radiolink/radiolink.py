from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from cyphal_ros2_bridge.msg import HMIBeeper, HMILed


USE_INDICATORS = False
THRESHOLD = 0.2


@dataclass
class GearConfig:
    max_linear: float  # m/s
    max_angular: float  # rad/s


class Gears(Enum):
    FIRST = GearConfig(0.1, 0.07)
    SECOND = GearConfig(0.2, 0.15)
    THIRD = GearConfig(0.5, 0.5)


GEARS_LIST = list(Gears)


class GyrobroRadiolink(Node):
    def __init__(self):
        super().__init__("radiolink")
        self._is_on = False
        self.gear = Gears.FIRST
        self.last_gear_signal = None
        self.last_reset_signal = None
        self.last_mode_signal = None
        self.pub_to_cmd_vel = None
        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)

        self.hmi_led = self.create_publisher(HMILed, "/hmi/led", 2)
        self.hmi_beeper = self.create_publisher(HMIBeeper, "/hmi/beeper", 2)

    @property
    def max_linear_speed(self):
        return self.gear.value.max_linear

    @property
    def max_angular_speed(self):
        return self.gear.value.max_angular

    @property
    def is_on(self) -> bool:
        return self._is_on

    @is_on.setter
    def is_on(self, value):
        if isinstance(value, bool):
            self._is_on = value
        old_is_on = self._is_on
        self._is_on = value == 2
        if self._is_on != old_is_on:
            self.send_indicators()

    def send_indicators(self):
        if not USE_INDICATORS:
            return
        if self._is_on:
            self.hmi_led.publish(HMILed(r=0, g=255, b=0))
            self.hmi_beeper.publish(HMIBeeper(duration=1, frequency=10))
        else:
            self.hmi_led.publish(HMILed(r=0, g=0, b=255))
            self.hmi_beeper.publish(HMIBeeper(duration=1, frequency=1))

    # def set(self, gear, mode):
    #     if gear != 0:
    #         self.gear = GEARS_LIST[gear - 1]
    #     if mode != 0:
    #         self.is_on = mode

    # @staticmethod
    # def parse_signal(signal, intervals=(1, 2, 3)):
    #     if signal is None:
    #         return 0
    #     res_sig = 1
    #     if -0.01 < signal < 0.01:
    #         res_sig = intervals[1]
    #     elif signal < 0:
    #         res_sig = intervals[0]
    #     else:
    #         res_sig = intervals[2]
    #     return res_sig

    def send_reset(self):
        # TODO
        print("send_reset !!")

    # def process_state(self, gear_signal, mode_signal):
    #     gear = self.parse_signal(gear_signal)
    #     mode = self.parse_signal(mode_signal, (1, 1, 2))
    #     self.set(gear=gear, mode=mode)

    def joy_callback(self, data: Joy):
        # right bottom switch at lower pose
        # self.pub_to_cmd_vel = data.axes[4] > 0.5
        self.pub_to_cmd_vel = 1

        should_update_state = False

        # gear_signal = data.axes[6]
        # mode_signal = data.axes[4]
        reset_signal = data.axes[5]
        #
        # if gear_signal != self.last_gear_signal:
        #     self.last_gear_signal = gear_signal
        #     should_update_state = True
        # else:
        #     gear_signal = None
        #
        # if mode_signal != self.last_mode_signal:
        #     self.last_mode_signal = mode_signal
        #     should_update_state = True
        # else:
        #     mode_signal = None
        #
        # if should_update_state:
        #     self.process_state(gear_signal, mode_signal)

        if reset_signal != self.last_reset_signal:
            self.last_reset_signal = reset_signal
            if reset_signal < 0:
                self.send_reset()

        cmd = TwistStamped()
        cmd.header.frame_id = "base_link"
        sec, nanosec = self.get_clock().now().seconds_nanoseconds()
        cmd.header.stamp.nanosec = nanosec
        cmd.header.stamp.sec = sec
        # Getting linear and angular speed from joystick
        if abs(data.axes[1]) > THRESHOLD:
            if abs(data.axes[2])>0.9:
                data.axes[2] = 1 if data.axes[2]>0.9 else -1
            cmd.twist.linear.x = (data.axes[2]+1)*0.5 if data.axes[1]>0 else -(data.axes[2]+1)*0.5
        else:
            cmd.twist.linear.x = 0.0
        if abs(data.axes[0]) > THRESHOLD:
            if abs(data.axes[3])>0.9:
                data.axes[3] = 1 if data.axes[3]>0.9 else -1
            cmd.twist.angular.z = (data.axes[3]+1)*0.5 if data.axes[0]>0 else -(data.axes[3]+1)*0.5
        else:
            cmd.twist.angular.z = 0.0
        if self.pub_to_cmd_vel:
            self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    radiolink = GyrobroRadiolink()
    rclpy.spin(radiolink)
    radiolink.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
