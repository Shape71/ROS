#!/home/brover/ws/.venv/bin/python

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class WheelCommander(Node):
    def __init__(self):
        super().__init__("wheel_commander")
        self.pubs = [self.create_publisher(Float32, f"/node{i}/angular_velocity", 10) for i in range(1, 5)]

    def send_command(self, velocity: float):
        for _ in range(10):
            for pub in self.pubs:
                msg = Float32()
                msg.data = velocity
                pub.publish(msg)
            time.sleep(0.05)
        self.get_logger().info(f"Sent {velocity} to all wheels")


def main():
    rclpy.init()
    node = WheelCommander()

    try:
        node.send_command(1.0)
        time.sleep(2.0)

        node.send_command(0.0)
        time.sleep(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()
