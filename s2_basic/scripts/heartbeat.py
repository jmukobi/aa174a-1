#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64, Bool

class Heartbeat(Node):
    def __init__(self) -> None:
        super().__init__("heartbeat")

        self.hb_pub = self.create_publisher(Int64, "/heartbeat", 10)
        self.hb_timer = self.create_timer(1.0, self.hb_callback)
        self.motor_sub = self.create_subscription(Bool, "/health/motor",
                                                  self.health_callback, 10)
        self.hb_cnt = 0

    def hb_callback(self) -> None:
        msg = Int64()
        msg.data = self.hb_cnt
        self.hb_pub.publish(msg)

        self.hb_cnt += 1

    def health_callback(self, msg: Bool) -> None:
        """ sensor health callback triggered by subscription """
        if not msg.data:
            self.get_logger().fatal("Heartbeat stopped")
            self.hb_timer.cancel()

if __name__ == "__main__":
    rclpy.init()
    node = Heartbeat()
    rclpy.spin(node)
    rclpy.shutdown()
