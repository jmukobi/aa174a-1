#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64, Bool
from geometry_msgs.msg import Twist

class Heartbeat(Node):
    def __init__(self) -> None:
        super().__init__("heartbeat")

        self.hb_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.hb_timer = self.create_timer(1.0, self.hb_callback)

        self.twist_sub = self.create_subscription(Bool, "/kill",
                                                  self.kill_callback, 10)
        self.hb_cnt = 0.0

    def hb_callback(self) -> None:
        msg = Twist()
        msg.angular.x = self.hb_cnt
        msg.angular.z = self.hb_cnt

        self.hb_pub.publish(msg)

        self.hb_cnt += 1.0

    def kill_callback(self, msg: Bool) -> None:
        print("called")
        self.get_logger().fatal("Control stopped")
        print("control stopped")
        self.hb_timer.cancel()

        msg = Twist()
        msg.angular.x = 0.0
        msg.angular.z = 0.0

        self.hb_pub.publish(msg)
            

if __name__ == "__main__":
    rclpy.init()
    node = Heartbeat()
    rclpy.spin(node)
    rclpy.shutdown()
