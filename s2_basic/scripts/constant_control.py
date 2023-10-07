#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64, Bool
from geometry_msgs.msg import Twist

class Control(Node):
    def __init__(self) -> None:
        super().__init__("control")

        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.twist_timer = self.create_timer(1.0, self.twist_callback)

        self.twist_sub = self.create_subscription(Bool, "/kill",
                                                  self.kill_callback, 10)
        self.twist_cnt = 0.0

    def twist_callback(self) -> None:
        msg = Twist()
        msg.angular.x = self.twist_cnt
        msg.angular.z = self.twist_cnt

        self.twist_pub.publish(msg)

        self.twist_cnt += 1.0

    def kill_callback(self, msg: Bool) -> None:
        print("called")
        self.get_logger().fatal("Control stopped")
        print("control stopped")
        self.twist_timer.cancel()

        msg = Twist()
        msg.angular.x = 0.0
        msg.angular.z = 0.0

        self.twist_pub.publish(msg)
            

if __name__ == "__main__":
    rclpy.init()
    node = Control()
    rclpy.spin(node)
    rclpy.shutdown()
