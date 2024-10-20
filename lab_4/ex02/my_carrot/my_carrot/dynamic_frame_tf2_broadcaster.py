#Предназначен для трансляции изменений положения и ориентации между двумя фреймами (координатными системами).
#между turtle1 и carrot1 

import math
import sys

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class DynamicFrameBroadcaster(Node):

    def __init__(self, args):
        super().__init__('dynamic_frame_tf2_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.025, lambda: self.broadcast_timer_callback(args))

    def broadcast_timer_callback(self, args):
        seconds, _ = self.get_clock().now().seconds_nanoseconds()

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot1'
        x = seconds
        t.transform.translation.x = math.cos(x*float(args[4])) * float(args[2]) 
        t.transform.translation.y = math.sin(x*float(args[4])) * float(args[2]) 
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = DynamicFrameBroadcaster(sys.argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
