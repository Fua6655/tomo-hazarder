#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Control(Node):
    def __init__(self, name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        
        self.subscription = self.create_subscription(Joy, '/joy', self.listener_callback, 10)
        
    def listener_callback(self, msg):
        L_horizontal = msg.axes[0]
        L_vertical = msg.axes[1]
        circle = msg.buttons[1]
        velocity = [L_horizontal*(1+circle), L_vertical*(1+circle)]   
            
        t = Twist()
        t.angular.z, t.linear.x = velocity
        self.publisher_.publish(t)
        self.get_logger().info("Sending msg")

def main(args=None):
    rclpy.init(args=args)
    node = Control("control_Node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
