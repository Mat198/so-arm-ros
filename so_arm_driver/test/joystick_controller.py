#!/usr/bin/env python

from trajectory_msgs.msg import JointTrajectoryPoint
import math
import threading
from pyPS4Controller.controller import Controller

import rclpy
from rclpy.node import Node
import rclpy
from rclpy.node import Node


class PS4JointController(Node, Controller):
    def __init__(self):
        Node.__init__(self, node_name='ps4_joint_controller')
        Controller.__init__(self, interface="/dev/input/js0", connecting_using_ds4drv=False)

        self.joint_pub = self.create_publisher(JointTrajectoryPoint, '/so_arm/target', 10)
        self.timer = self.create_timer(0.020, self.publish_callback)
        self.target = JointTrajectoryPoint()
        self.target.positions = [0, 0, 0, 0, 0, 0]
        self.target.velocities = [0, 0, 0, 0, 0, 0]

    def publish_callback(self):
        self.joint_pub.publish(self.target)
        
    # ================ Controles da direita ================ 
    def on_R3_up(self, value):
        self.target.positions[0] += value /   
        if value < 1000:
            return

    def on_R3_down(self, value):
        if value < 1000:
            return

    def on_R3_right(self, value):
        if value < 1000:
            return
    
    def on_R3_left(self, value):
        if value < 1000:
            return
        
    def on_R3_x_at_rest(self):
        pass

    def on_R3_y_at_rest(self):
        pass

    # =============== Controles da esquerda ================
    def on_L3_up(self, value):
        if value < 1000:
            return

    def on_L3_down(self, value):
        if value < 1000:
            return
    
    def on_L3_left(self, value):
        if value < 1000:
            return
    
    def on_L3_right(self, value):
        if value < 1000:
            return
        
    def on_L3_x_at_rest(self):
        pass

    def on_L3_y_at_rest(self):
        pass

    # =============== Controles dos gatilhos ================
    def on_R2_press(self, value):
        if value < 1000:
            return
    
    def on_R2_release(self):
        pass

    def on_L2_press(self, value):
        if value < 1000:
            return
    
    def on_L2_release(self):
        pass


if __name__ == '__main__':
    rclpy.init()
    node = PS4JointController()
    print("Is blocked?")
    node.listen(timeout=60)
    print("No")
    pass