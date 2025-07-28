#!/usr/bin/env python

# Based in the ros2_teleop_keyboard package
import math
import rclpy
from rclpy.qos import qos_profile_system_default

from trajectory_msgs.msg import JointTrajectoryPoint

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

msg = """
---------------------------
For move joint 0: +q -a
For move joint 1: +w -s
For move joint 2: +e -d
For move joint 3: +r -f
For move joint 4: +t -g
For move joint 5: +y -h
Speed: z/c increase/decrease max speeds by 10%
anything else or no key: stop
---------------------------

CTRL-C to quit
"""

info = "Reading from the keyboard  and publishing to "

bindings = {
    'q':(0,1), 'a':(0,-1),
    'w':(1,1), 's':(1,-1),
    'e':(2,1), 'd':(2,-1),
    'r':(3,1), 'f':(3,-1),
    't':(4,1), 'g':(4,-1),
    'y':(5,1), 'h':(5,-1),
    'z':(-1,1),'c':(-1,-1),
}

STEPS_PER_REVOLUTION = 4096
MAX_STEPS_PER_SECOND = 3000

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed):
    return f"Current arm servo's speed: {round(100 * speed, 0)}"

def main():    
    rclpy.init()
    node = rclpy.create_node('teleop_arm_keyboard')
    pub = node.create_publisher(JointTrajectoryPoint, '/so_arm/target', qos_profile_system_default)

    max_speed = 2 * math.pi * MAX_STEPS_PER_SECOND / STEPS_PER_REVOLUTION # rads/s
    
    limits = [
        {'min': -1.920, 'max': 1.920},
        {'min': -1.745, 'max': 1.745},
        {'min': -1.690, 'max': 1.580},
        {'min': -1.658, 'max': 1.658},
        {'min': -2.744, 'max': 2.841},
        {'min': -0.175, 'max': 1.745}
    ]
    target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    speed_scale = 0.1
    frequency = 50 # Hz
    print(info + pub.topic_name)
    print(msg)
    print(vels(speed_scale))
    
    while(rclpy.ok()):
        try:
            print("waiting key")
            key = getKey()
            print("got key")
            if not key in bindings.keys():
                continue

            joint = bindings[key][0]
            direction = bindings[key][1]
            if joint >= 0:
                target[joint] += direction * speed_scale * max_speed * (1/frequency)
                if target[joint] > limits[joint]['max']: target[joint] = limits[joint]["max"]
                if target[joint] < limits[joint]['min']: target[joint] = limits[joint]["min"]
            else:
                speed_scale += direction * 0.1
                if speed_scale < 0.0: speed_scale = 0.0
                if speed_scale > 1.0: speed_scale = 1.0
                print(vels(speed_scale))

            target_msg = JointTrajectoryPoint()
            target_msg.positions = target
            target_msg.velocities = [speed_scale*max_speed for i in range(0,6)]
            pub.publish(target_msg)
        except Exception as e:
            print(e)
        finally:
            # Publish a message stoping the robot
            stop_msg = JointTrajectoryPoint()
            stop_msg.positions = target
            stop_msg.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pub.publish(stop_msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == "__main__":
    main()