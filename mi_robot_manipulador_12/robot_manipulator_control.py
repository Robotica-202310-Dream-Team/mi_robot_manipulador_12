import time
import rclpy
import math 
import numpy as np
import sympy as sp
from time import sleep
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray, Float64MultiArray, Bool

class Robot_Manipulator_Control(Node):

    def __init__(self):
        self.flagZone1 = False
        self.flagZone2 = False
        self.flag_take = False
        self.flag_place = False
        self.subscription_retro_recoger = self.create_subscription(Bool, 'flag_recoger_retro', self.listener_callback_flag_take_retro, 10)
        self.subscription_retro_dejar = self.create_subscription(Bool, 'flag_dejar_retro', self.listener_callback_flag_take_place, 10)
        self.publisher_tomar = self.create_publisher(Bool, 'flag_recoger', 10)
        self.publisher_dejar = self.create_publisher(Bool, 'flag_dejar', 10)

    def listener_callback_flag_take_retro(self, msg):
        self.flag_retro_take = msg.data 

    def listener_callback_flag_take_place(self, msg):
        self.flag_retro_place = msg.data 

def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_control = Robot_Manipulator_Control()
    rclpy.spin(robot_manipulator_control)
    Robot_Manipulator_Control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
