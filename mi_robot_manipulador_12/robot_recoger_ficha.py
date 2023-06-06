import rclpy
import numpy as np
import sympy as sp
import time
from time import sleep
from rclpy.node import Node
from pynput import keyboard
from geometry_msgs.msg import Vector3
from rclpy.duration import Duration
from std_msgs.msg import String, Float32MultiArray, Bool


class Robot_Manipulator_Planner(Node):

    # -----------------------------------------------------INIT--------------------------------------------------------------
    def __init__(self):
        super().__init__('robot_manipulator_planner')
        print("Inicio del nodo que sirve para recoger la ficha .")
        self.P1 = 0
        self.P2 = 0
        self.flagTake = False
        self.flagPlace = False
        self.subscription_flag_recoger = self.create_subscription(Bool, 'flag_recoger', self.listener_callback_flag_take, 10)
        self.subscription_flag_dejar = self.create_subscription(Bool, 'flag_dejar', self.listener_callback_flag_place, 10)
        self.publisher = self.create_publisher(Float32MultiArray, 'manipulator_cmdVel', 10)
        self.publisher_retro_recoger = self.create_publisher(Bool, 'flag_recoger_retro', 10)
        self.publisher_retro_dejar = self.create_publisher(Bool, 'flag_dejar_retro', 10)
    
    def listener_callback_flag_take(self, msg):
        msg1 = Bool()
        msg1.data = False
        self.flagTake = msg.data
        if self.flagTake == False:
            self.P1 = 0
        if self.P1 == 1:
            msg1.data = True
            self.publisher_retro_recoger.publish(msg1)
        if self.flagTake == True and self.P1 == 0:
            self.P1 += 1
            self.pickUp()
        else:
            self.publisher_retro_recoger.publish(msg1)

    def listener_callback_flag_place(self, msg):
        msg1 = Bool()
        msg1.data = False
        self.flagPlace = msg.data
        if self.flagPlace == False:
            self.P2 = 0
        if self.P2 == 1:
            msg1.data = True
            self.publisher_retro_dejar.publish(msg1)
        if self.flagPlace == True and self.P2 == 0:
            self.P2 += 1
            self.place()
        else:
            self.publisher_retro_dejar.publish(msg1)

    def pickUp(self, msg):
        print("Recogiendo")
        msg = Float32MultiArray()
        self.msg.data[0] = 0
        self.msg.data[1] = 120
        self.msg.data[2] = 245
        self.msg.data[3] = 1
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ###########################
        self.msg.data[0] = 0
        self.msg.data[1] = 120
        self.msg.data[2] = 275
        self.msg.data[3] = 1
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ##########################
        self.msg.data[0] = 0
        self.msg.data[1] = 76
        self.msg.data[2] = 275
        self.msg.data[3] = 1
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        self.msg.data[0] = 0
        self.msg.data[1] = 76
        self.msg.data[2] = 275
        self.msg.data[3] = 0
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        self.msg.data[0] = 0
        self.msg.data[1] = 120
        self.msg.data[2] = 275
        self.msg.data[3] = 0
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        self.msg.data[0] = 0
        self.msg.data[1] = 120
        self.msg.data[2] = 245
        self.msg.data[3] = 0
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        print("Ya recogió")

    def place(self,msg):
        print("Soltando")
        msg = Float32MultiArray()
        self.msg.data[0] = 0
        self.msg.data[1] = 120
        self.msg.data[2] = 245
        self.msg.data[3] = 0
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ###########################
        self.msg.data[0] = 0
    
        self.msg.data[1] = 120
        self.msg.data[2] = 275
        self.msg.data[3] = 0
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ##########################
        self.msg.data[0] = 0
        self.msg.data[1] = 76
        self.msg.data[2] = 275
        self.msg.data[3] = 0
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        self.msg.data[0] = 0
        self.msg.data[1] = 76
        self.msg.data[2] = 275
        self.msg.data[3] = 1
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        self.msg.data[0] = 0
        self.msg.data[1] = 120
        self.msg.data[2] = 275
        self.msg.data[3] = 1
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        self.msg.data[0] = 0
        self.msg.data[1] = 120
        self.msg.data[2] = 245
        self.msg.data[3] = 1
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        print("Ya soltó")
    
def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_planner = Robot_Manipulator_Planner()
    rclpy.spin(robot_manipulator_planner)
    Robot_Manipulator_Planner.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

