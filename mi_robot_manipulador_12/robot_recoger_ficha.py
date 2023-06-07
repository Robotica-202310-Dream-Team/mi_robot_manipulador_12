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


class Robot_Recoger_Ficha(Node):

    # -----------------------------------------------------INIT--------------------------------------------------------------
    def __init__(self):
    	self.msg1 = Bool()
    	self.msg2 = Bool()
    	self.msg = Float32MultiArray()
        super().__init__('robot_recoger_ficha')
        self.publisher = self.create_publisher(Float32MultiArray, 'manipulator_cmdVel', 10)
        self.subscription_flag_recoger = self.create_subscription(Bool, 'pick_up', self.listener_callback_flag_pick_up, 10)
        self.subscription_flag_dejar = self.create_subscription(Bool, 'release', self.listener_callback_flag_place, 10)
    
    def listener_callback_flag_pick_up(self, msg):
        msg1.data = msg.data
	if msg1.data == True:
		self.Pick_Up()
	else:
		pass
		
    def listener_callback_flag_place(self, msg):
        msg2.data = msg.data
	if msg1.data == True:
		self.Place()
	else:
		pass

   def Pick_Up(self):
   	# Home
   	msg = Float32MultiArray()
   	self.msg.data[0] = 0.0
        self.msg.data[1] = 120.0
        self.msg.data[2] = 245.0
        self.msg.data[3] = 0.0
        self.publisher_.publish(msg)
        time.sleep(1)
        
        # P1
   	self.msg.data[0] = 0.0
        self.msg.data[1] = 111.0
        self.msg.data[2] = 245.0
        self.msg.data[3] = 0.0
        self.publisher_.publish(msg)
        time.sleep(1)
        
        # P2
   	self.msg.data[0] = 0.0
        self.msg.data[1] = 102.0
        self.msg.data[2] = 245.0
        self.msg.data[3] = 0.0
        self.publisher_.publish(msg)
        time.sleep(1)
        
        # P3
   	self.msg.data[0] = 0.0
        self.msg.data[1] = 93.0
        self.msg.data[2] = 245.0
        self.msg.data[3] = 0.0
        self.publisher_.publish(msg)
        time.sleep(1)
        
        # Cerrar
   	self.msg.data[0] = 0.0
        self.msg.data[1] = 93.0
        self.msg.data[2] = 245.0
        self.msg.data[3] = 1.0
        self.publisher_.publish(msg)
        time.sleep(1)
        
        # P2
   	self.msg.data[0] = 0.0
        self.msg.data[1] = 102.0
        self.msg.data[2] = 245.0
        self.msg.data[3] = 1.0
        self.publisher_.publish(msg)
        time.sleep(1)
        
        # P1
   	self.msg.data[0] = 0.0
        self.msg.data[1] = 111.0
        self.msg.data[2] = 245.0
        self.msg.data[3] = 1.0
        self.publisher_.publish(msg)
        time.sleep(1)
        
        # Home
        self.msg.data[0] = 0.0
        self.msg.data[1] = 120.0
        self.msg.data[2] = 245.0
        self.msg.data[3] = 1.0
        self.publisher_.publish(msg)
        time.sleep(1)
        print("Se recogio ficha")
   
    def Place(self, msg):
        # Home
        msg = Float32MultiArray()
        self.msg.data[0] = 0.0
        self.msg.data[1] = 120.0
        self.msg.data[2] = 245.0
        self.msg.data[3] = 1.0
        self.publisher_.publish(msg)
        time.sleep(1)
        
        # P1
        self.msg.data[0] = 0.0
        self.msg.data[1] = 120.0
        self.msg.data[2] = 275.0
        self.msg.data[3] = 1.0
        self.publisher_.publish(msg)
        time.sleep(1)
        
        # P2
        self.msg.data[0] = 0.0
        self.msg.data[1] = 76.0
        self.msg.data[2] = 275.0
        self.msg.data[3] = 1.0
        self.publisher_.publish(msg)
        time.sleep(1)
        
        # Abrir
        self.msg.data[0] = 0.0
        self.msg.data[1] = 76.0
        self.msg.data[2] = 275.0
        self.msg.data[3] = 0.0
        self.publisher_.publish(msg)
        time.sleep(1)
        
        # P2
        self.msg.data[0] = 0.0
        self.msg.data[1] = 76.0
        self.msg.data[2] = 275.0
        self.msg.data[3] = 0.0
        self.publisher_.publish(msg)
        time.sleep(1)
        
        # P1
        self.msg.data[0] = 0.0
        self.msg.data[1] = 120.0
        self.msg.data[2] = 275.0
        self.msg.data[3] = 0.0
        self.publisher_.publish(msg)
        time.sleep(1)
        
        # Home
        self.msg.data[0] = 0.0
        self.msg.data[1] = 120.0
        self.msg.data[2] = 245.0
        self.msg.data[3] = 0.0
        self.publisher_.publish(msg)
        time.sleep(1)
        print("Se solto	 ficha")
    
def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_planner = Robot_Recoger_Ficha()
    rclpy.spin(robot_recoger_ficha)
    Robot_Manipulator_Planner.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

