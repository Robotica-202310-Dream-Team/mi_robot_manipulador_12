import rclpy
import math as mt
import numpy as np
import sympy as sp
from time import sleep
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from std_msgs.msg import String, Float32MultiArray

class Direct_Kinematics(Node):

    def __init__(self):
        super().__init__('robot_manipulator_directK')
        self.PF = [0.0, 0.0, 0.0]
        # Denavit-Hartenberg Parameters
        self.a_DH = np.array([0, 7.88, 14.25])
        self.d_DH = np.array([7.0, 0, 9.26])
        self.alpha_DH = np.array([0, 90, 0])

        # Articular information
        self.subscription = self.create_subscription(Float32MultiArray, 'manipulador_cmdVel', self.listener_callback, 10)
        self.publisher = self.create_publisher(Float32MultiArray, 'endeffector_position', 10)
        timer_period = 0.5 # Seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)


    def homogeneous_matrix(self, a, d, alpha, theta):
        theta = np.deg2rad(theta)
        Perspective = np.array([0, 0, 0, 1])
    	
    	# Rotation and traslation matriz 
        Matrix_H = np.array([[np.cos(theta), -np.cos(alpha)*np.sin(theta), np.sin(alpha)*np.sin(theta), a*np.cos(theta)],
    	[np.sin(theta), np.cos(alpha)*np.cos(theta), -np.sin(alpha)*np.cos(theta), a*np.sin(theta)],
    	[0, np.sin(alpha), np.cos(alpha), d],
    	Perspective])
    	
        return Matrix_H

    def listener_callback(self, msg): 
        self.articular = msg.data
        self.T01 = self.homogeneous_matrix(self.a_DH[0], self.d_DH[0], self.alpha_DH[0], self.articular[0])
        self.T12 = self.homogeneous_matrix(self.a_DH[1], self.d_DH[1], self.alpha_DH[1], self.articular[1])
        self.T23 = self.homogeneous_matrix(self.a_DH[2], self.d_DH[2], self.alpha_DH[2], self.articular[2])

        # Operational information
        J1 = self.T01
        J2 = np.dot(J1,self.T12)
        J3 = np.dot(J2,self.T23)
        self.PF = [J3[0,3], J3[1,3], J3[2,3]]
        
        data = {"Junta": ['J1', 'J2', 'J3'],
        "Posición [X, Y. Z]": [J1[0:3,3], J2[0:3,3], J3[0:3,3]]}

    def publisher_callback(self):
        msg = Float32MultiArray()
        msg.data = self.PF
        self.publisher.publish(msg)
        

	
        
def main(args=None):
    rclpy.init(args=args)
    direct_kinematics = Direct_Kinematics()
    rclpy.spin(direct_kinematics)
    direct_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
