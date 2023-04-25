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

        # Denavit-Hartenberg Parameters
        self.a_DH = np.array([0, 1, 2])
        self.d_DH = np.array([3, 0, 0])
        self.alpha_DH = np.array([0, 90, 0])

        # Articular information
        self.articular_values = self.create_subscription(Float32MultiArray, 'manipulator_cmdVel', self.listener_callback, 10)
        self.T01 = self.homogeneous_matrix(self.a_DH[0], self.d_DH[0], self.alpha_DH[0], self.articular_values[0])
        self.T12 = self.homogeneous_matrix(self.a_DH[1], self.d_DH[1], self.alpha_DH[1], self.articular_values[1])
        self.T23 = self.homogeneous_matrix(self.a_DH[2], self.d_DH[2], self.alpha_DH[2], self.articular_values[2])

        # Operational information
        J1 = self.T01
        J2 = J1*self.T12 
        J3 = J2*self.T23

        J = np.array([[J1[0:3][3]],
        [J2[0:3][3]],
        [J3[0:3][3]]])
        print(J)

    def homogeneous_matrix(self, a, d, alpha, theta):
    
        Perspective = np.array([0, 0, 0, 1])

        # Rotation and traslation matriz 
        RT = np.array([[np.cos(theta), -np.cos(alpha)*np.sin(theta), np.sin(alpha)*np.sin(theta), a*np.cos(theta)],
        [np.sin(theta), np.cos(alpha)*np.cos(theta), -np.sin(alpha)*np.cos(theta), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d]])
    
        # Concatenation between rotation, traslation and perspective
        Matrix_H = np.append(Matrix_H, Perspective, axis=0)

        return Matrix_H

    def listener_callback(self, msg): 
        self.articular = msg.data

        
def main(args=None):
    rclpy.init(args=args)
    direct_kinematics = Direct_Kinematics()
    rclpy.spin(direct_kinematics)
    direct_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
