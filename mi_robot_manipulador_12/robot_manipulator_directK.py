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
        self.a_DH = np.array([0, 0, 7.88, 14.25])
        self.d_DH = np.array([0, 7.0, 0, 0])
        self.alpha_DH = np.array([0, 90, 0, 0])

        # Articular information
        self.subscription = self.create_subscription(Float32MultiArray, 'manipulator_cmdVel', self.listener_callback, 10)
        self.publisher = self.create_publisher(Float32MultiArray, 'endeffector_position', 10)
        timer_period = 0.01 # Seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)


## --------------------------------------------------------------------------
## -----------------------------ROTATION-------------------------------------
## --------------------------------------------------------------------------

    # Dynamic movement: Roll
    def rot_X(self, Theta):
        
        # Rotation in the X-axis 
        self.rotation_x = np.matrix([[1, 0, 0, 0],
                                    [0, mt.cos(Theta), -(mt.sin(Theta)), 0],
                                [0, mt.sinh(Theta), mt.cos(Theta), 0],
                                [0, 0, 0, 1]])
        
        return self.rotation_x
        
        
    # Dynamic movement: pitch
    def rot_Y(self, Theta):

        # Rotation in the Y-axis 
        self.rotation_y = np.matrix([[mt.cos(Theta), 0, mt.sin(Theta), 0],
                                [0, 1, 0, 0],
                                [-(mt.sin(Theta)), 0, mt.cos(Theta), 0],
                                [0, 0, 0, 1]])

        return self.rotation_y
        
    # Dynamic movement: Yaw
    def rot_Z(self, Theta):

        # Rotation in the Z-axis 
        self.rotation_z = np.matrix([[mt.cos(Theta), -(mt.sin(Theta)), 0, 0],
                                [mt.sin(Theta), mt.cos(Theta), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

        return self.rotation_z


## --------------------------------------------------------------------------
## ----------------------------TRASLATION------------------------------------
## --------------------------------------------------------------------------

# Traslation in Z 
    def tras_Z(self, d):
        
        self.trasl_Z = np. matrix([[1, 0, 0, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, d],
                                    [0, 0, 0, 1]])
        return self.trasl_Z
        
# Traslation in X
    def tras_X(self, a):
        
        self.trasl_X = np. matrix([[1, 0, 0, a],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])
        return self.trasl_X


## --------------------------------------------------------------------------
## ----------------------------PERSPECTIVE-----------------------------------
## --------------------------------------------------------------------------

# Perspective in homogneous matrix
    def set_perspective(self, pers_X, pers_Y, pers_Z):
        
        # Sets the perspective parameter of the homogeneous matrix
        self.matrix[3, 0] = pers_X
        self.matrix[3, 1] = pers_Y
        self.matrix[3, 2] = pers_Z


## --------------------------------------------------------------------------
## -------------------------------SCALE--------------------------------------
## --------------------------------------------------------------------------
    
    # Scale in homogneous matrix
    def set_scale(self, scale):
        
        # Scale factor of the homogeneous matrix
        self.matrix[3, 3] = scale

## --------------------------------------------------------------------------
## -----------------------------CALLBACK-------------------------------------
## --------------------------------------------------------------------------


    def listener_callback(self, msg): 
        self.articular = msg.data
        print(self.articular)

        self.HM_Base = np.identity(4)
        self.HM_J1 = (self.rot_Z(mt.radians(self.articular[0]))) @ (self.tras_Z(self.d_DH[1])) @ (self.tras_X(self.a_DH[1])) @ (self.rot_X(mt.radians(self.alpha_DH[1])))
        self.HM_J2 = (self.rot_Z(mt.radians(self.articular[1]))) @ (self.tras_Z(self.d_DH[2])) @ (self.tras_X(self.a_DH[2])) @ (self.rot_X(mt.radians(self.alpha_DH[2])))
        self.HM_J3 = (self.rot_Z(mt.radians(self.articular[2]))) @ (self.tras_Z(self.d_DH[3])) @ (self.tras_X(self.a_DH[3])) @ (self.rot_X(mt.radians(self.alpha_DH[3])))

        # Operational information
        self.J1 = self.HM_Base @ self.HM_J1
        self.J2 = self.J1 @ self.HM_J2
        self.J3 = self.J2 @ self.HM_J3
        self.PF = [self.J1[0,3], self.J1[1,3], self.J1[2,3], self.J2[0,3], self.J2[1,3], self.J2[2,3], self.J3[0,3], self.J3[1,3], self.J3[2,3]]
        
        data = {"Junta": ['J1', 'J2', 'J3'],
        "Posición [X, Y. Z]": [self.J1[0:3,3], self.J2[0:3,3], self.J3[0:3,3]]}

    def publisher_callback(self):
        msg = Float32MultiArray()
        msg.data = self.PF
        self.publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    direct_kinematics = Direct_Kinematics()
    rclpy.spin(direct_kinematics)
    Direct_Kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


