import rclpy
import numpy as np
import sympy as sp
from time import sleep
from rclpy.node import Node
from pynput import keyboard
from geometry_msgs.msg import Vector3
from rclpy.duration import Duration
from std_msgs.msg import String, Float32MultiArray


class Robot_Manipulator_Planner(Node):

    # -----------------------------------------------------INIT--------------------------------------------------------------
    def __init__(self):
        super().__init__('robot_manipulator_planner')

        # Denavit-Hartenberg Parameters
        self.a_DH = np.array([0, 0, 7.88, 14.25])
        self.d_DH = np.array([0, 7.0, 0, 0])
        self.alpha_DH = np.array([0, 90, 0, 0])

        print("Inicio del nodo que sirve para llevar el end-effector del robot a una posición destino deseada.")
        self.subscription_goal = self.create_subscription(Vector3, 'robot_manipulator_goal', self.listener_callback_goal, 10)
        self.subscription_zone = self.create_subscription(String, 'robot_manipulator_zone', self.listener_callback_zone, 10)
        
        self.publisher = self.create_publisher(Float32MultiArray, 'manipulator_cmdVel', 10)

    def listener_callback_goal(self, msg):
        x_goal = msg.x
        y_goal = msg.y
        z_goal = msg.z
        print("X: " + str(x_goal) + ", ")
        print("Y: " + str(y_goal) + ", ")
        print("Z: " + str(z_goal) + "\n")

        articular_state = self.invKinema(x_goal, y_goal, z_goal)
        msg = Float32MultiArray()
        msg.data = articular_state
        self.publisher.publish(msg)

    def listener_callback_zone(self, msg):
        mensaje = msg.data
        if mensaje == "zone1":
            x_goal = 1
            y_goal = 1
            z_goal = 16
        elif mensaje == "zone2":
            x_goal = 2
            y_goal = 1
            z_goal = 16
        elif mensaje == "zone3":
            x_goal = 3
            y_goal = 1
            z_goal = 16
        else:
            print("La zona especificada no es válida")
        print("X: " + str(x_goal) + ", ")
        print("Y: " + str(y_goal) + ", ")
        print("Z: " + str(z_goal) + "\n")
        articular_state = self.invKinema(x_goal,y_goal,z_goal)
        msg = Float32MultiArray()
        msg.data = articular_state
        self.publisher.publish(msg)


    def invKinema(self, x_goal, y_goal, z_goal):
    
        # Theta 1
        x1_1 = y_goal
        x1_2 = x_goal
        Theta1 = np.arctan2(x1_1, x1_2)
        if Theta1 < 0:
        	Theta1 += np.pi
        
    	# Theta 3
        self.c3 = (x_goal**2 + y_goal**2 + z_goal**2 - (self.d_DH[1]**2 + self.a_DH[2]**2 + self.a_DH[3]**2) - 2*self.d_DH[1]*(z_goal-self.d_DH[1]))/(2*self.a_DH[2]*self.a_DH[3])
        
        self.s3 = np.sqrt(1 - self.c3)
        Theta3 = np.arctan2(self.s3, self.c3)
        if Theta3 < 0:
        	Theta3 += np.pi
        
        # Theta 2
        self.x2_11 = (z_goal-self.d_DH[1])*(np.cos(Theta1)-np.sin(Theta1))
        self.x2_12 = (x_goal-y_goal)
        self.x2_21 = (self.a_DH[3]*np.sin(Theta3))
        self.x2_22 = (self.a_DH[3]*np.cos(Theta3)+self.a_DH[2])
        Theta2 = np.arctan2(self.x2_11, self.x2_12) - np.arctan2(self.x2_21, self.x2_22)
        if Theta2 < 0:
        	Theta2 += np.pi
        
        Theta = [np.rad2deg(Theta1), np.rad2deg(Theta2), np.rad2deg(Theta3), 90.0]
        return Theta
    
# --------------------------------------------------------MAIN-----------------------------------------------------------



def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_planner = Robot_Manipulator_Planner()
    rclpy.spin(robot_manipulator_planner)
    Robot_Manipulator_Planner.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

