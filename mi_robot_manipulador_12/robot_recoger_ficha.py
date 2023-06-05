import rclpy
import numpy as np
import sympy as sp
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

        # Denavit-Hartenberg Parameters
        #self.a_DH = np.array([0, 0, 7.88, 14.25])
        #self.d_DH = np.array([0, 7.0, 0, 0])
        #self.alpha_DH = np.array([0, 90, 0, 0])

        print("Inicio del nodo que sirve para recoger la ficha .")
        #self.subscription_goal = self.create_subscription(Vector3, 'robot_manipulator_goal', self.listener_callback_goal, 10)
        #self.subscription_zone = self.create_subscription(String, 'robot_manipulator_zone', self.listener_callback_zone, 10)
        self.subscription_flag_recoger = self.create_subscription(Bool, 'flag_recoger', self.listener_callback, 10)
        self.publisher = self.create_publisher(Float32MultiArray, 'manipulator_cmdVel', 10)
        self.trayectoria = [[90,90,90,90]]
        def listener_callback(self, msg):
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

def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_planner = Robot_Manipulator_Planner()
    rclpy.spin(robot_manipulator_planner)
    Robot_Manipulator_Planner.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

