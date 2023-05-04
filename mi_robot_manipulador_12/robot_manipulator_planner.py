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
        print("Inicio del nodo que sirve para llevar el end-effector del robot a una posición destino deseada.")
        self.subscription_goal = self.create_subscription(Vector3, 'robot_manipulator_goal', self.listener_callback_goal, 10)
        self.subscription_zone = self.create_subscription(String, 'robot_manipulator_zone', self.listener_callback_zone, 10)
        self.publisher = self.create_publisher(Float32MultiArray, 'manipulator_cmdVel', 10)


    def listener_callback_goal(self, msg):
        x_goal = msg.x
        y_goal = msg.y
        z_goal = msg.z
        print("X: " + x_goal + ", ")
        print("Y: " + y_goal + ", ")
        print("Z: " + z_goal + "\n")
        vel = self.cineInversa(x_goal,y_goal,z_goal)
        msg = Float32MultiArray()
        msg.data = vel
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
        print("X: " + x_goal + ", ")
        print("Y: " + y_goal + ", ")
        print("Z: " + z_goal + "\n")
        vel = self.cineInversa(x_goal,y_goal,z_goal)
        msg = Float32MultiArray()
        msg.data = vel
        self.publisher.publish(msg)


    def cineInversa(self, x_goal, y_goal, z_goal):
        vel1 = 0
        vel2 = 0
        vel3 = 0
        vel = [vel1,vel2,vel3]
        return vel


        
        

# --------------------------------------------------------MAIN-----------------------------------------------------------



def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_planner = Robot_Manipulator_Planner()
    rclpy.spin(robot_manipulator_planner)
    Robot_Manipulator_Planner.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

