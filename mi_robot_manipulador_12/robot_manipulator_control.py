import time
import rclpy
import math 
import numpy as np
import sympy as sp
from time import sleep
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray, Bool

class Robot_Manipulator_Control(Node):

    def __init__(self):
        # Flags
        self.FlagPosZone1_Up = False
        self.FlagPos1  = False
        self.FlagPick= False
        self.FlagUp= False

        self.FlagPosZone1_Down = False
        
        # Initial coordinates
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_Theta = 0.0

        super().__init__('robot_manipulator_control')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                        depth=1)
        
        # Articular information
        self.subscription = self.create_subscription(Odometry, 'camera/pose/sample' ,self.subscriber_callback_pos_actual, qos_profile=qos_policy)
        self.publisher = self.create_publisher(Float32MultiArray, 'manipulator_cmdVel', 10)

    # Map zones
    def zone11_goal(self):
        # Align with base 1
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_Theta = 90.0

    def zone12_goal(self):
        # Go back 
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_Theta = 90.0

    def zone21_goal(self):
        # Align with base 2
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_Theta = -90.0

    def zone22_goal(self):
        # Go back 
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_Theta = -90.0

    # Pick-Up trajectories
    def home_traj(self):
        self.Theta1 = 0.0
        self.Theta2 = 120.0
        self.Theta3 = 245.0
        self.Theta4 = 10.0
        self.ThetaS = [self.Theta1, self.Theta2, self.Theta3, self.Theta4]

    def Pos1_traj(self, intsance):
        self.Theta1 = 0.0
        self.Theta2 = 72.0
        self.Theta3 = 270.0
        self.Theta4 = 10.0
        self.ThetaS = [self.Theta1, self.Theta2, self.Theta3, self.Theta4]

    def Pick_traj(self):
        self.Theta1 = 0.0
        self.Theta2 = 72.0
        self.Theta3 = 270.0
        self.Theta4 = 90.0
        self.ThetaS = [self.Theta1, self.Theta2, self.Theta3, self.Theta4]

    def Up_traj(self):
        self.Theta1 = 0.0
        self.Theta2 = 120.0
        self.Theta3 = 245.0
        self.Theta4 = 90.0
        self.ThetaS = [self.Theta1, self.Theta2, self.Theta3, self.Theta4]

    def subscriber_callback_pos_actual(self, msg):
        # Actual position of the robot
        self.actual_pos_x = round (msg.pose.pose.position.x*100,2 )
        self.actual_pos_y = round (msg.pose.pose.position.y *100,2 )
        self.actual_pos_ThetaX = round (msg.pose.pose.orientation.x,2 )
        self.actual_pos_ThetaY = round (msg.pose.pose.orientation.y,2 )
        self.actual_pos_ThetaZ = round (msg.pose.pose.orientation.z,2 )
        self.actual_pos_ThetaW = round (msg.pose.pose.orientation.w,2 )

        self.actualGrado2 = self.euler_from_quaternion(self.actual_pos_ThetaX,self.actual_pos_ThetaY,self.actual_pos_ThetaZ,self.actual_pos_ThetaW)
        self.actualGrado = self.actualGrado2[2]*180/(np.pi)
        print("x_actual: "+ str(self.actual_pos_x) +"\n")
        print("y_actual: "+ str(self.actual_pos_y)+"\n") 
        print("ThetaZ_actual: "+ str(self.actualGrado)+"\n")

        # Pick up
        self.instance = 0
        self.FlagPosZone1_Up = True

        if self.FlagPosZone1_Up== True and self.instance == 0:
            self.home_traj()
            self.msg1.data = self.ThetaS
            self.publisher_vel.publish(self.msg1)
            time.sleep(3)
            self.FlagPos1 = True

        if self.FlagPos1 == True:
            self.Pos1_traj()
            self.msg1.data = self.ThetaS
            self.publisher_vel.publish(self.msg1)
            time.sleep(3)
            self.FlagPick = True

        if self.FlagPick == True:
            self.Up_traj()
            self.msg1.data = self.ThetaS
            self.publisher_vel.publish(self.msg1)
            time.sleep(3)
            self.FlagUp = True

        if self.FlagUp == True:
            self.Up_traj()
            self.msg1.data = self.ThetaS
            self.publisher_vel.publish(self.msg1)
            time.sleep(3)
            self.instance = 1
            self.FlagPosZone1_Down = True
            self.FlagPosZone1_Up = False
            self.FlagPos1  = False
            self.FlagPick= False
            self.FlagUp= False

        # Place
        if self.FlagPosZone1_Down == True:
            time.sleep(5)
            self.Pick_traj()
            self.msg1.data = self.ThetaS
            self.publisher_vel.publish(self.msg1)
            time.sleep(3)
            self.FlagPlace = True

        if self.FlagPlace == True:
            self.Pos1_traj()
            self.msg1.data = self.ThetaS
            self.publisher_vel.publish(self.msg1)
            time.sleep(3) 
            self.GoHome = True

        if self.GoHome == True:
            self.home_traj()
            self.msg1.data = self.ThetaS
            self.publisher_vel.publish(self.msg1)
            time.sleep(3) 
            self.FlagPosZone1_Down = False
            self.FlagPlace = False
            self.GoHome = False

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return float(roll_x), float(pitch_y), float(yaw_z) # in radians
    
    def publisher_callback(self):
        msg = Float32MultiArray()
        msg.data = self.PF
        self.publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_control = Robot_Manipulator_Control()
    rclpy.spin(robot_manipulator_control)
    Robot_Manipulator_Control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
