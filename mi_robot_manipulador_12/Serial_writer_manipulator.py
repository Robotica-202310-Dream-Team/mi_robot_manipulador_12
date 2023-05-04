#!/usr/bin/env python3 
import rclpy
import time
import serial
import json
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
#import pyfirmata
import time

class Serial_writer(Node):
    theta0 = 90.0 
    theta1 = 90.0
    theta2 = 90.0
    theta3 = 0.0

    board = None
    servo0 = None # servo correspondiente al end-effector
    servo1 = None
    servo2 = None
    servo3 = None



    def __init__(self):
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.ser.reset_input_buffer()   
            print("Conexion Serial exitosa")     
        except Exception:
            print("Conexion Serial Fallida")
            pass   
        super().__init__('Serial_writer_manipulator')
        print("Inicio del nodo que escribe los ángulos de los joints por serial a la ESP32\n") 
        print("Corriendo en la raspberry. \n")
        self.subscription = self.create_subscription(Float32MultiArray, 'manipulator_cmdVel', self.listener_callback, 10)
        self.mensaje=""
            
        self.joint1 = 90
        self.joint2 = 90
        self.joint3 = 90
        self.endeffector = 0

        """self.board = pyfirmata.Arduino('/dev/ttyUSB0') # Identificar placa
        # Configuracion inicila de pines a los que se conecta los servos

        self.board.servo_config(3, angle = self.theta0) # Configure a pin as servo with first angle.
        self.servo1 = self.board.get_pin('d:3:s') #digital pin, pin3, serial
        self.servo1.write(self.theta0) #Move servo to initial position

        self.board.servo_config(5, angle = self.theta1) # Configure a pin as servo with first angle.
        self.servo2 = self.board.get_pin('d:5:s') #digital pin, pin3, serial
        self.servo2.write(self.theta1) #Move servo to initial position

        self.board.servo_config(6, angle = self.theta2)# Configure a pin as servo with first angle.
        self.servo3 = self.board.get_pin('d:6:s') #digital pin, pin3, serial
        self.servo3.write(self.theta2) #Move servo to initial position

        # end-effector
        self.board.servo_config(9, angle = self.theta3)# Configure a pin as servo with first angle.
        self.servo0 = self.board.get_pin('d:9:s') #digital pin, pin3, serial 
        self.servo0.write(self.theta3) #Move servo to initial position
        """

    def listener_callback(self, msg):
        print("Llego mensaje: "+ str(msg)+ "\n")
        self.joint1 = agregar_ceros(int(msg.data[0]))
        self.joint2 = agregar_ceros(int(msg.data[1]))
        self.joint3 = agregar_ceros(int(msg.data[2]))
        self.endeffector = (int(msg.data[3]))
        self.sleep = 0.015
        #self.goto(self.joint1,self.joint2,self.joint3,self.endeffector)#Mover los joints 1, 2, 3 y end effector
        if self.endeffector:
            self.openHand()
        else:
            self.closeHand()
        self.mensaje = str([self.joint1,self.joint2,self.joint3,self.endeffector]) + "\n"
        print(self.mensaje)
        self.ser.write(self.mensaje.encode('utf-8'))


        

    def __del__(self):
        print("in __del__")
        #self.goto(90, 90 ,90)
        self.closeHand()
        #self.board.exit()

    """
    def set0(self, theta0_desired): # mover end effector
        theta0_desired = max(theta0_desired, 0.0)
        theta0_desired = min(theta0_desired, 180.0)
        while abs(self.theta0-theta0_desired)>0.1:
            if self.theta0 <= theta0_desired:
                self.theta0 = self.theta0 + 1.0
            elif self.theta0 > theta0_desired:
                self.theta0 = self.theta0 - 1.0
            #self.servo0.write(self.theta0) # escribir posicion al end effector
            self.endeffector = self.theta0
            time.sleep(self.sleep)
        #print(str(self.theta0) + "," + str(self.theta1) + "," + str(self.theta2))

    def set1(self, theta1_desired):# mover joint 1
        theta1_desired = max(theta1_desired, 0.0)
        theta1_desired = min(theta1_desired, 180.0)
        while abs(self.theta1-theta1_desired)>0.1:
            if self.theta1 <= theta1_desired:
                self.theta1 = self.theta1 + 1.0
            elif self.theta1 > theta1_desired:
                self.theta1 = self.theta1 - 1.0
            #self.servo1.write(self.theta1) # escribir al joint 1
            self.joint1 = self.theta1
            time.sleep(self.sleep)
        #print(str(self.theta0) + "," + str(self.theta1) + "," + str(self.theta2))

        
    def set2(self, theta2_desired): # mover al joint 2
        theta2_desired = max(theta2_desired, 0.0)
        theta2_desired = min(theta2_desired, 180.0)
        while abs(self.theta2-theta2_desired)>0.1:
            if self.theta2 <= theta2_desired:
                self.theta2 = self.theta2 + 1.0
            elif self.theta2 > theta2_desired:
                self.theta2 = self.theta2 - 1.0
            #self.servo2.write(self.theta2) # escribir al joint 2
            self.joint2 = self.theta2
            time.sleep(self.sleep)
        #print(str(self.theta0) + "," + str(self.theta1) + "," + str(self.theta2))

    def set3(self, theta3_desired): # mover al joint 3
        theta3_desired = max(theta3_desired, 0.0)
        theta3_desired = min(theta3_desired, 90.0)
        while abs(self.theta3-theta3_desired)>0.1:
            if self.theta3 <= theta3_desired:
                self.theta3 = self.theta3 + 1.0
            elif self.theta3 > theta3_desired:
                self.theta3 = self.theta3 - 1.0
            #self.servo3.write(self.theta3) # escribir al joint 3
            self.joint3 = self.theta3
            time.sleep(self.sleep)


    def goto(self, theta1_desired, theta2_desired, theta3_desired):
        self.set1(theta1_desired)
        self.set2(theta2_desired)
        self.set3(theta3_desired)
        #print(str(self.theta1) + "," + str(self.theta2) + "," + str(self.theta3))
        time.sleep(0.2)
    """
    def openHand(self):
        self.endeffector = agregar_ceros(70) 


    def closeHand(self):
        self.endeffector = agregar_ceros(10) 


def agregar_ceros(numero):
    es_positivo = (numero >= 0 and numero <=180)
    numero_str=str(abs(numero))
    if es_positivo:
        numero_str="0"*(4-len(numero_str))+numero_str
    else:
        print("El numero no esta en el rango permitido.")
    return numero_str

def main(args=None):
    rclpy.init(args=args)
    serial_writer=Serial_writer()
    rclpy.spin(serial_writer)
    Serial_writer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
