#!/usr/bin/env python3 
import rclpy
import time
import serial
import json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray

# Este codigo sirve para la comunicacion serial entre un Arduino/ESP y una Raspberry. Es un nodo de ros2 que se subscribe 
# al tópico cmd_Vel y lee el mensaje para pasarlo luego al Arduino/ESP. Los mensajes son tipo String.

# Este codigo es para ROS2.

# Basado en el trabajo hecho por  el subsistema motion control ROBOCOl Colombia,
# Implementación por Robocol en ROS1:https://github.com/Motion-control-rem-u/motion_control_rem_u.git 

class serial_Writer(Node):

    def __init__(self):

        super().__init__('Serial_proyecto')
        self.subscriptionNav = self.create_subscription(Float32MultiArray, 'robot_cmdVel', self.listener_callback_velocidad, 50)   
        self.subscriptionMan = self.create_subscription(Float32MultiArray, 'manipulator_cmdVel', self.listener_callback_manipulator, 10)
        self.mensajeNav=""
        self.mensajeMan=""

        self.left = "0000"
        self.right = "0000"

        self.joint1 = "0000"
        self.joint2 = "0120"
        self.joint3 = "0245"

        self.endeffector = 0 
        print("Inicio del nodo que pasa la informacion de la Raspberry al Arduino")
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.ser.reset_input_buffer()   
            print("Conexion Serial exitosa en USB0")    
        except:
            print("Conexion en USB0 fallida, probando USB1")
            try:
                self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
                self.ser.reset_input_buffer()
                print("Conexion Serial exitosa en USB1")
            except:
                print("Conexion en USB1 fallida, probando USB2")
                try:
                    self.ser = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
                    self.ser.reset_input_buffer()
                    print("Conexion Serial exitosa en USB2")
                except:
                    print("Conexion en USB2 fallida, probando USB3")
                    try:
                        self.ser = serial.Serial('/dev/ttyUSB3', 115200, timeout=1)
                        self.ser.reset_input_buffer()
                        print("Conexion Serial exitosa en USB3")
                    except:
                        print("Conexion en USB3 fallida, probando USB4")
                        try:
                            self.ser = serial.Serial('/dev/ttyUSB4', 115200, timeout=1)
                            self.ser.reset_input_buffer()
                            print("Conexion Serial exitosa en USB4")
                        except:
                            print("Trateme serio, reinicien esa mondá")



    def listener_callback_velocidad(self, msg):
        self.left = agregar_ceros_nav(int(msg.data[0]))
        self.right = agregar_ceros_nav(int(msg.data[1]))
        #print (f"left{left}")
        self.serialWriteAll()

    def listener_callback_manipulator(self, msg):
        #print("Llego mensaje: "+ str(msg)+ "\n")
        self.joint1 = agregar_ceros_man(int(msg.data[0]))
        self.joint2 = agregar_ceros_man(int(msg.data[1]))
        self.joint3 = agregar_ceros_man(int(msg.data[2]))
        self.endeffector = (int(msg.data[3]))

        if self.endeffector:
            self.openHand()
        else:
            self.closeHand()

        self.serialWriteAll()
        

    def openHand(self):
        self.endeffector = agregar_ceros_man(90)


    def closeHand(self):
        self.endeffector = agregar_ceros_man(10)

    def serialWriteAll(self):
        self.mensaje = str([self.left,self.right,self.joint1,self.joint2,self.joint3,self.endeffector]) + "\n"
        print(self.mensaje)
        self.ser.write(self.mensaje.encode('utf-8'))




def agregar_ceros_nav(numero):
    #print (f"numero{numero}")
    es_positivo=numero>=0
    numero_str=str(abs(numero))
    if es_positivo:
        numero_str="0"*(4-len(numero_str))+numero_str
    else:
        numero_str="-"+"0"*(3-len(numero_str))+numero_str
    return numero_str

def agregar_ceros_man(numero):
    es_positivo = (numero >= 0 and numero <=180)
    numero_str=str(abs(numero))
    if es_positivo:
        numero_str="0"*(4-len(numero_str))+numero_str
    else:
        print("El numero no esta en el rango permitido.")
    return numero_str

def main(args=None):
    rclpy.init(args=args)
    Serial_Writer=serial_Writer()
    rclpy.spin(Serial_Writer)
    Serial_Writer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
