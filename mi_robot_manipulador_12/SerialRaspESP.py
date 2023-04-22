#!/usr/bin/env python3 
import rclpy
import time
import serial
import json
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist


# Este codigo sirve para la comunicacion serial entre un Arduino/ESP y una Raspberry. Es un nodo de ros2 que se subscribe 
# al tópico manipulator_cmdVel y lee el mensaje para pasarlo luego al Arduino/ESP. Los mensajes son tipo String. 

# Este codigo es para ROS2.

class serialRaspESP(Node):

    def __init__(self):
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.ser.reset_input_buffer()   
            print("Conexion Serial exitosa")     
        except Exception:
            pass
        super().__init__('SerialRaspESP')
        print("Inicio del nodo que sirve para pasar la información que llega al tópico manipulator_cmdVel a la ESP. \n") 
        print("Corriendo en la raspberry. \n")
        self.subscription = self.create_subscription(Float32MultiArray, 'manipulator_cmdVel', self.listener_callback, 10)
        self.mensaje=""
    

    def listener_callback(self, msg):
        print("Llego mensaje: "+ str(msg)+ "\n")
        primero = int(msg.data[0])
        segundo = int(msg.data[1])
        tercero = int(msg.data[2])
        self.mensaje = "("+str(agregar_ceros(primero))+","+str(agregar_ceros(segundo))+","+str(agregar_ceros(tercero))+")"
        print("El mensaje enviado a la ESP es: " + self.mensaje)
        serial.Serial('/dev/ttyUSB0', 115200, timeout=1).write(self.mensaje.encode('utf-8'))
           

def agregar_ceros(numero):
    es_positivo = (numero >= 0 and numero <=180)
    numero_str=str(abs(numero))
    if es_positivo:
        numero_str="0"*(3-len(numero_str))+numero_str
    else:
        print("El numero no esta en el rango permitido.")
    return numero_str
    
    

def main(args=None):
    rclpy.init(args=args)
    SerialRaspESP=serialRaspESP()
    rclpy.spin(SerialRaspESP)
    SerialRaspESP.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()