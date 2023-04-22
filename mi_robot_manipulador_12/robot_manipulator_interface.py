
from random import randint
from time import sleep

from matplotlib import axes
import rclpy
from rclpy.node import Node
from pynput import keyboard
from std_msgs.msg import Float32MultiArray

from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from tkinter import Tk, Frame,Button,Label, ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from geometry_msgs.msg import Vector3
from threading import Thread
import threading

#Este es el nodo para la interfaz del manipulador


class Robot_Manipulator_Interface(Node):

# ----------------------------------------------------- CLASE --------------------------------------------------------------

    # ----------------------------------------------------- INIT y GRÁFICA --------------------------------------------------------------


    def __init__(self):
        super().__init__('robot_manipulator_teleop')
        print("Inicio del nodo que sirve para visualizar el manipulador.")
        self.subscription = self.create_subscription(Vector3, 'robot_manipulator_position', self.listener_callback, 10)


        fig = plt.figure(figsize=(10,10))
        global ax
        ax = plt.axes(projection="3d")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_xlim(-10,10)
        ax.set_ylim(-10,10)
        ax.set_zlim(-10,10)

        global ventana
        ventana = Tk()
        ventana.geometry('1025x1125')
        ventana.wm_title('Grafica para visualizar la posición del End-Effector')
        ventana.minsize(width=1025,height=1125)

        frame = Frame(ventana,  bg='gray22',bd=3)
        frame.grid(column=0,row=0)

        global canvas
        canvas = FigureCanvasTkAgg(fig, master = frame)  # Crea el area de dibujo en Tkinter
        canvas.get_tk_widget().grid(column=0, row=0, columnspan=3, padx=5, pady =5)
        Button(frame, text='Iniciar', width = 15, bg='magenta',fg='white', command= self.inicio).grid(column=0, row=1, pady =5)

        style = ttk.Style()
        style.configure("Horizontal.TScale", background= 'gray22')  


    # ----------------------------------------------------- CALLBACK (Actualizar gráfica) --------------------------------------------------------------


    def inicio(self):
        print ("Boton de inicio presionado")
        thread = threading.Thread(target=rclpy.spin(self))
        thread.start()

    def listener_callback(self, msg):
        print("Llego el mensaje: "+ str(msg)+ "\n")

        x = msg.x
        y = msg.y
        z = msg.z
        print(x)
        print(y)
        print(z)
        ax.scatter3D(x,y,z)
        canvas.draw()  

    
        
           

# --------------------------------------------------------MAIN-----------------------------------------------------------



def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_interface = Robot_Manipulator_Interface()
    Robot_Manipulator_Interface.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()