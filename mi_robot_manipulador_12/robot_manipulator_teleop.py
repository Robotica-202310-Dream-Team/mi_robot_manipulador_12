
from time import sleep
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from pynput import keyboard
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist

#Este es el nodo para las teclas


class Robot_Manipulator_Teleop(Node):

    # -----------------------------------------------------INIT--------------------------------------------------------------


    def __init__(self):
        super().__init__('robot_manipulator_teleop')
        print("Este nodo sirve para mover el manipulador a partir de las teclas j,k,l.")
        self.primera = float(input("Ingrese la velocidad de la primera juntura (0-180): \n"))
        self.segunda = float(input("Ingrese la velocidad de la segunda juntura (0-180): \n"))
        self.tercera = float(input("Ingrese la velocidad de la tercera juntura (0-180): \n"))
        while self.primera > 180 or self.primera < 0 or self.segunda > 180 or self.segunda < 0 or self.tercera > 180 or self.tercera < 0:
            print("Los valores ingresados exceden el rango permitido. Ingrese los valores nuevamente. \n")
            self.primera = float(input("Ingrese la velocidad de la primera juntura (0-180): \n"))
            self.segunda = float(input("Ingrese la velocidad de la segunda juntura (0-180): \n"))
            self.tercera = float(input("Ingrese la velocidad de la tercera juntura (0-180): \n"))


        self.publisher = self.create_publisher(Float32MultiArray, 'manipulador_cmdVel', 10)
        self.msg = Float32MultiArray()
        self.msg.data = [0.0, 0.0, 0.0]
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
       

    # -----------------------------------------------------KEYBOARD THREAD--------------------------------------------------------------

    def on_press(self,key):
        try:    
            print('alphanumeric key {0} pressed'.format(key.char))
            if key.char =='j':
                self.msg.data[0] = self.primera
                self.msg.data[1] = 0
                self.msg.data[2] = 0
            elif key.char == 'k':
                self.msg.data[0] = 0
                self.msg.data[1] = self.segunda
                self.msg.data[2] = 0
            elif key.char == 'l':
                self.msg.data[0] = 0
                self.msg.data[1] = 0
                self.msg.data[2] = self.tercera
            self.publisher.publish(self.msg)
        except AttributeError:
            try:
                print('special key {0} pressed'.format(key))
                if key.char =='j':
                    self.msg.data[0] = self.primera
                    self.msg.data[1] = 0
                    self.msg.data[2] = 0
                elif key.char == 'k':
                    self.msg.data[0] = 0
                    self.msg.data[1] = self.segunda
                    self.msg.data[2] = 0
                elif key.char == 'l':
                    self.msg.data[0] = 0
                    self.msg.data[1] = 0
                    self.msg.data[2] = self.tercera
                self.publisher.publish(self.msg)
            except: 
                pass


    def on_release(self,key):
        try:
            print('{0} released'.format(key))
            self.msg.data[0] = 0
            self.msg.data[1] = 0
            self.msg.data[2] = 0
            self.publisher.publish(self.msg)
            if key == keyboard.Key.esc:
                return False
        except AttributeError:
            if key == keyboard.Key.esc:
                return False
          

# --------------------------------------------------------MAIN-----------------------------------------------------------



def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_teleop = Robot_Manipulator_Teleop()
    rclpy.spin(robot_manipulator_teleop)
    Robot_Manipulator_Teleop.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()