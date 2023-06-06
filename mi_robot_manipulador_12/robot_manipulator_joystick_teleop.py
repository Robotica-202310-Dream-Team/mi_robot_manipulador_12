import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import pygame
import time

class Joystick_Publisher(Node):
    _axis_moved = False
    def __init__(self):
        super().__init__('robot_manipulator_joystick_teleop')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'manipulator_cmdVel', 10)
        self.msg = Float32MultiArray()
        self.msg.data = [0.0, 120.0, 245.0, 0.0]
        self.open = 0
        timer_period = 0.01  # seconds
        pygame.init()
        pygame.joystick.init()
        print('Esperando joystick...')
        joystick_ref = pygame.joystick.Joystick(0) # Se crea la referencia al joystick
        joystick_ref.init()# Se inicializa el joystick de esa referencia
        
        joystick_ref.get_button(1)
        self.timer = self.create_timer(timer_period, lambda : self.timer_callback(msg=self.msg, joystick_ref=joystick_ref))
        
    def timer_callback(self, msg, joystick_ref):
        self.empty_event_queue()
        if self._axis_moved:
            axis3 = joystick_ref.get_axis(3)
            angle1 = round ((axis3)*-90 +360 )
            if angle1 > 360:
                angle1 -= 360
            angle2 = round (axis3*-65 +65)
            angle3 = round ((axis3)*-90 +335 )
            if angle3 > 360:
                angle3 -= 360
            boton_joint1 = joystick_ref.get_button(4)
            boton_joint2 = joystick_ref.get_button(2)
            boton_joint3 = joystick_ref.get_button(3)
            boton_recoger = joystick_ref.get_button(10)
            boton_soltar = joystick_ref.get_button(11)
            boton_abrir_end_effector = joystick_ref.get_button(0)
            boton_cerrar_end_effector = joystick_ref.get_button(1)
            if boton_joint1:
                self.msg.data[0] = angle1
            elif boton_joint2:
                self.msg.data[1] = angle2 
            elif boton_joint3:
                self.msg.data[2] = angle3 
            ####################################    
            elif boton_abrir_end_effector:
                self.msg.data[3] = 1
            elif boton_cerrar_end_effector:
                self.msg.data[3] = 0
            ###################################     
            elif boton_recoger:
                self.recoger(msg)
            elif boton_soltar:
                self.soltar(msg)
            
            self._axis_moved = False
            self.publisher_.publish(msg)
        print (f"Mensaje: {self.msg.data}")

    def recoger(self,msg):
        print("Recogiendo")
        self.msg.data[0] = 0
        self.msg.data[1] = 120
        self.msg.data[2] = 245
        self.msg.data[3] = 1
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ###########################
        self.msg.data[0] = 0
        self.msg.data[1] = 120
        self.msg.data[2] = 275
        self.msg.data[3] = 1
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ##########################
        self.msg.data[0] = 0
        self.msg.data[1] = 76
        self.msg.data[2] = 275
        self.msg.data[3] = 1
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        self.msg.data[0] = 0
        self.msg.data[1] = 76
        self.msg.data[2] = 275
        self.msg.data[3] = 0
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        self.msg.data[0] = 0
        self.msg.data[1] = 120
        self.msg.data[2] = 275
        self.msg.data[3] = 0
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        self.msg.data[0] = 0
        self.msg.data[1] = 120
        self.msg.data[2] = 245
        self.msg.data[3] = 0
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        print("Ya recogió")

    def soltar(self,msg):
        print("Soltando")
        self.msg.data[0] = 0
        self.msg.data[1] = 120
        self.msg.data[2] = 245
        self.msg.data[3] = 0
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ###########################
        self.msg.data[0] = 0
    
        self.msg.data[1] = 120
        self.msg.data[2] = 275
        self.msg.data[3] = 0
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ##########################
        self.msg.data[0] = 0
        self.msg.data[1] = 76
        self.msg.data[2] = 275
        self.msg.data[3] = 0
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        self.msg.data[0] = 0
        self.msg.data[1] = 76
        self.msg.data[2] = 275
        self.msg.data[3] = 1
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        self.msg.data[0] = 0
        self.msg.data[1] = 120
        self.msg.data[2] = 275
        self.msg.data[3] = 1
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        self.msg.data[0] = 0
        self.msg.data[1] = 120
        self.msg.data[2] = 245
        self.msg.data[3] = 1
        self.publisher_.publish(msg)
        time.sleep(0.5)
        ######################
        print("Ya soltó")

    def empty_event_queue(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self._axis_moved = True
            elif event.type == pygame.JOYBALLMOTION:
                pass
            elif event.type == pygame.JOYHATMOTION:
                pass
            elif event.type == pygame.JOYBUTTONDOWN:
                pass
            elif event.type == pygame.JOYBUTTONUP:
                pass



def main():
    rclpy.init()
    joystick_publisher = Joystick_Publisher()
    rclpy.spin(joystick_publisher)
    joystick_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
