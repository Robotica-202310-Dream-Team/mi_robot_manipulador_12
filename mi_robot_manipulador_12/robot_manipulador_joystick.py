import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import pygame

class Joystick_Publisher(Node):
    _axis_moved = False
    def __init__(self):
        super().__init__('robot_manipulator_joystick_teleop')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'manipulador_cmdVel', 10)
        self.msg = Float32MultiArray()
        self.msg.data = [0.0, 0.0, 0.0]
        
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
            angle = round (axis3*-90 +90)
            boton_joint1 = joystick_ref.get_button(4)
            #print (f"Joint1: {boton_joint1}")
            boton_joint2 = joystick_ref.get_button(2)
            #print (f"Joint2: {boton_joint2}")
            boton_joint3 = joystick_ref.get_button(3)
            #print (f"Joint3: {boton_joint3}")
            self.msg.data[0] = boton_joint1*angle 
            self.msg.data[1] = boton_joint2*angle
            self.msg.data[2] = boton_joint3*angle
            boton_end_effector = joystick_ref.get_button(0)
            #print (f"End effector: {boton_end_effector}")
            
            print (f"Mensaje: {self.msg.data}")
            self._axis_moved = False
        

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
