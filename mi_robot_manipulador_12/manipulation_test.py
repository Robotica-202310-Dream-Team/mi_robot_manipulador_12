import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from proyecto_interfaces.srv import StartManipulationTest
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray, Bool

class Manipulation_test(Node):

    def __init__(self):
        super().__init__('manipulation_test')
        self.srv = self.create_service(StartManipulationTest, 'group_12/start_manipulation_test_srv', self.read_txt_callback)
        self.publisher= self.create_publisher(Bool, 'panel_flag', 50)
        self.msg = Bool()
        print('Servicio para la prueba de manipulación listo')
        

    def read_txt_callback(self, request, response):
        print('Se ha llamado al servicio para la prueba de manipulación')
        plataformaFicha = request.platform
        tipo_ficha = request.x
        if plataformaFicha == "platform_1":
            self.msg.data = True
            self.publisher.publish(self.msg)
            plataformaOrigen = 1
            plataformaDestino = 2
        elif plataformaFicha == "platform_2":
            self.msg.data = False
            self.publisher.publish(self.msg)
            plataformaOrigen = 2
            plataformaDestino = 1
        response.answer = "La ficha de tipo "+str(tipo_ficha)+" se encuentra en la plataforma "+str(plataformaOrigen)+" y la llevaré a la plataforma "+ str(plataformaDestino)+"."
        return response



def main(args=None):
    rclpy.init(args=args)
    manipulation_test = Manipulation_test()
    rclpy.spin(manipulation_test)
    manipulation_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()