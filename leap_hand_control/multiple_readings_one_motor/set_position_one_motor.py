import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class SetPositionNode(Node):
    def __init__(self):
        super().__init__('set_position_node')
        self.publisher = self.create_publisher(Int32, '/dynamixel/set_position', 10)
        self.get_logger().info("Nó set_position ativo. Digite uma posição para publicar.")
        
        self.run()

    def run(self):
        while rclpy.ok():
            try:
                position = int(input("Digite a posição desejada: "))
                self.publish_position(position)
            except ValueError:
                print("Por favor, insira um número inteiro válido.")
            except KeyboardInterrupt:
                print("Encerrando nó.")
                break
    
    def publish_position(self, position):
        msg = Int32()
        msg.data = position
        self.publisher.publish(msg)
        self.get_logger().info(f'Enviando posição: {position}')

def main(args=None):
    rclpy.init(args=args)
    node = SetPositionNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
