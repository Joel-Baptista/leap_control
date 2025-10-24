import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import re
import numpy as np

class SetPositions(Node):
    def __init__(self):
        super().__init__('set_positions')
        self.publisher = self.create_publisher(Int32MultiArray, '/dynamixel_set_finger_positions', 10)
        self.min = np.array([-1.047, -0.314, -0.506, -0.366, -1.047, -0.314, -0.506, -0.366, -1.047, -0.314, -0.506, -0.366, -0.349, -0.47, -1.20, -1.34]) + np.pi #limites minimos de todos os motores para simulação fornecidos peo codigo da LEAP Hand
        self.max = np.array([1.047,    2.23,  1.885,  2.042,  1.047,   2.23,  1.885,  2.042,  1.047,   2.23,  1.885,  2.042,  2.094,  2.443, 1.90,  1.88]) + np.pi #limites maximos de todos os motores para simulação fornecidos peo codigo da LEAP Hand

    def publish_positions(self, positions):
        msg = Int32MultiArray(data=positions)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publicando posições: {positions}')

    def radians_to_dynamixel(self,angle_rad):
        return int((angle_rad * 4095) / (2 * np.pi))

def main(args=None):
    rclpy.init(args=args)
    node = SetPositions()
    
    try:
        while rclpy.ok():
            input_str = input("Introduzir as posições (ex: 2048 2048 2048 2048) ou comando (ex: close): ").strip()
            
            if input_str.lower() == "close":
                positions = list([1800, 2048, 2890, 2700])
                # positions = [node.radians_to_dynamixel(angle) for angle in node.min[3:7]]
                # positions[1] = 2048
            elif input_str.lower() == "open":
                positions = list([1024, 2048, 2048, 2048])
                # positions = [node.radians_to_dynamixel(angle) for angle in node.max[3:7]]
                # positions[1] = 2048
            else:
                input_str = re.sub(r'[^0-9\s]', '', input_str)  # Remove caracteres não numéricos
                positions = list(map(int, input_str.split()))

            node.get_logger().info(f'Minhas posições: {positions}')
            node.publish_positions(positions)

    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

