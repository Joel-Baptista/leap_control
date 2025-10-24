import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import csv
import os
from datetime import datetime

class ReadPositions(Node):
    def __init__(self):
        super().__init__('save_data')

        # Criar diretório se não existir
        directory = "/home/beatrix/ros2_ws/src/leap_control/leap_hand_control/leap_hand_control/data/hand/"
        os.makedirs(directory, exist_ok=True)

        # Criar timestamp para o nome do ficheiro
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(directory, f"data_{timestamp}.csv")

        # Abrir ficheiro CSV uma única vez
        self.file = open(self.csv_filename, mode="w", newline="")
        self.writer = csv.writer(self.file)

        # Escrever cabeçalho
        self.writer.writerow(["Timestamp", "FingerID", "Pos1", "Vel1", "Curr1", "Pos2", 
                              "Vel2", "Curr2", "Pos3", "Vel3", "Curr3", "Pos4", "Vel4", "Curr4"])

        # Criar subscrições
        self.thumb_subscription = self.create_subscription(
            Float32MultiArray,
            '/thumb_data',
            self.thumb_listener_callback,
            10)  

        self.middle_subscription = self.create_subscription(
            Float32MultiArray,
            '/middle_data',
            self.middle_listener_callback,
            10)  

        self.start_time = self.get_clock().now()

    def thumb_listener_callback(self, msg):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        data_row = [elapsed_time, 3] + list(msg.data[:-1])  # Descarta o último valor de msg.data
        self.writer.writerow(data_row)

    def middle_listener_callback(self, msg):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        data_row = [elapsed_time, 1] + list(msg.data[:-1])  # Descarta o último valor de msg.data
        print(msg.data)
        self.writer.writerow(data_row)

def main(args=None):
    rclpy.init(args=args)
    node = ReadPositions()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando nó.")
    finally:
        node.file.close()  # Fecha o ficheiro corretamente
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
