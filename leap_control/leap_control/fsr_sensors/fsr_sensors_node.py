# fsr_sensor_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import csv
import os
from datetime import datetime

class FSRSensorNode(Node):
    def __init__(self):
        super().__init__('fsr_sensor_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'fsr_values', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200)
        self.timer = self.create_timer(0.001, self.read_fsr_data)

        # Criar ficheiro CSV
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        directory = "/home/beatrix/ros2_ws/src/leap_control/leap_hand_control/leap_hand_control/data/fsr_sensors/"
        os.makedirs(directory, exist_ok=True)
        self.csv_filename = os.path.join(directory, f'fsr_data_{timestamp}.csv')
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.start_time = self.get_clock().now()

        # Escrever cabeçalho
        self.csv_writer.writerow(['timestamp', 'FSR1','FSR2','FSR3','FSR4','FSR5'])

    def read_fsr_data(self):
        if self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                values_str = line.split('\t')
                if len(values_str) != 5:
                    self.get_logger().warn(f'Esperados 5 valores, recebido: {line}')
                    return

                values = [float(v) for v in values_str]
                msg = Float32MultiArray()
                msg.data = values
                self.publisher_.publish(msg)

                # Guardar no CSV com timestamp
                #timestamp = datetime.now().isoformat()
                #elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
                #self.csv_writer.writerow([elapsed_time] + values)

                self.get_logger().info(f'Publicado e guardado: {values}')
            except ValueError:
                self.get_logger().warn(f'Valor inválido: {line}')

    def destroy_node(self):
        super().destroy_node()
        self.csv_file.close()
        self.get_logger().info(f'Ficheiro CSV "{self.csv_filename}" guardado com sucesso.')

def main(args=None):
    rclpy.init(args=args)
    node = FSRSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
