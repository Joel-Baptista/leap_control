# fsr_sensor_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class FSRSensorReader(Node):
    def __init__(self):
        super().__init__('fsr_sensor_node')

        self.port_name = self.get_rosparam("port_name", "/dev/ttyUSB0")
        self.baudrate = self.get_rosparam("baudrate", 4000000)

        self.publisher_ = self.create_publisher(Float32MultiArray, 'fsr_touch_sensor', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200)
        self.timer = self.create_timer(0.001, self.read_fsr_data)

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

                self.get_logger().info(f'Publicado e guardado: {values}')
            except ValueError:
                self.get_logger().warn(f'Valor inválido: {line}')

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FSRSensorReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
