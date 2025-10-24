#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_sdk import *  # Biblioteca Dynamixel SDK
from std_msgs.msg import Int32
import numpy as np

# Configurações do Dynamixel
MY_DXL = "X_SERIES"
ADDR_PRESENT_PWM = 124  # Endereço para leitura do PWM
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
DXL_ID = 3
DEVICENAME = '/dev/ttyUSB0'
ADDR_TORQUE_ENABLE = 64  



class ReadPWMNode(Node):
    """Nó ROS2 para ler continuamente o PWM do motor Dynamixel e publicá-lo"""

    def __init__(self):
        super().__init__('read_pwm_node')
        self.publisher_ = self.create_publisher(Int32, 'dynamixel_pwm', 10)
        self.timer = self.create_timer(0.1, self.read_pwm)  # 10 Hz
        
        # Inicializar comunicação com o Dynamixel
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error("Falha ao abrir a porta")
            return
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("Falha ao configurar a baudrate")
            return
        
        #ativar o torque
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

        self.get_logger().info("Dynamixel conectado!")


    def read_pwm(self):
        """Ler e publicar o PWM do motor"""
        dxl_present_pwm, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, DXL_ID, ADDR_PRESENT_PWM)

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().error(self.packet_handler.getRxPacketError(dxl_error))
        else:
            dxl_present_pwm = int(np.int16(dxl_present_pwm)) 
            self.get_logger().info(f"PWM Atual: {dxl_present_pwm}")
            self.publisher_.publish(Int32(data=dxl_present_pwm))

    def destroy_node(self):
        """Fechar a porta ao encerrar o nó"""
        self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ReadPWMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
