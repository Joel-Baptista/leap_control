#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_sdk import *  # Biblioteca Dynamixel SDK
from std_msgs.msg import Int32
import numpy as np  # Para conversão correta de valores

# Configurações do Dynamixel
MY_DXL = "X_SERIES"
ADDR_PRESENT_VELOCITY = 128
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
DXL_ID = 3
DEVICENAME = "/dev/ttyUSB0"
ADDR_TORQUE_ENABLE = 64  

class ReadVelocityNode(Node):
    """Nó ROS2 para ler continuamente a velocidade do motor Dynamixel e publicá-la"""

    def __init__(self):
        super().__init__("read_velocity_node")
        self.publisher_ = self.create_publisher(Int32, "dynamixel_velocity", 10)
        self.timer = self.create_timer(0.1, self.read_velocity)  # 10 Hz
        
        # Inicializar comunicação com o Dynamixel
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error("Falha ao abrir a porta")
            return
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("Falha ao configurar a baudrate")
            return

        #desativar o torque
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
        self.get_logger().info("Dynamixel conectado!")

    def read_velocity(self):
        """Ler e publicar a velocidade do motor"""
        dxl_present_velocity, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, DXL_ID, ADDR_PRESENT_VELOCITY
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().error(self.packet_handler.getRxPacketError(dxl_error))
        else:
            # Converter para int32 
            dxl_present_velocity_signed = int(np.int32(dxl_present_velocity))
            self.get_logger().info(f"Velocidade Atual: {dxl_present_velocity_signed}")
            self.publisher_.publish(Int32(data=dxl_present_velocity_signed))

    def destroy_node(self):
        """Fechar a porta ao encerrar o nó"""
        self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ReadVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
