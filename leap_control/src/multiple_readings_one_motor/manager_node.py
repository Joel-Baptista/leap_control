import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from dynamixel_sdk import *  # Biblioteca Dynamixel SDK

# Configuração do motor
DXL_ID = 3  # ID do motor (altere conforme necessário)
BAUDRATE = 57600
DEVICENAME = "/dev/ttyUSB0"
PROTOCOL_VERSION = 2.0
TORQUE_ENABLE = 64  

# Endereços para Indirect Address
ADDR_INDIRECT_START = 168
ADDR_INDIRECT_DATA_START = 208  # Local onde os dados mapeados aparecem

# Endereços reais dos dados que queremos mapear
ADDR_PRESENT_POSITION = 132  # 4 bytes
ADDR_PRESENT_VELOCITY = 128  # 4 bytes
ADDR_PRESENT_CURRENT = 126  # 2 bytes

ADDR_GOAL_POSITION = 116

class DynamixelNode(Node):
    def __init__(self):
        super().__init__('dynamixel_node')
        
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        
        if not self.setup_dynamixel():
            self.get_logger().error("Falha ao configurar o Dynamixel.")
            return
        
        self.position_publisher = self.create_publisher(Int32, '/dynamixel/position', 10)
        self.velocity_publisher = self.create_publisher(Int32, '/dynamixel/velocity', 10)
        self.current_publisher = self.create_publisher(Int32, '/dynamixel/current', 10)
        
        self.create_subscription(Int32, '/dynamixel/set_position', self.set_motor_position, 10)
        
        self.timer = self.create_timer(0.1, self.read_dynamixel)  # 10 Hz
        
    def setup_dynamixel(self):
        """ Configura os Indirect Addresses para ler posição e velocidade. """
        if not self.port_handler.openPort():
            self.get_logger().error("Erro ao abrir a porta!")
            return False

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("Erro ao configurar a taxa de baud!")
            return False

        # Desativa o torque antes da configuração
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, TORQUE_ENABLE, 0)

        # Configura Indirect Address para Posição
        self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_INDIRECT_START, ADDR_PRESENT_POSITION)
        self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_INDIRECT_START + 2, ADDR_PRESENT_POSITION + 1)
        self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_INDIRECT_START + 4, ADDR_PRESENT_POSITION + 2)
        self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_INDIRECT_START + 6, ADDR_PRESENT_POSITION + 3)

        # Configura Indirect Address para Velocidade
        self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_INDIRECT_START + 8, ADDR_PRESENT_VELOCITY)
        self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_INDIRECT_START + 10, ADDR_PRESENT_VELOCITY + 1)
        self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_INDIRECT_START + 12, ADDR_PRESENT_VELOCITY + 2)
        self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_INDIRECT_START + 14, ADDR_PRESENT_VELOCITY + 3)

        # Configura Indirect Address para corrente
        self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_INDIRECT_START + 16, ADDR_PRESENT_CURRENT)
        self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_INDIRECT_START + 18, ADDR_PRESENT_CURRENT + 1)

        # Volta a ativar o torque para ser possivel enviar comandos de posiçao e ler a corrente
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, TORQUE_ENABLE, 0)

        return True

    def set_motor_position(self, msg):
        """ Callback que recebe a posição desejada e envia para o motor Dynamixel. """
        position = msg.data
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, DXL_ID, ADDR_GOAL_POSITION, position
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Falha ao enviar posição! Código de erro: {dxl_comm_result}")
        elif dxl_error != 0:
            self.get_logger().error(f"Erro de status do motor: {dxl_error}")

    def read_dynamixel(self):
        """ Lê posição e velocidade do motor usando Indirect Data e publica em tópicos ROS 2. """
        dxl_data, dxl_comm_result, dxl_error = self.packet_handler.readTxRx(
            self.port_handler, DXL_ID, ADDR_INDIRECT_DATA_START, 10
        )

        if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
            pos = int.from_bytes(dxl_data[0:4], byteorder='little', signed=True)
            vel = int.from_bytes(dxl_data[4:8], byteorder='little', signed=True)
            curr = int.from_bytes(dxl_data[8:10], byteorder='little', signed=True)

            self.position_publisher.publish(Int32(data=pos))
            self.velocity_publisher.publish(Int32(data=vel))
            self.current_publisher.publish(Int32(data=curr))

        else:
            self.get_logger().warn(f"Erro na leitura: {dxl_comm_result}, {dxl_error}")

    def shutdown(self):
        """ Fecha a porta do Dynamixel ao encerrar o nó. """
        self.port_handler.closePort()
        self.get_logger().info("Encerrando nó Dynamixel.")


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
