import rclpy
from rclpy.node import Node
from dynamixel_sdk import *  # Biblioteca Dynamixel SDK
import numpy as np
from std_msgs.msg import Int32MultiArray



# Parâmetros de comunicação
PORT_NAME = "/dev/ttyUSB0"  # Ajusta conforme necessário
BAUDRATE = 4000000
PROTOCOL_VERSION = 2.0

# Endereços dos dados
TORQUE_ENABLE = 64

ADDR_GOAL_POSITION = 116
ADDR_GOAL_CURRENT = 102
ADDR_OPERATING_MODE = 11 #endereço do modo de operação

CURRENT_BASED_POSITION_MODE = 5 

ADDR_CURRENT_LIMIT = 38 #endereço do limite de corrente
CURRENT_LIMIT_VALUE = 1800
GOAL_CURRENT_VALUE = 200

ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132  # Endereço da posição atual
ADDR_PRESENT_VELOCITY = 128  # Endereço da velocidade atual
TOTAL_LENGTH = 10
VEL_LENGTH = 4
POS_LENGTH = 4
CURR_LENGTH = 2

ADDR_INDIRECT_START = 168

ADDR_PROFILE_VELOCITY = 112
PROFILE_VELOCITY_VALUE = 50


# Lista de IDs dos motores
MOTOR_IDS = [1, 2, 3, 4]  # Ajusta conforme necessário


class DynamixelReader(Node):
    def __init__(self):
        super().__init__('dynamixel_reader')

        self.publisher_position = self.create_publisher(Int32MultiArray, '/dynamixel_finger_positions', 10)
        self.publisher_velocity = self.create_publisher(Int32MultiArray, '/dynamixel_finger_velocities', 10)
        self.publisher_current = self.create_publisher(Int32MultiArray, '/dynamixel_finger_currents', 10)
        self.subscription = self.create_subscription(Int32MultiArray, '/dynamixel_set_finger_positions', self.set_motor_positions, 2000)

        # Inicializar comunicação com Dynamixel
        self.port_handler = PortHandler(PORT_NAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.group_bulk_read = GroupBulkRead(self.port_handler, self.packet_handler)
        self.group_bulk_write = GroupBulkWrite(self.port_handler, self.packet_handler)

        if self.port_handler.openPort() and self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().info("Conexão com Dynamixel estabelecida.")
        else:
            self.get_logger().error("Falha ao conectar com Dynamixel.")
            return


        # Abrir porta
        if self.port_handler.openPort() and self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().info("Conexão com Dynamixel estabelecida.")
        else:
            self.get_logger().error("Falha ao conectar com Dynamixel.")
            return
        

        # Adicionar motores ao Bulk Read
        for motor_id in MOTOR_IDS:
            #desativar o torque para alterar o modo de operação e realizar as configurações iniciais
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, TORQUE_ENABLE, 0)
            self.group_bulk_read.addParam(motor_id, ADDR_PRESENT_CURRENT, TOTAL_LENGTH)
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_OPERATING_MODE, CURRENT_BASED_POSITION_MODE)
            self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY_VALUE)
            self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, ADDR_GOAL_CURRENT, GOAL_CURRENT_VALUE)
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, TORQUE_ENABLE, 1)

        # Criar um Timer para ler com frequancia de 2khz
        self.timer = self.create_timer(0.0005, self.read_motors)

    def read_motors(self):
        """Função para ler os motores e publicar no ROS 2"""
        if self.group_bulk_read.txRxPacket() != COMM_SUCCESS:
            self.get_logger().error("Erro ao ler os motores.")
            return
        positions, velocities, currents = [], [], []

        for motor_id in MOTOR_IDS:
            cur = self.group_bulk_read.getData(motor_id, ADDR_PRESENT_CURRENT, CURR_LENGTH)
            cur = int(np.int16(cur))
            vel = self.group_bulk_read.getData(motor_id, ADDR_PRESENT_VELOCITY, VEL_LENGTH)
            vel = int(np.int32(vel))
            pos = self.group_bulk_read.getData(motor_id,ADDR_PRESENT_POSITION,POS_LENGTH)

            if pos is None or vel is None or cur is None:
                self.get_logger().warn(f'Falha na leitura do motor {motor_id}')
                continue


            positions.append(pos)
            velocities.append(vel)
            currents.append(cur)
        
        if positions:
            self.publisher_position.publish(Int32MultiArray(data=positions))
        if velocities:
            self.publisher_velocity.publish(Int32MultiArray(data=velocities))
        if currents:
            self.publisher_current.publish(Int32MultiArray(data=currents))
    
    def set_motor_positions(self, msg):
        """Define as posições dos motores ao receber mensagem no tópico."""
        positions = msg.data
        if len(positions) != len(MOTOR_IDS):
            self.get_logger().error("Número incorreto de posições recebidas.")
            return

        self.group_bulk_write.clearParam()
        for motor_id,pos in zip(MOTOR_IDS,positions):
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(pos)), DXL_HIBYTE(DXL_LOWORD(pos)),
                                   DXL_LOBYTE(DXL_HIWORD(pos)), DXL_HIBYTE(DXL_HIWORD(pos))]
            add_success_pos = self.group_bulk_write.addParam(motor_id, ADDR_GOAL_POSITION, 4,param_goal_position)

            if not add_success_pos:
                self.get_logger().error(f'Erro ao adicionar motor {motor_id} ao Bulk Write')

        dxl_comm_result = self.group_bulk_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error('Erro ao enviar posições para os motores')
        else:
            self.get_logger().info('Posições enviadas com sucesso')


        self.group_bulk_write.clearParam()


    
    def destroy_node(self):
        """Fechar a porta ao encerrar o nó"""
        self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
