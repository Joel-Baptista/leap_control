import rclpy
from rclpy.node import Node
from dynamixel_sdk import *  # Biblioteca Dynamixel SDK
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from datetime import datetime
import time



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
GOAL_CURRENT_VALUE = 500

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


class Finger:
    def __init__(self, name, factor, motor_factors):
        self.name = name
        self.factor = factor  # Velocidade relativa do dedo
        self.motor_factors = motor_factors  # Velocidades relativas dos motores do dedo

    def get_motor_speed(self, motor_name):
        return PROFILE_VELOCITY_VALUE * self.factor * self.motor_factors.get(motor_name, 1.0)
    
    def get_finger_speed(self):
        return PROFILE_VELOCITY_VALUE * self.factor



# Lista de IDs dos motores
MOTOR_IDS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,12,13,14,15] 

class DynamixelReader(Node):
    def __init__(self):
        super().__init__('manager_node')

        # Criar os dedos com as respetivas configurações
        self.fingers = [
            Finger("index", 1, {"0": 1, "1": 1, "2": 1, "3": 1}),   # finger_0
            Finger("middle", 1, {"4": 1, "5": 1, "6": 1, "7": 1}),  # finger_1
            Finger("ring", 1, {"8": 1, "9": 1, "10": 1, "11": 1}), # finger_2
            Finger("thumb", 1, {"12": 1, "13": 1, "14": 1, "15": 1}) # finger_3
        ]

        #topicos para enviar os dados dos motores para posterior analise
        self.publisher_position = self.create_publisher(Int32MultiArray, '/dynamixel_finger_positions', 10)
        self.publisher_velocity = self.create_publisher(Int32MultiArray, '/dynamixel_finger_velocities', 10)
        self.publisher_current = self.create_publisher(Int32MultiArray, '/dynamixel_finger_currents', 10)
        
        #topicos para enviar os dados de cada dedo
        self.publisher_middle = self.create_publisher(Float32MultiArray, '/middle_data', 10)
        self.publisher_ring = self.create_publisher(Float32MultiArray, '/ring_data', 10)
        self.publisher_thumb = self.create_publisher(Float32MultiArray, '/thumb_data', 10)
        self.publisher_index = self.create_publisher(Float32MultiArray, '/index_data', 10)


        self.subscription = self.create_subscription(Int32MultiArray, '/set_fingers_positions', self.set_motor_positions, 2000)
        self.subscription = self.create_subscription(Int32MultiArray, '/set_fingers_currents', self.set_currents, 2000)

        # Inicializar comunicação com Dynamixel
        self.port_handler = PortHandler(PORT_NAME)
        self.port_handler.setPacketTimeout(0.5) 
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.group_bulk_read = GroupBulkRead(self.port_handler, self.packet_handler)
        self.group_bulk_write = GroupBulkWrite(self.port_handler, self.packet_handler)

        self.time_last_vel = self.get_clock().now()


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
        
        #lista com os motores detetados para verificar quais os dedos que estão conectados
        #no futuro devera ser uma lista com os ids de 0 a 15
        self.present_motors = []
        
        # deteta quais os motores que estao conectados
        for motor_id in MOTOR_IDS:
            dxl_model_number, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, motor_id)
            if dxl_comm_result == COMM_SUCCESS:
                #self.get_logger().info(f'Motor {motor_id} detectado - Modelo: {dxl_model_number}')
                self.present_motors.append(motor_id)


        # Adicionar motores ao Bulk Read
        for motor_id in self.present_motors:
            #desativar o torque para alterar o modo de operação e realizar as configurações iniciais
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, TORQUE_ENABLE, 0)
            #self.group_bulk_read.addParam(motor_id, ADDR_PRESENT_CURRENT, TOTAL_LENGTH)
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_OPERATING_MODE, CURRENT_BASED_POSITION_MODE)
            self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, ADDR_PROFILE_VELOCITY, self.get_motor_speed(motor_id))
            self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, ADDR_GOAL_CURRENT, GOAL_CURRENT_VALUE)
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, TORQUE_ENABLE, 1)
        
        # SyncRead para posição, velocidade e corrente
        self.group_sync_read = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_PRESENT_CURRENT, TOTAL_LENGTH)

        # Adiciona os 16 motores
        for motor_id in self.present_motors:
            dxl_addparam_result = self.group_sync_read.addParam(motor_id)
            if not dxl_addparam_result:
                print(f"[ERROR] Falha ao adicionar motor {motor_id} ao SyncRead")


        # Criar um Timer para ler com frequencia de 2khz
        self.timer = self.create_timer(0.005, self.read_motors)

    def read_motors(self):
        """Função para ler os motores e publicar no ROS 2"""

        #comm_result = self.group_bulk_read.txRxPacket()
        dxl_comm_result = self.group_sync_read.txRxPacket()
        # if comm_result != COMM_SUCCESS:
        #     error_msg = self.packet_handler.getTxRxResult(comm_result)
        #     self.get_logger().error(f"Erro ao ler os motores: {error_msg}")
        #     #self.get_logger().error("Erro ao ler os motores.")
        #     return
        self.time_now = self.get_clock().now()
        time_diff = (self.time_now - self.time_last_vel).nanoseconds / 1e9
        positions, velocities, currents = [], [], []
        index_data = []
        middle_data = []
        ring_data = []
        thumb_data = []

        for motor_id in self.present_motors:
            #cur = self.group_bulk_read.getData(motor_id, ADDR_PRESENT_CURRENT, CURR_LENGTH)
            cur = self.group_sync_read.getData(motor_id, ADDR_PRESENT_CURRENT, CURR_LENGTH)
            cur = int(np.int16(cur))
            #vel = self.group_bulk_read.getData(motor_id, ADDR_PRESENT_VELOCITY, VEL_LENGTH)
            vel = self.group_sync_read.getData(motor_id, ADDR_PRESENT_VELOCITY, VEL_LENGTH)
            vel = int(np.int32(vel))
            #pos = self.group_bulk_read.getData(motor_id,ADDR_PRESENT_POSITION,POS_LENGTH)
            pos = self.group_sync_read.getData(motor_id, ADDR_PRESENT_POSITION, POS_LENGTH)
            #print(f"Motor {motor_id} | Pos: {pos} | Vel: {vel} | Corrente: {cur}")
            if pos is None or vel is None or cur is None:
                self.get_logger().warn(f'Falha na leitura do motor {motor_id}')
                continue
            else:
                if motor_id <= 3:
                    index_data.extend([pos,vel,cur])
                elif motor_id > 3 and motor_id < 8:
                    middle_data.extend([pos,vel,cur])
                elif motor_id >= 8 and motor_id < 12:
                    ring_data.extend([pos,vel,cur])
                else:
                    thumb_data.extend([pos,vel,cur])
                
            positions.append(pos)
            velocities.append(vel)
            currents.append(cur)
        
        if positions and velocities and currents:
            self.publisher_position.publish(Int32MultiArray(data=positions))
            self.publisher_velocity.publish(Int32MultiArray(data=velocities))
            self.publisher_current.publish(Int32MultiArray(data=currents))

            self.publisher_index.publish(Float32MultiArray(data = index_data+[time_diff]))
            self.publisher_middle.publish(Float32MultiArray(data = middle_data+[time_diff]))
            self.publisher_ring.publish(Float32MultiArray(data = ring_data+[time_diff]))
            self.publisher_thumb.publish(Float32MultiArray(data = thumb_data+[time_diff]))
            #self.get_logger().info(f'Time: {time_diff}')
        self.time_last_vel = self.time_now
        
    
    def set_motor_positions(self, msg):
        """Define as posições dos motores ao receber mensagem no tópico."""
        
        if len(msg.data) % 5 != 0:
            self.get_logger().error("Formato incorreto da mensagem recebida.")
            return

        self.group_bulk_write.clearParam()
        

        for i in range(0, len(msg.data), 5):
            finger = msg.data[i]  # Índice do dedo
            positions = msg.data[i + 1:i + 5]  # Quatro posições para os motores desse dedo

            if len(positions) != 4:
                self.get_logger().error(f"Número incorreto de posições recebidas para o dedo {finger}.")
                continue

            for motor_id, pos in zip(range(4), positions):
                param_goal_position = [
                    DXL_LOBYTE(DXL_LOWORD(pos)), 
                    DXL_HIBYTE(DXL_LOWORD(pos)),
                    DXL_LOBYTE(DXL_HIWORD(pos)), 
                    DXL_HIBYTE(DXL_HIWORD(pos))
                ]

                add_success_pos = self.group_bulk_write.addParam(
                    (motor_id + 4 * finger), ADDR_GOAL_POSITION, 4, param_goal_position
                )

                if not add_success_pos:
                    self.get_logger().error(f'Erro ao adicionar motor {motor_id + 4 * finger} ao Bulk Write')

        dxl_comm_result = self.group_bulk_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error('Erro ao enviar posições para os motores')
        else:
            self.get_logger().info('Posições enviadas com sucesso')


        self.group_bulk_write.clearParam()
    
    def set_currents(self, msg):
        """Define as correntes dos motores ao receber mensagem no tópico."""
        currents = msg.data[1:5]
        finger = msg.data[0]

        if len(currents) != 4:
            self.get_logger().info(f'Correntes recebidas:{currents}')
            self.get_logger().error("Número incorreto de posições recebidas.")
            return

        self.group_bulk_write.clearParam()
        for motor_id,curr in zip(range(4),currents):
            param_goal_curr = [DXL_LOBYTE(DXL_LOWORD(curr)), DXL_HIBYTE(DXL_LOWORD(curr))]
            add_success_curr = self.group_bulk_write.addParam((motor_id+4*finger), ADDR_GOAL_CURRENT, 2,param_goal_curr)

            if not add_success_curr:
                self.get_logger().error(f'Erro ao adicionar motor {motor_id} ao Bulk Write')

        dxl_comm_result = self.group_bulk_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error('Erro ao enviar correntes para os motores')
        else:
            self.get_logger().info('Correntes enviadas com sucesso')


        self.group_bulk_write.clearParam()

    def get_motor_speed(self,motor_id):
        finger_index = motor_id // 4
        print(self.fingers[finger_index].get_motor_speed(str(motor_id)))
        return self.fingers[finger_index].get_motor_speed(str(motor_id))



    
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
