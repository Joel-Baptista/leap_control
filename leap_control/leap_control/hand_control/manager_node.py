import rclpy
from rclpy.node import Node
from dynamixel_sdk import *  # Biblioteca Dynamixel SDK
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from datetime import datetime
import time

class Finger:
    def __init__(self, name, factor, motor_factors, profile_vel_value):
        self.name = name
        self.factor = factor  # Velocidade relativa do dedo
        self.motor_factors = motor_factors  # Velocidades relativas dos motores do dedo
        self.profile_vel_value = profile_vel_value
    def get_motor_speed(self, motor_name):
        return self.profile_vel_value * self.factor * self.motor_factors.get(motor_name, 1.0)
    
    def get_finger_speed(self):
        return self.profile_vel_value * self.factor

class DynamixelReader(Node):
    def __init__(self):
        super().__init__('manager_node')

        self.port_name = self.get_rosparam('port_name', '/dev/ttyUSB0')
        self.baudrate = self.get_rosparam('baudrate', 4000000)
        self.protocol_version = self.get_rosparam('protocol_version', 2.0)
        self.torque_enable = self.get_rosparam('torque_enable', 64)
        self.addr_goal_position = self.get_rosparam('addr_goal_position', 116)
        self.addr_goal_current = self.get_rosparam('addr_goal_current', 102)
        self.addr_operating_mode = self.get_rosparam('addr_operating_mode', 11)
        self.current_based_position_mode = self.get_rosparam('current_based_position_mode', 5)
        self.addr_current_limit = self.get_rosparam('addr_current_limit', 38)
        self.current_limit_value = self.get_rosparam('current_limit_value', 1800)
        self.goal_current_value = self.get_rosparam('goal_current_value', 500)
        self.addr_present_current = self.get_rosparam('addr_present_current', 126)
        self.addr_present_position = self.get_rosparam('addr_present_position', 132)
        self.addr_present_velocity = self.get_rosparam('addr_present_velocity', 128)
        self.total_length = self.get_rosparam('total_length', 10)
        self.vel_length = self.get_rosparam('vel_length', 4) 
        self.pos_length = self.get_rosparam('pos_length', 4) 
        self.curr_length = self.get_rosparam('curr_length', 2) 
        self.addr_indirect_start = self.get_rosparam('addr_indirect_start', 168)
        self.addr_profile_velocity = self.get_rosparam('addr_profile_velocity', 112)
        self.profile_velocity_value = self.get_rosparam('profile_velocity_value', 50)

        self.motor_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,12,13,14,15] 
        
        # Criar os dedos com as respetivas configurações
        self.fingers = [
            Finger("index", 1, {"0": 1, "1": 1, "2": 1, "3": 1}, self.profile_velocity_value),   # finger_0
            Finger("middle", 1, {"4": 1, "5": 1, "6": 1, "7": 1}, self.profile_velocity_value),  # finger_1
            Finger("ring", 1, {"8": 1, "9": 1, "10": 1, "11": 1}, self.profile_velocity_value), # finger_2
            Finger("thumb", 1, {"12": 1, "13": 1, "14": 1, "15": 1}, self.profile_velocity_value) # finger_3
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
        self.port_handler = PortHandler(self.port_name)
        self.port_handler.setPacketTimeout(0.5) 
        self.packet_handler = PacketHandler(self.protocol_version)
        self.group_bulk_read = GroupBulkRead(self.port_handler, self.packet_handler)
        self.group_bulk_write = GroupBulkWrite(self.port_handler, self.packet_handler)

        self.time_last_vel = self.get_clock().now()


        if self.port_handler.openPort() and self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().info("Conexão com Dynamixel estabelecida.")
        else:
            self.get_logger().error("Falha ao conectar com Dynamixel.")
            return

        # # Abrir porta
        if self.port_handler.openPort() and self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().info("Conexão com Dynamixel estabelecida.")
        else:
            self.get_logger().error("Falha ao conectar com Dynamixel.")
            return
        
        #lista com os motores detetados para verificar quais os dedos que estão conectados
        #no futuro devera ser uma lista com os ids de 0 a 15
        self.present_motors = []
        
        # deteta quais os motores que estao conectados
        for motor_id in self.motor_ids:
            dxl_model_number, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, motor_id)
            if dxl_comm_result == COMM_SUCCESS:
                #self.get_logger().info(f'Motor {motor_id} detectado - Modelo: {dxl_model_number}')
                self.present_motors.append(motor_id)


        # Adicionar motores ao Bulk Read
        for motor_id in self.present_motors:
            #desativar o torque para alterar o modo de operação e realizar as configurações iniciais
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.torque_enable, 0)
            #self.group_bulk_read.addParam(motor_id, ADDR_PRESENT_CURRENT, TOTAL_LENGTH)
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.addr_operating_mode, self.current_based_position_mode)
            self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, self.addr_profile_velocity, self.get_motor_speed(motor_id))
            self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, self.addr_goal_current, self.goal_current_value)
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.torque_enable, 1)
        
        # SyncRead para posição, velocidade e corrente
        self.group_sync_read = GroupSyncRead(self.port_handler, self.packet_handler, self.addr_present_current, self.total_length)

        # Adiciona os 16 motores
        for motor_id in self.present_motors:
            dxl_addparam_result = self.group_sync_read.addParam(motor_id)
            if not dxl_addparam_result:
                print(f"[ERROR] Falha ao adicionar motor {motor_id} ao SyncRead")


        # Criar um Timer para ler com frequencia de 2khz
        self.timer = self.create_timer(0.005, self.read_motors)

    def get_rosparam(self, parameter_name, default_value):
        self.declare_parameter(parameter_name, default_value)
        parameter = self.get_parameter(parameter_name).value
        self.get_logger().info(f"Starting finger manager for: {parameter}")

        return parameter

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
            cur = self.group_sync_read.getData(motor_id, self.addr_present_current, self.curr_length)
            cur = int(np.int16(cur))
            #vel = self.group_bulk_read.getData(motor_id, ADDR_PRESENT_VELOCITY, VEL_LENGTH)
            vel = self.group_sync_read.getData(motor_id, self.addr_present_velocity, self.vel_length)
            vel = int(np.int32(vel))
            #pos = self.group_bulk_read.getData(motor_id,ADDR_PRESENT_POSITION,POS_LENGTH)
            pos = self.group_sync_read.getData(motor_id, self.addr_present_position, self.pos_length)
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
                    (motor_id + 4 * finger), self.addr_goal_position, 4, param_goal_position
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
            add_success_curr = self.group_bulk_write.addParam((motor_id+4*finger), self.addr_goal_current, 2,param_goal_curr)

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
