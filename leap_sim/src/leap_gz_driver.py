import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from sensor_msgs.msg import JointState
from datetime import datetime
import time

import copy

PI = 3.14159


MOTOR_TORQUE_CONST = 0.1
MOTOR_IDS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,12,13,14,15] 

class LeapGzDriver(Node):
    def __init__(self):
        super().__init__('leap_gz_driver')

        self.publisher_controller = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        
        #topicos para enviar os dados de cada dedo
        self.publisher_middle = self.create_publisher(Float32MultiArray, '/middle_data', 10)
        self.publisher_ring = self.create_publisher(Float32MultiArray, '/ring_data', 10)
        self.publisher_thumb = self.create_publisher(Float32MultiArray, '/thumb_data', 10)
        self.publisher_index = self.create_publisher(Float32MultiArray, '/index_data', 10)


        self.subscription = self.create_subscription(Int32MultiArray, '/set_fingers_positions', self.set_motor_positions, 2000)
        self.subscription = self.create_subscription(Int32MultiArray, '/set_fingers_currents', self.set_currents, 2000)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.get_sim_data, 2000)

        self.time_last_vel = self.get_clock().now()
        self.joint_states: JointState = None
        self.last_control = [0.0] * len(MOTOR_IDS)
        
        #lista com os motores detetados para verificar quais os dedos que estão conectados
        #no futuro devera ser uma lista com os ids de 0 a 15
        self.present_motors = MOTOR_IDS

        # Criar um Timer para ler com frequencia de 2khz
        self.timer = self.create_timer(0.005, self.read_motors)

    def get_sim_data(self, msg: JointState):

        self.joint_states = msg

    def read_motors(self):
        """Função para ler os motores e publicar no ROS 2"""

        if self.joint_states is None:
            return

        self.time_now = self.get_clock().now()
        joint_states = copy.deepcopy(self.joint_states)
        time_diff = (self.time_now - self.time_last_vel).nanoseconds / 1e9
        positions, velocities, currents = [], [], []
        index_data = []
        middle_data = []
        ring_data = []
        thumb_data = []

        for motor_id in self.present_motors:

            id = joint_states.name.index(str(motor_id))

            cur = joint_states.effort[id] / MOTOR_TORQUE_CONST
            cur = cur if not np.isnan(cur) else 0 # Temporary fix until understanding why Gazebo gives Nan for efforts
            cur = int(np.int16(cur))

            vel = joint_states.velocity[id]
            vel = int(np.int32(vel))
            
            pos = joint_states.position[id]
            
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
            # self.publisher_position.publish(Int32MultiArray(data=positions))
            # self.publisher_velocity.publish(Int32MultiArray(data=velocities))
            # self.publisher_current.publish(Int32MultiArray(data=currents))

            self.publisher_index.publish(Float32MultiArray(data = index_data+[time_diff]))
            self.publisher_middle.publish(Float32MultiArray(data = middle_data+[time_diff]))
            self.publisher_ring.publish(Float32MultiArray(data = ring_data+[time_diff]))
            self.publisher_thumb.publish(Float32MultiArray(data = thumb_data+[time_diff]))
            #self.get_logger().info(f'Time: {time_diff}')
        self.time_last_vel = self.time_now
        
    
    def set_motor_positions(self, msg : Int32MultiArray):
        """Define as posições dos motores ao receber mensagem no tópico."""
        
        if len(msg.data) % 5 != 0:
            self.get_logger().error("Formato incorreto da mensagem recebida.")
            return

        for i in range(0, len(msg.data), 5):
            finger = msg.data[i]  # Índice do dedo
            positions = msg.data[i + 1:i + 5]  # Quatro posições para os motores desse dedo

            if len(positions) != 4:
                self.get_logger().error(f"Número incorreto de posições recebidas para o dedo {finger}.")
                continue

            for motor_id, pos in zip(range(4), positions):
                self.last_control[MOTOR_IDS[motor_id + 4 * finger]] = (2 * PI) * ((pos - 2048) / 4096) 

        
        self.publisher_controller.publish(Float64MultiArray(data=self.last_control))

    def set_currents(self, msg):
        """Define as correntes dos motores ao receber mensagem no tópico."""
        self.get_logger().error("Error: Current control not implemented in Gazebo yet!")
        
        # currents = msg.data[1:5]
        # finger = msg.data[0]

        # if len(currents) != 4:
        #     self.get_logger().info(f'Correntes recebidas:{currents}')
        #     self.get_logger().error("Número incorreto de posições recebidas.")
        #     return

        # self.group_bulk_write.clearParam()
        # for motor_id,curr in zip(range(4),currents):
        #     param_goal_curr = [DXL_LOBYTE(DXL_LOWORD(curr)), DXL_HIBYTE(DXL_LOWORD(curr))]
        #     add_success_curr = self.group_bulk_write.addParam((motor_id+4*finger), ADDR_GOAL_CURRENT, 2,param_goal_curr)

        #     if not add_success_curr:
        #         self.get_logger().error(f'Erro ao adicionar motor {motor_id} ao Bulk Write')

        # dxl_comm_result = self.group_bulk_write.txPacket()
        # if dxl_comm_result != COMM_SUCCESS:
        #     self.get_logger().error('Erro ao enviar correntes para os motores')
        # else:
        #     self.get_logger().info('Correntes enviadas com sucesso')


        # self.group_bulk_write.clearParam()
    
    def destroy_node(self):
        """Fechar a porta ao encerrar o nó"""
        self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeapGzDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
