import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from datetime import datetime
import time
import math

import copy

PI = 3.14159


MOTOR_TORQUE_CONST = 0.1
MOTOR_IDS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,12,13,14,15] 

class LeapGzDriver(Node):
    def __init__(self):
        super().__init__('leap_gz_driver')

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/hand_controller/follow_joint_trajectory",
            self.execute_callback,
        )

        self.ordered_joints = [
            "index_pip_flex_joint",
            "index_pip_abb_joint",
            "index_mcp_joint",
            "index_dip_joint",
            "middle_pip_flex_joint",
            "middle_pip_abb_joint",
            "middle_mcp_joint",
            "middle_dip_joint",
            "ring_pip_flex_joint",
            "ring_pip_abb_joint",
            "ring_mcp_joint",
            "ring_dip_joint",
            "thumb_cmc_abb_joint",
            "thumb_cmc_flex_joint",
            "thumb_mcp_joint",
            "thumb_ip_joint",
        ]

        self.traj_joint_names = None
        self.joint_order_map = {name: i for i, name in enumerate(self.ordered_joints)}

        self.publisher_controller = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        
        #topicos para enviar os dados de cada dedo
        self.publisher_middle = self.create_publisher(Float32MultiArray, '/middle_data', 10)
        self.publisher_ring = self.create_publisher(Float32MultiArray, '/ring_data', 10)
        self.publisher_thumb = self.create_publisher(Float32MultiArray, '/thumb_data', 10)
        self.publisher_index = self.create_publisher(Float32MultiArray, '/index_data', 10)

        self.subscription = self.create_subscription(Int32MultiArray, '/set_fingers_currents', self.set_currents, 2000)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.get_sim_data, 2000)

        self.time_last_vel = self.get_clock().now()
        self.joint_states: JointState = None
        self.last_control = [0.0] * len(self.ordered_joints)
        time.sleep(2.0)
        print("Initial Position sent!")
        self.publisher_controller.publish(Float64MultiArray(data=self.last_control))
        
        #lista com os motores detetados para verificar quais os dedos que estão conectados
        #no futuro devera ser uma lista com os ids de 0 a 15
        self.present_motors = self.ordered_joints

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
                if "index" in motor_id:
                    index_data.extend([pos,vel,cur])
                elif "middle" in motor_id:
                    middle_data.extend([pos,vel,cur])
                elif "ring" in motor_id:
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
        
    def execute_callback(self, goal_handle: FollowJointTrajectory):
        traj = goal_handle.request.trajectory

        hz = 1000.0

        if self.traj_joint_names is None or self.traj_joint_names != traj.joint_names:
            self.joint_order_map = {name: i for i, name in enumerate(traj.joint_names)}

        self.last_traj_time = time.time()

        positions = [np.asarray(point.positions) for point in traj.points]
        time_from_start = [
            point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            for point in traj.points
        ]

        interp_positions = []

        for i in range(0, len(positions) - 1):
            interp_positions.append(positions[i])
            N = math.floor((time_from_start[i+1] - time_from_start[i]) * hz)
            for j in range(0, N):                
                curr_pos = positions[i] + (positions[i+1] - positions[i]) * (j / N)
                interp_positions.append(curr_pos)

        interp_positions.append(positions[-1])

        for positions in interp_positions:
            ordered_positions = [
                positions[self.joint_order_map[joint_name]]
                for joint_name in self.ordered_joints
            ]

            self.send_to_robot(ordered_positions, hz)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def send_to_robot(self, positions, freq):
        for motor_id, pos in enumerate(positions):
            self.last_control[motor_id] = pos

        self.publisher_controller.publish(Float64MultiArray(data=self.last_control))
        time.sleep(1 / freq)


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
    print("Launching Leap Gazebo Driver")
    node = LeapGzDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
