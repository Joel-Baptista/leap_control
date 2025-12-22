import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory

from dynamixel_sdk import *  # Biblioteca Dynamixel SDK
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

import numpy as np
import math


class LeapDriver(Node):

    def __init__(self):
        super().__init__("hand_controller_driver")

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

        self.port_name = self.get_rosparam("port_name", "/dev/ttyUSB0")
        self.baudrate = self.get_rosparam("baudrate", 4000000)
        self.protocol_version = self.get_rosparam("protocol_version", 2.0)
        self.torque_enable = self.get_rosparam("torque_enable", 64)
        self.addr_goal_position = self.get_rosparam("addr_goal_position", 116)
        self.addr_goal_current = self.get_rosparam("addr_goal_current", 102)
        self.addr_operating_mode = self.get_rosparam("addr_operating_mode", 11)
        self.current_based_position_mode = self.get_rosparam(
            "current_based_position_mode", 5
        )
        self.addr_current_limit = self.get_rosparam("addr_current_limit", 38)
        self.current_limit_value = self.get_rosparam("current_limit_value", 1800)
        self.goal_current_value = self.get_rosparam("goal_current_value", 500)
        self.addr_present_current = self.get_rosparam("addr_present_current", 126)
        self.addr_present_position = self.get_rosparam("addr_present_position", 132)
        self.addr_present_velocity = self.get_rosparam("addr_present_velocity", 128)
        self.total_length = self.get_rosparam("total_length", 10)
        self.vel_length = self.get_rosparam("vel_length", 4)
        self.pos_length = self.get_rosparam("pos_length", 4)
        self.curr_length = self.get_rosparam("curr_length", 2)
        self.addr_indirect_start = self.get_rosparam("addr_indirect_start", 168)
        self.addr_profile_velocity = self.get_rosparam("addr_profile_velocity", 112)
        self.profile_velocity_value = self.get_rosparam("profile_velocity_value", 100)

        self.motor_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

        # topicos para enviar os dados dos motores para posterior analise
        self.publisher_position = self.create_publisher(
            Int32MultiArray, "/dynamixel_finger_positions", 10
        )
        self.publisher_velocity = self.create_publisher(
            Int32MultiArray, "/dynamixel_finger_velocities", 10
        )
        self.publisher_current = self.create_publisher(
            Int32MultiArray, "/dynamixel_finger_currents", 10
        )

        # topicos para enviar os dados de cada dedo
        self.publisher_middle = self.create_publisher(
            Float32MultiArray, "/middle_data", 10
        )
        self.publisher_ring = self.create_publisher(Float32MultiArray, "/ring_data", 10)
        self.publisher_thumb = self.create_publisher(
            Float32MultiArray, "/thumb_data", 10
        )
        self.publisher_index = self.create_publisher(
            Float32MultiArray, "/index_data", 10
        )

        # Inicializar comunicação com Dynamixel
        self.port_handler = PortHandler(self.port_name)
        self.port_handler.setPacketTimeout(0.5)
        self.packet_handler = PacketHandler(self.protocol_version)
        self.group_bulk_read = GroupBulkRead(self.port_handler, self.packet_handler)
        self.group_bulk_write = GroupBulkWrite(self.port_handler, self.packet_handler)

        self.time_last_vel = self.get_clock().now()
        self.last_traj_time = time.time()

        if self.port_handler.openPort() and self.port_handler.setBaudRate(
            self.baudrate
        ):
            self.get_logger().info("Conexão com Dynamixel estabelecida.")
        else:
            self.get_logger().error("Falha ao conectar com Dynamixel.")
            return

        # # Abrir porta
        if self.port_handler.openPort() and self.port_handler.setBaudRate(
            self.baudrate
        ):
            self.get_logger().info("Conexão com Dynamixel estabelecida.")
        else:
            self.get_logger().error("Falha ao conectar com Dynamixel.")
            return

        # lista com os motores detetados para verificar quais os dedos que estão conectados
        # no futuro devera ser uma lista com os ids de 0 a 15
        self.present_motors = []

        # deteta quais os motores que estao conectados
        for motor_id in self.motor_ids:
            dxl_model_number, dxl_comm_result, dxl_error = self.packet_handler.ping(
                self.port_handler, motor_id
            )
            if dxl_comm_result == COMM_SUCCESS:
                # self.get_logger().info(f'Motor {motor_id} detectado - Modelo: {dxl_model_number}')
                self.present_motors.append(motor_id)

        # Adicionar motores ao Bulk Read
        for motor_id in self.present_motors:
            # desativar o torque para alterar o modo de operação e realizar as configurações iniciais
            self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, self.torque_enable, 0
            )
            # self.group_bulk_read.addParam(motor_id, ADDR_PRESENT_CURRENT, TOTAL_LENGTH)
            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                motor_id,
                self.addr_operating_mode,
                self.current_based_position_mode,
            )
            self.packet_handler.write4ByteTxRx(
                self.port_handler,
                motor_id,
                self.addr_profile_velocity,
                self.profile_velocity_value,
            )
            self.packet_handler.write2ByteTxRx(
                self.port_handler,
                motor_id,
                self.addr_goal_current,
                self.goal_current_value,
            )
            self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, self.torque_enable, 1
            )

        # SyncRead para posição, velocidade e corrente
        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            self.addr_present_current,
            self.total_length,
        )

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

        # comm_result = self.group_bulk_read.txRxPacket()
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
            # cur = self.group_bulk_read.getData(motor_id, ADDR_PRESENT_CURRENT, CURR_LENGTH)
            cur = self.group_sync_read.getData(
                motor_id, self.addr_present_current, self.curr_length
            )
            cur = int(np.int16(cur))
            # vel = self.group_bulk_read.getData(motor_id, ADDR_PRESENT_VELOCITY, VEL_LENGTH)
            vel = self.group_sync_read.getData(
                motor_id, self.addr_present_velocity, self.vel_length
            )
            vel = int(np.int32(vel))
            # pos = self.group_bulk_read.getData(motor_id,ADDR_PRESENT_POSITION,POS_LENGTH)
            pos = self.group_sync_read.getData(
                motor_id, self.addr_present_position, self.pos_length
            )
            # print(f"Motor {motor_id} | Pos: {pos} | Vel: {vel} | Corrente: {cur}")
            if pos is None or vel is None or cur is None:
                self.get_logger().warn(f"Falha na leitura do motor {motor_id}")
                continue
            else:
                if motor_id <= 3:
                    index_data.extend([pos, vel, cur])
                elif motor_id > 3 and motor_id < 8:
                    middle_data.extend([pos, vel, cur])
                elif motor_id >= 8 and motor_id < 12:
                    ring_data.extend([pos, vel, cur])
                else:
                    thumb_data.extend([pos, vel, cur])

            positions.append(pos)
            velocities.append(vel)
            currents.append(cur)

        if positions and velocities and currents:
            self.publisher_position.publish(Int32MultiArray(data=positions))
            self.publisher_velocity.publish(Int32MultiArray(data=velocities))
            self.publisher_current.publish(Int32MultiArray(data=currents))

            self.publisher_index.publish(
                Float32MultiArray(data=index_data + [time_diff])
            )
            self.publisher_middle.publish(
                Float32MultiArray(data=middle_data + [time_diff])
            )
            self.publisher_ring.publish(Float32MultiArray(data=ring_data + [time_diff]))
            self.publisher_thumb.publish(
                Float32MultiArray(data=thumb_data + [time_diff])
            )
            # self.get_logger().info(f'Time: {time_diff}')
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

        print(len(interp_positions))
        for positions in interp_positions:
            ordered_positions = [
                positions[self.joint_order_map[joint_name]]
                for joint_name in self.ordered_joints
            ]

            self.send_to_robot(ordered_positions, hz)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def send_to_robot(self, positions, freq):
        st = time.time()
        self.group_bulk_write.clearParam()

        for motor_id, pos in enumerate(positions):
            dxl = self.radians_to_dinamixel(pos)
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(dxl)),
                DXL_HIBYTE(DXL_LOWORD(dxl)),
                DXL_LOBYTE(DXL_HIWORD(dxl)),
                DXL_HIBYTE(DXL_HIWORD(dxl)),
            ]

            add_success_pos = self.group_bulk_write.addParam(
                (motor_id), self.addr_goal_position, 4, param_goal_position
            )

            if not add_success_pos:
                self.get_logger().error(
                    f"Erro ao adicionar motor {motor_id} ao Bulk Write"
                )
        dxl_comm_result = self.group_bulk_write.txPacket()
        time.sleep(1 / freq)

        # if dxl_comm_result != COMM_SUCCESS:
        #     self.get_logger().error("Erro ao enviar posições para os motores")
        # else:
        #     self.get_logger().info("Posições enviadas com sucesso")


    @staticmethod
    def radians_to_dinamixel(rad):
        # return int(((rad * 4096) / (2 * np.pi)))
        return int(2048 + ((rad * 4096) / (2 * np.pi)))


def main(args=None):
    rclpy.init(args=args)
    node = LeapDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
