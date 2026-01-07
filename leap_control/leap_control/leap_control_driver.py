import rclpy
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory

from dynamixel_sdk import *  # Biblioteca Dynamixel SDK
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

import numpy as np
import math
import yaml
import os

from utils.leap_base import LeapBase


class LeapDriver(LeapBase):

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

        self.current_limit = self.get_rosparam("current_limit", 1800)
        self.pwd_limit = self.get_rosparam("pwd_limit", 885)
        self.goal_current = self.get_rosparam("goal_current", 500)
        self.profile_velocity = self.get_rosparam("profile_velocity", 100)
        operating_mode = self.get_rosparam("operating_mode", "current-based position")

        self.pos_res = self.get_rosparam("pos_res", 0.0015343541)
        self.vel_res = self.get_rosparam("pos_res", 0.2288857142)
        self.effort_res = self.get_rosparam("pos_res", 0.000517)
        self.neutral_pos = self.get_rosparam("pos_res", 2048)
        
        pkg_share = get_package_share_directory('leap_control')
        dynamixel_yaml_file = os.path.join(pkg_share, 'config', 'dynamixel_address.yaml')

        with open(dynamixel_yaml_file, 'r') as file:
            addresses = yaml.safe_load(file)

        self.addresses = dict(addresses)

        self.addr_operating_mode = self.addresses.get("addr_operating_mode", 11)
        self.addr_pwm_limit = self.addresses.get("addr_pwm_limit", 36)
        self.addr_current_limit = self.addresses.get("addr_current_limit", 38)
        self.addr_torque_enable = self.addresses.get("addr_torque_enable", 64)
        self.addr_led = self.addresses.get("addr_led", 65)
        self.addr_hardware_error_status = self.addresses.get("addr_hardware_error_status", 70)
        self.addr_vel_i_gain = self.addresses.get("addr_vel_i_gain", 76)
        self.addr_vel_p_gain = self.addresses.get("addr_vel_p_gain", 78)
        self.addr_pos_d_gain = self.addresses.get("addr_pos_d_gain", 80)
        self.addr_pos_i_gain = self.addresses.get("addr_pos_i_gain", 82)
        self.addr_pos_p_gain = self.addresses.get("addr_pos_p_gain", 84)
        self.addr_goal_pwm = self.addresses.get("addr_goal_pwm", 100)
        self.addr_goal_current = self.addresses.get("addr_goal_current", 102)
        self.addr_goal_vel = self.addresses.get("addr_goal_vel", 104)
        self.addr_profile_accel = self.addresses.get("addr_profile_accel", 108)
        self.addr_profile_velocity = self.addresses.get("addr_profile_velocity", 112)
        self.addr_goal_position = self.addresses.get("addr_goal_position", 116)
        self.addr_moving = self.addresses.get("addr_moving", 122)
        self.addr_moving_status = self.addresses.get("addr_moving_status", 123)
        self.addr_present_pwm = self.addresses.get("addr_present_pwm", 124)
        self.addr_present_current = self.addresses.get("addr_present_current", 126)
        self.addr_present_velocity = self.addresses.get("addr_present_velocity", 128)
        self.addr_present_position = self.addresses.get("addr_present_position", 132)
        self.addr_indirect_start = self.addresses.get("addr_indirect_start", 168)

        self.pos_length = self.addresses.get("pos_length", 4)
        self.vel_length = self.addresses.get("vel_length", 4)
        self.curr_length = self.addresses.get("curr_length", 2)
        self.total_length = self.addresses.get("total_length", 10)

        self.motor_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

        default_operation_modes = {
            "current": 0,
            "velocity": 1,
            "position": 3,
            "extended postion": 4,
            "current-based position": 5,
            "pwm": 16,
        }

        self.operating_mode = default_operation_modes.get(operating_mode.lower(), None)
        if self.operating_mode:
            self.operating_mode = 5
            self.get_logger().warning(
                f"Default operating mode {operating_mode} is not configured. Falling back on 'current-based position'"
            )

        self.joint_state_pub = self.create_publisher(
            JointState, "/joint_states", 10
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
            _, dxl_comm_result, _ = self.packet_handler.ping(
                self.port_handler, motor_id
            )
            if dxl_comm_result == COMM_SUCCESS:
                # self.get_logger().info(f'Motor {motor_id} detectado - Modelo: {dxl_model_number}')
                self.present_motors.append(motor_id)

        # Adicionar motores ao Bulk Read
        for motor_id in self.present_motors:
            self.motor_disable(motor_id)

            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                motor_id,
                self.addr_operating_mode,
                self.operating_mode,
            )
            self.packet_handler.write4ByteTxRx(
                self.port_handler,
                motor_id,
                self.addr_profile_velocity,
                self.profile_velocity,
            )
            self.packet_handler.write2ByteTxRx(
                self.port_handler,
                motor_id,
                self.addr_goal_current,
                self.goal_current,
            )
            self.motor_enable(motor_id)

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

    def motor_enable(self, motor_id):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.addr_torque_enable, 1
        )

    def motor_disable(self, motor_id):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.addr_torque_enable, 0
        )

    def read_motors(self):
        """Função para ler os motores e publicar no ROS 2"""
        dxl_comm_result = self.group_sync_read.txRxPacket()

        self.time_now = self.get_clock().now()

        msg = JointState()
        msg.header.stamp.sec = self.time_now.seconds_nanoseconds()[0]
        msg.header.stamp.nanosec = self.time_now.seconds_nanoseconds()[1]
        msg.name = self.ordered_joints

        positions = []
        velocities = []
        efforts = []

        for motor_id in self.present_motors:
            cur = self.group_sync_read.getData(
                motor_id, self.addr_present_current, self.curr_length
            )
            cur = int(np.int16(cur))

            efforts.append(0.517 * cur / 1000)
            vel = self.group_sync_read.getData(
                motor_id, self.addr_present_velocity, self.vel_length
            )
            vel = int(np.int32(vel)) 
            velocities.append(vel * 0.2288857142 * 0.104719755)
            pos = self.group_sync_read.getData(
                motor_id, self.addr_present_position, self.pos_length
            )

            positions.append((pos - 2048) * 0.0015343541)
            # positions.append(pos)

        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts

        self.joint_state_pub.publish(msg)
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
            N = math.floor((time_from_start[i + 1] - time_from_start[i]) * hz)
            for j in range(0, N):
                curr_pos = positions[i] + (positions[i + 1] - positions[i]) * (j / N)
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
