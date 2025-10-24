import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from urdfpy import URDF
import numpy as np
import os
import matplotlib.pyplot as plt


DIST_TRESHOLD = 0.02 # 2cm

class FingerCollisionDetector(Node):
    def __init__(self):
        super().__init__('finger_collision_detector')

        # Caminho do URDF
        urdf_path = os.path.abspath("/home/beatrix/ros2_ws/src/leap_control/leap_hand_control/leap_hand_control/data/leap_right/robot.urdf")
        self.robot = URDF.load(urdf_path)
        self.joint_angles = np.ones(16)*np.pi

        # Inscrevendo nos tópicos
        self.subscription_middle = self.create_subscription(
            Float32MultiArray, '/middle_data', self.middle_callback, 10)
        self.subscription_thumb = self.create_subscription(
            Float32MultiArray, '/thumb_data', self.thumb_callback, 10)

        # Inicializa as posições dos dedos
        self.middle_position = None
        self.thumb_position = None

        self.get_logger().info("Finger Collision Detector Node Started")

    def middle_callback(self, msg):
        
        self.middle_position = np.array([(msg.data[pos] / 4095) * 2*np.pi for pos in range(0, len(msg.data) - 1, 3)]) 

        self.joint_angles[4:8] = self.middle_position
        self.check_collision()

    def thumb_callback(self, msg):
        #self.get_logger().info(f'{np.array([msg.data[pos] for pos in range(0, len(msg.data) - 1, 3)])}')
        self.thumb_position = np.array([(msg.data[pos] / 4095) * 2*np.pi for pos in range(0, len(msg.data) - 1, 3)]) 
        self.joint_angles[12:16] = self.thumb_position
        self.check_collision()

    def check_collision(self):
        if self.middle_position is not None and self.thumb_position is not None:
            fk_result = self.robot.link_fk(self.joint_angles - np.pi)
            middle_fingertip = fk_result[self.robot.links[8]][:3, 3]
            thumb_fingertip = fk_result[self.robot.links[16]][:3, 3]
            distance = np.linalg.norm(middle_fingertip- thumb_fingertip)
            self.get_logger().info(f'Distance: {distance}')
            # if distance < DIST_TRESHOLD:
            #     self.get_logger().warn(f"COLISÃO DETECTADA! Distância: {distance:.4f}m")
            # else:
            #     self.get_logger().info(f"Distância segura: {distance:.4f}m")
            #self.get_logger().info(self.joint_angles - np.pi)
            #self.visualize_robot()
    
    def visualize_robot(self):
        """Exibe a mão robótica com os ângulos de junta atuais."""
        joint_cfg = {joint.name: angle for joint, angle in zip(self.robot.joints, self.joint_angles)}

        plt.ion()  # Ativa o modo interativo para atualizar a exibição
        plt.clf()  # Limpa a figura anterior
        self.robot.show(cfg=joint_cfg)  # Renderiza a nova posição do robô
        plt.pause(0.1)  # Dá um pequeno intervalo para atualização gráfica

            


def main(args=None):
    rclpy.init(args=args)
    node = FingerCollisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
