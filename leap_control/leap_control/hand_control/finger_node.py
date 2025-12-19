import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import matplotlib.pyplot as plt
import collections
import time
import csv
from datetime import datetime
import numpy as np


class Finger(Node):
    def __init__(self):
        super().__init__('finger_manager')

        self.finger_name = self.get_rosparam('finger_name', 'unkown')
        self.goal_current_value = self.get_rosparam('goal_current_value', 300)
        self.grasping_current_value = self.get_rosparam('grasping_current_value', 100)

        self.publisher = self.create_publisher(Int32MultiArray, '/set_fingers_currents', 10)
        self.state_publisher = self.create_publisher(Int32, '/state', 10)

        self.pos = []
        self.vels = []
        self.currs = []
        self.is_grasping = 0
        self.last_vel = [0, 0, 0, 0]
        self.time_last_vel = 0.0
        self.last_vels = np.zeros(4)
        self.relative_vels = np.ones(4,dtype=int)

        self.currents = np.ones(4)*self.goal_current_value

        self.subscription = self.create_subscription(
            Float32MultiArray,
            f'/{self.finger_name}_data',
            self.listener_callback,
            10) 

    def get_rosparam(self, parameter_name, default_value):
        self.declare_parameter(parameter_name, default_value)
        parameter = self.get_parameter(parameter_name).value
        self.get_logger().info(f"Starting finger manager for: {parameter}")

        return parameter

    def listener_callback(self, msg):
        self.pos = [msg.data[pos] for pos in range(0,len(msg.data)-1,3)]
        self.vels = [msg.data[vel] for vel in range(1,len(msg.data)-1,3)]
        self.currs = [msg.data[curr] for curr in range(2,len(msg.data)-1,3)]
        time_diff = msg.data[len(msg.data)-1]

        #quando existe redução da velocidade e aumento de corrente, diminui a goal current para não esmagar objetos
        if (any(abs((np.array(self.vels) - self.last_vels)) / time_diff) < 0.1) and (any(abs(np.array(self.currs)) > 0.8*self.goal_current_value)) and (self.is_grasping == 0):
            self.get_logger().info('Grasping!!!!')
            self.publish_currents(np.ones(4, dtype=int)*self.grasping_current_value)
            self.is_grasping = 1
            self.state_publisher.publish(Int32(data=1))
        elif (self.is_grasping == 1) and all(abs(np.array(self.currs)) < 0.5*self.goal_current_value) and  (all(abs((np.array(self.vels) - self.last_vels)) / time_diff) > 0.1):
            #se a corrente for pequena, o dedo não está a apanhar nada e apenas se movimenta
            self.publish_currents(np.ones(4, dtype=int)*500)
            self.is_grasping = 0
            self.state_publisher.publish(Int32(data=0))
        
        
        self.time_last_vel = msg.data[len(msg.data)-1]
        self.last_vels = self.vels
        #self.get_logger().info(f'Time:{msg.data[len(msg.data)-1]}')


    def publish_currents(self, currents):
        msg = Int32MultiArray(data=[1] + list(currents))
        self.get_logger().info(f'Correntes: {currents}')
        self.publisher.publish(msg)
        #self.get_logger().info(f'Publicando posições: {currents}')
        


def main(args=None):
    rclpy.init(args=args)
    node = Finger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
