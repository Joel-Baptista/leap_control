import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
import matplotlib.pyplot as plt
import collections
import time
import csv
from datetime import datetime
import numpy as np

GOAL_CURRENT_VALUE = 300
GRASPING_CURRENT_VALUE = 100


class Thumb(Node):
    def __init__(self):
        super().__init__('thumb_manager')

        self.publisher = self.create_publisher(Int32MultiArray, '/set_fingers_currents', 10)

        self.pos = []
        self.vels = []
        self.currs = []
        self.is_grasping = 0
        self.last_vel = [0, 0, 0, 0]
        self.time_last_vel = 0.0
        self.last_vels = np.zeros(4)
        self.relative_vels = np.ones(4,dtype=int)

        #self.current = GOAL_CURRENT_VALUE
        self.currents = np.ones(4)*GOAL_CURRENT_VALUE

        #timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        #self.time_last_vel = self.get_clock().now()

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/thumb_data',
            self.listener_callback,
            10) 

        #self.start_time = self.get_clock().now()


    def listener_callback(self, msg):
        self.pos = [msg.data[pos] for pos in range(0,len(msg.data)-1,3)]
        self.vels = [msg.data[vel] for vel in range(1,len(msg.data)-1,3)]
        self.currs = [msg.data[curr] for curr in range(2,len(msg.data)-1,3)]
        time_diff = msg.data[len(msg.data)-1]

        #self.get_logger().info(f'Acc:{np.array(self.currs)}')
        
        #quando existe redução da velocidade e aumento de corrente, diminui a goal current para não esmagar objetos
        if (any(abs((np.array(self.vels) - self.last_vels)) / time_diff) < 0.1) and (any(abs(np.array(self.currs)) > 0.8*GOAL_CURRENT_VALUE)) and (self.is_grasping == 0):
            self.get_logger().info('Grasping!!!!')
            #self.get_logger().info(f'Correntes: {np.ones(4, dtype=int)*GRASPING_CURRENT_VALUE}')
            self.publish_currents(np.ones(4, dtype=int)*GRASPING_CURRENT_VALUE)
            self.is_grasping = 1
        elif (self.is_grasping == 1) and all(abs(np.array(self.currs)) < 0.5*GOAL_CURRENT_VALUE) and  (all(abs((np.array(self.vels) - self.last_vels)) / time_diff) > 0.1):
            #se a corrente for pequena, o dedo não está a apanhar nada e apenas se movimenta
            self.publish_currents(np.ones(4, dtype=int)*500)
            self.is_grasping = 0
        
        
        self.time_last_vel = msg.data[len(msg.data)-1]
        self.last_vels = self.vels
        #self.get_logger().info(f'Time:{msg.data[len(msg.data)-1]}')


    def publish_currents(self, currents):
        msg = Int32MultiArray(data=[3] + list(currents))
        self.get_logger().info(f'Correntes: {currents}')
        self.publisher.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = Thumb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
