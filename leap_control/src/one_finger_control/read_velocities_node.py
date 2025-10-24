import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time
from datetime import datetime
import csv
import matplotlib.pyplot as plt
import collections
import numpy as np

class ReadVelocities(Node):
    def __init__(self):
        super().__init__('read_velocities')

        # Criar timestamp para o nome do ficheiro
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"/home/beatrix/ros2_ws/src/leap_control/leap_hand_control/leap_hand_control/data/velocities/finger_velocities_{timestamp}.csv"

        # Criar e escrever o cabeçalho do CSV
        with open(self.csv_filename, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "Motor1", "Motor2", "Motor3", "Motor4"])  # Cabeçalho


        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/dynamixel_finger_velocities',
            self.listener_callback,
            10)
        
        self.start_time = self.get_clock().now()

        self.min_data_points = 100
        self.max_data_points = 300
        self.data_points = self.min_data_points

        self.timestamps = collections.deque(maxlen=self.data_points)
        self.motor_data = {f"Motor{i+1}": collections.deque(maxlen=self.data_points) for i in range(4)}

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.lines = {motor: self.ax.plot([], [], label=motor)[0] for motor in self.motor_data.keys()}
        self.ax.set_ylim(-6, 6)
        self.ax.set_xlabel("Tempo (s)")
        self.ax.set_ylabel("Velocidade do Motor (rad/s)")
        self.ax.legend()
    
    def listener_callback(self, msg):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        data_row = [elapsed_time] + list(msg.data)  # Converte msg.data para lista
        with open(self.csv_filename, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(data_row)

        self.timestamps.append(elapsed_time)

        for i, motor in enumerate(self.motor_data.keys()):
            self.motor_data[motor].append(msg.data[i])

        self.adjust_buffer_size(elapsed_time)
        #self.get_logger().info(f'Velocidades : {msg.data}')
    
    def adjust_buffer_size(self,elapsed_time):
        """ Ajusta dinamicamente o número de pontos no buffer baseado na frequência dos dados """
        if len(self.timestamps) > 2:
            delta_time = self.timestamps[-1] - self.timestamps[0]  # Janela de tempo atual
            if delta_time > 5.0:  # Se a janela do gráfico for maior que 5s, reduzir buffer
                self.data_points = max(self.min_data_points, self.data_points - 5)
            elif delta_time < 2.0:  # Se for menor que 2s, aumentar buffer
                self.data_points = min(self.max_data_points, self.data_points + 5)

            # Atualizar os buffers
            self.timestamps = collections.deque(self.timestamps, maxlen=self.data_points)
            for motor in self.motor_data.keys():
                self.motor_data[motor] = collections.deque(self.motor_data[motor], maxlen=self.data_points)

    def update_plot(self):
        if len(self.timestamps) > 0:
            min_time = max(0, self.timestamps[0])
            max_time = self.timestamps[-1]
            self.ax.set_xlim(min_time, max_time)

            for motor, line in self.lines.items():
                vels = [(vel*0.229*2*np.pi) / 60 for vel in self.motor_data[motor]]
                line.set_data(self.timestamps, vels)

            self.ax.relim()
            self.ax.autoscale_view()
            self.fig.canvas.draw()
            plt.pause(0.001)  

        
        


def main(args=None):
    rclpy.init(args=args)
    node = ReadVelocities()
    try:
        #rclpy.spin(node)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.update_plot()
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando nó e gráfico.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()
