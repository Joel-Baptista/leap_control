import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
import csv
import os
from datetime import datetime


class SaveData(Node):
    def __init__(self):
        super().__init__('save_data')
        self.get_logger().info("Nó SaveData iniciado com sucesso.")

        self.state = 0
        self.my_class = 1
        self.pos = []
        self.curr=[]
        self.sensors = []

        # Criar diretório se não existir
        directory = "/home/beatrix/ros2_ws/src/leap_control/leap_hand_control/leap_hand_control/data/dataset/"
        os.makedirs(directory, exist_ok=True)

        # Criar timestamp para o nome do ficheiro
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(directory, f"data_test.csv")

        # Verificar se o ficheiro já existe e tem conteúdo
        file_exists_and_has_content = os.path.isfile(self.csv_filename) and os.path.getsize(self.csv_filename) > 0


        # Abrir ficheiro CSV uma única vez
        self.file = open(self.csv_filename, mode="a", newline="")
        self.writer = csv.writer(self.file)

        # Escrever cabeçalho
        if not file_exists_and_has_content:
            header = []
            header += [f'Pos{i+1}' for i in range(16)]
            header += [f'Curr{i+1}' for i in range(16)]
            header += [f'Sensor{i+1}' for i in range(5)]
            header += ['Class']
            self.writer.writerow(header)

        #Criar subscrições
        self.state_subscription = self.create_subscription(
            Int32,
            '/state',
            self.state_listener_callback,
            10) 
        
        self.class_subscription = self.create_subscription(
            Int32,
            '/set_class',
            self.class_listener_callback,
            10) 
        

        
        self.pos_subscription = self.create_subscription(
            Int32MultiArray,
            '/dynamixel_finger_positions',
            self.pos_listener_callback,
            10)
        
        self.curr_subscription = self.create_subscription(
            Int32MultiArray,
            '/dynamixel_finger_currents',
            self.curr_listener_callback,
            10)
        
        self.sensor_subscription = self.create_subscription(
            Float32MultiArray,
            '/fsr_values',
            self.fsr_listener_callback,
            10) 
        

        timer_period = 0.1  # 100ms
        self.timer = self.create_timer(timer_period, self.save_data_if_state_is_one)


    def pos_listener_callback(self,msg):
        self.pos = list(msg.data)
        #self.get_logger().info(f'Posiçoes: {self.pos}')
    
    def curr_listener_callback(self,msg):
        self.curr = list(msg.data)
        #self.get_logger().info(f'Correntes: {self.curr}')
    
    def fsr_listener_callback(self,msg):
        self.sensors = list(msg.data)
        self.sensors = [round(s, 2) for s in self.sensors]

    def class_listener_callback(self,msg):
        if self.my_class != msg.data:
            self.my_class = msg.data
            self.get_logger().info(f'Classe: {self.my_class}')

        



    def state_listener_callback(self, msg):
        self.state = msg.data
        self.get_logger().info(f'Estado: {self.state}')
    
    def save_data_if_state_is_one(self):
        if self.state == 1:
            # Verificar se todos os dados de uma das listas são diferentes de zero
            all_pos_nonzero = all(p != 0 for p in self.pos)
            all_curr_nonzero = all(c != 0 for c in self.curr)
            all_sensors_nonzero = all(s != 0 for s in self.sensors)

            if all_pos_nonzero or all_curr_nonzero or all_sensors_nonzero:
                row_data = self.pos + self.curr + self.sensors + [self.my_class]
                self.writer.writerow(row_data)
                self.get_logger().info(f'Dados guardados: {row_data}')
            


def main(args=None):
    rclpy.init(args=args)
    node = SaveData()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando nó.")
    finally:
        node.timer.cancel()
        node.file.close()  # Fecha o ficheiro corretamente
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
