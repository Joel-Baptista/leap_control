import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import re
import numpy as np
import time

class SetPositions(Node):
    def __init__(self):
        super().__init__('set_positions')
        self.publisher = self.create_publisher(Int32MultiArray, '/set_fingers_positions', 10)
        self.experience_publisher = self.create_publisher(Int32, '/set_class', 10)

    def publish_positions(self, positions):
        msg = Int32MultiArray(data=positions)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publicando posições: {positions}')

    def publish_class(self,new_class):
        msg = Int32(data=new_class)
        self.experience_publisher.publish(msg)
        self.get_logger().info(f'Publicando classe: {new_class}')

    def publish_ordered_positions(self, data):
        # Ordenar os dados com base no offset
        data.sort(key=lambda x: x[-1])

        msg = data[0][:-1] 
        previous_offset = data[0][-1] / 1000  # primeiro offset
        for i in range(1, len(data)):  #
        
            offset = data[i][-1] / 1000
            
            if offset != previous_offset:  
                self.publish_positions(msg)  # Publica as posições acumuladas
                self.get_logger().info(f"Aguardando {offset - previous_offset} segundos antes de enviar a próxima posição...")
                time.sleep(offset - previous_offset) 
                msg = []  

            # Acumula as posições para o mesmo offset
            msg.extend(data[i][:-1])  
            previous_offset = offset  


        self.publish_positions(msg)


    def radians_to_dynamixel(self,angle_rad):
        return int((angle_rad * 4095) / (2 * np.pi))

def main(args=None):
    rclpy.init(args=args)
    node = SetPositions()
    finger_names = ["index", "middle", "ring", "thumb"]
    positions = []
    
    try:
        while rclpy.ok():
            print("AAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHH")
            input_str = input("Introduzir o dedo e as posições (ex: middle 2048 2048 2048 2048) ou comando (ex: thumb close): ").strip()
            input_parts = input_str.split()

            if not input_parts:
                continue

            finger_name = input_parts[0].lower()

            data_to_send = []
            offsets = [0,0,0,0]

            if len(input_parts) >= 2 and input_parts[0].lower() == "hand":
                command = input_parts[1].lower()
                if command == "close":

                    if len(input_parts) > 2:
                        offsets = list(map(float, input_parts[2:]))
                        offsets = [int(offset * 1000) for offset in offsets] #converter para milissegundos

                    # Enviar posições de fecho
                    data_to_send = [
                        [finger_names.index("middle"), 3486, 2048, 2369, 2929, offsets[1]],
                        [finger_names.index("thumb"), 3016, 591, 2980, 2483, offsets[3]],
                        [finger_names.index("index"), 3486, 2048, 2369, 2929, offsets[0]],
                        [finger_names.index("ring"), 3486, 2048, 2369, 2929, offsets[2]],
                        #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
                    ]

                    node.publish_ordered_positions(data_to_send)
                    
                elif command == "open":

                    if len(input_parts) > 2:
                        offsets = list(map(float, input_parts[2:]))
                        offsets = [int(offset * 1000) for offset in offsets] #converter para milissegundos

                    # Enviar posições de abertura
                    data_to_send = [
                        [finger_names.index("middle"), 2048, 2048, 2048, 2048,offsets[1]],
                        [finger_names.index("thumb"), 2048, 1028, 2048, 2048,offsets[3]],
                        [finger_names.index("index"), 2048, 2048, 2048, 2048,offsets[0]],
                        [finger_names.index("ring"), 2048, 2048, 2048, 2048,offsets[2]],
                        #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
                    ]

                    node.publish_ordered_positions(data_to_send)
                    
                else:
                    node.get_logger().error("Comando inválido para 'hand'. Use 'close' ou 'open'.")
                    continue

            elif len(input_parts) == 1 and input_parts[0].lower() == "wave":
                offsets = [500,700,900,0]
                data_to_send = [
                        [finger_names.index("middle"), 3486, 2048, 2369, 2929, offsets[1]],
                        [finger_names.index("thumb"), 3016, 591, 2980, 2483, offsets[3]],
                        [finger_names.index("index"), 3486, 2048, 2369, 2929, offsets[0]],
                        [finger_names.index("ring"), 3486, 2048, 2369, 2929, offsets[2]],
                        #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
                    ]
                
                node.publish_ordered_positions(data_to_send)

                min_offset = min(offsets)
                max_offset = max(offsets)

                time.sleep(max_offset/1000 + 0.5)

                offsets = [max_offset - (o - min_offset) for o in offsets]


                data_to_send = [
                        [finger_names.index("middle"), 2048, 2048, 2048, 2048,offsets[1]],
                        [finger_names.index("thumb"), 2048, 1028, 2048, 2048,offsets[3]],
                        [finger_names.index("index"), 2048, 2048, 2048, 2048,offsets[0]],
                        [finger_names.index("ring"), 2048, 2048, 2048, 2048,offsets[2]],
                        #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
                    ]
                node.publish_ordered_positions(data_to_send)

            elif len(input_parts) == 1 and input_parts[0].lower() == "0":
                node.publish_class(0)
                offsets = [0,0,0,0]
                data_to_send = [
                        [finger_names.index("middle"), 3486, 2048, 2369, 2929, offsets[1]],
                        [finger_names.index("thumb"), 3016, 591, 2980, 2483, offsets[3]],
                        [finger_names.index("index"), 3486, 2048, 2369, 2929, offsets[0]],
                        [finger_names.index("ring"), 3486, 2048, 2369, 2929, offsets[2]],
                        #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
                    ]
                
                node.publish_ordered_positions(data_to_send)


                time.sleep(2)


                data_to_send = [
                        [finger_names.index("middle"), 2048, 2048, 2048, 2048,offsets[1]],
                        [finger_names.index("thumb"), 2048, 1024, 2048, 2048,offsets[3]],
                        [finger_names.index("index"), 2048, 2048, 2048, 2048,offsets[0]],
                        [finger_names.index("ring"), 2048, 2048, 2048, 2048,offsets[2]],
                        #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
                    ]
                
                node.publish_ordered_positions(data_to_send)


                data_to_send = [
                        [finger_names.index("middle"), 2048, 2048, 2048, 2048,offsets[1]],
                        [finger_names.index("thumb"), 2048, 1024, 2048, 2048,offsets[3]],
                        [finger_names.index("index"), 2048, 2048, 2048, 2048,offsets[0]],
                        [finger_names.index("ring"), 2048, 2048, 2048, 2048,offsets[2]],
                        #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
                    ]
                node.publish_ordered_positions(data_to_send)

            
            elif len(input_parts) == 1 and input_parts[0].lower() == "1":
                node.publish_class(1)
                offsets = [500,500,500,0]
                data_to_send = [
                        [finger_names.index("middle"), 3486, 2048, 2369, 2929, offsets[1]],
                        [finger_names.index("thumb"), 3016, 591, 2980, 2483, offsets[3]],
                        [finger_names.index("index"), 3486, 2048, 2369, 2929, offsets[0]],
                        [finger_names.index("ring"), 3486, 2048, 2369, 2929, offsets[2]],
                        #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
                    ]
                
                node.publish_ordered_positions(data_to_send)


                time.sleep(2)


                offsets = [0,0,0,500]

                data_to_send = [
                        [finger_names.index("middle"), 2048, 2048, 2048, 2048,offsets[1]],
                        [finger_names.index("thumb"), 2048, 1024, 2048, 2048,offsets[3]],
                        [finger_names.index("index"), 2048, 2048, 2048, 2048,offsets[0]],
                        [finger_names.index("ring"), 2048, 2048, 2048, 2048,offsets[2]],
                        #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
                    ]
                node.publish_ordered_positions(data_to_send)
            
            elif len(input_parts) == 1 and input_parts[0].lower() == "2":
                node.publish_class(2)
                offsets = [0,0,0,800]
                data_to_send = [
                        [finger_names.index("middle"), 3486, 2048, 2369, 2929, offsets[1]],
                        [finger_names.index("thumb"), 3016, 591, 2980, 2483, offsets[3]],
                        [finger_names.index("index"), 3486, 2048, 2369, 2929, offsets[0]],
                        [finger_names.index("ring"), 3486, 2048, 2369, 2929, offsets[2]],
                        #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
                    ]
                
                node.publish_ordered_positions(data_to_send)


                time.sleep(2)
                

                offsets = [500,500,500,0]

                data_to_send = [
                        [finger_names.index("middle"), 2048, 2048, 2048, 2048,offsets[1]],
                        [finger_names.index("thumb"), 2048, 1024, 2048, 2048,offsets[3]],
                        [finger_names.index("index"), 2048, 2048, 2048, 2048,offsets[0]],
                        [finger_names.index("ring"), 2048, 2048, 2048, 2048,offsets[2]],
                        #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
                    ]
                node.publish_ordered_positions(data_to_send)
                
            elif len(input_parts) == 1 and input_parts[0].lower() == "4":
                node.publish_class(1)
                offsets = [0,0,0,0]
                data_to_send = [
                        [finger_names.index("middle"), 2800, 2048, 2048, 2048, offsets[1]],
                        [finger_names.index("thumb"), 4000, 4000, 2048, 2048, offsets[3]],
                        # [finger_names.index("thumb"), 3016, 591, 2980, 2483, offsets[3]],
                        [finger_names.index("index"), 2800, 2048, 2048, 2048, offsets[0]],
                        [finger_names.index("ring"), 2800, 2048, 2048, 2048, offsets[2]],
                        #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
                    ]
                

                time.sleep(0.5)
                node.publish_ordered_positions(data_to_send)

                offsets = [0,0,0,0]
                data_to_send = [
                        [finger_names.index("middle"), 2800, 2048, 3500, 2048, offsets[1]],
                        [finger_names.index("thumb"), 4000, 4500, 2048, 3000, offsets[3]],
                        # [finger_names.index("thumb"), 3016, 591, 2980, 2483, offsets[3]],
                        [finger_names.index("index"), 2800, 2048, 3500, 2048, offsets[0]],
                        [finger_names.index("ring"), 2800, 2048, 3500, 2048, offsets[2]],
                        #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
                    ]
                

                time.sleep(2)
                node.publish_ordered_positions(data_to_send)

                input("Press ENTER...")

                offsets = [0,0,0,0]

                data_to_send = [
                        [finger_names.index("middle"), 2048, 2048, 2048, 2048,offsets[1]],
                        [finger_names.index("thumb"), 2048, 1028, 2048, 2048,offsets[3]],
                        [finger_names.index("index"), 2048, 2048, 2048, 2048,offsets[0]],
                        [finger_names.index("ring"), 2048, 2048, 2048, 2048,offsets[2]],
                        #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
                    ]
                node.publish_ordered_positions(data_to_send)


            elif finger_name not in finger_names:
                node.get_logger().error("Nome do dedo inválido! Escolha entre: index, middle, ring, thumb.")
                continue

            elif len(input_parts) == 2 and input_parts[1].lower() in ["close", "open"]:
                command = input_parts[1].lower()
                if command == "close":
                    if finger_name == "middle":
                        positions = [3486, 2048, 2369, 2929]
                    elif finger_name == "thumb":
                        #positions = [2550, 630, 2935, 3030]
                        positions = [3016, 591, 2980, 2483]
                    elif finger_name == "index":
                        positions = [3486, 2048, 2369, 2929]
                    elif finger_name == "ring":
                        positions = [3486, 2048, 2369, 2929]
                elif command == "open":
                    if finger_name == "middle":
                        positions = [2048, 2048, 2048, 2048]
                    elif finger_name == "thumb":
                        positions = [2048, 1028, 2048, 2048]
                    elif finger_name == "index":
                        positions = [2048, 2048, 2048, 2048]
                    elif finger_name == "ring":
                        positions = [2048, 2048, 2048, 2048]
                data_to_send.extend([finger_names.index(finger_name)] + positions)
                node.get_logger().info(f'Dedo: {finger_name}, Posições: {positions}, Offsets: {offsets}')
                node.publish_positions(data_to_send) 

            

            else:
                input_values = " ".join(input_parts[1:])  # Ignorar o nome do dedo
                input_values = re.sub(r'[^0-9\s]', '', input_values)  # Remover caracteres não numéricos
                positions = list(map(int, input_values.split()))
                data_to_send.extend([finger_names.index(finger_name)] + positions)
                node.get_logger().info(f'Dedo: {finger_name}, Posições: {positions}, Offsets: {offsets}')
                node.publish_positions(data_to_send) 
            
            

    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

