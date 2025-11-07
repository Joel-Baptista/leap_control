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

    ## Pick Up Ball

    offsets = [500,700,900,0]
    data_to_send = [
            [finger_names.index("middle"), 3486, 2048, 2369, 2929, offsets[1]],
            [finger_names.index("thumb"), 3016, 591, 2980, 2483, offsets[3]],
            [finger_names.index("index"), 3486, 2048, 2369, 2929, offsets[0]],
            [finger_names.index("ring"), 3486, 2048, 2369, 2929, offsets[2]],
            #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
        ]
    
    time.sleep(1)
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

    node.publish_class(2)
    offsets = [0,0,0,300]
    data_to_send = [
            [finger_names.index("middle"), 3486, 2048, 2369, 2929, offsets[1]],
            # [finger_names.index("thumb"), 2550, 630, 2935, 3030, offsets[3]],
            [finger_names.index("thumb"), 3000, 1300, 3000, 2500, offsets[3]],
            [finger_names.index("index"), 3486, 2048, 2369, 2929, offsets[0]],
            [finger_names.index("ring"), 3486, 2048, 2369, 2929, offsets[2]],
            # [finger_names.index("thumb"), 3016, 591, 2980, 2483, offsets[3]],
        ]
    
    time.sleep(3)
    node.publish_ordered_positions(data_to_send)

    offsets = [0, 0, 0,500]

    data_to_send = [
            [finger_names.index("middle"), 2048, 2048, 2048, 2048,offsets[1]],
            [finger_names.index("thumb"), 2048, 1024, 2048, 2048,offsets[3]],
            [finger_names.index("index"), 2048, 2048, 2048, 2048,offsets[0]],
            [finger_names.index("ring"), 2048, 2048, 2048, 2048,offsets[2]],
            #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
        ]
    

    time.sleep(5.5)
    node.publish_ordered_positions(data_to_send)

    # input("Press ENTER...")

    ## Pick Up Block 

    node.publish_class(1)
    offsets = [0,0,0,0]
    data_to_send = [
            [finger_names.index("middle"), 2800, 2048, 2048, 2048, offsets[1]],
            [finger_names.index("thumb"), 4000, 10, 2048, 2048, offsets[3]],
            # [finger_names.index("thumb"), 3016, 591, 2980, 2483, offsets[3]],
            [finger_names.index("index"), 2800, 2048, 2048, 2048, offsets[0]],
            [finger_names.index("ring"), 2800, 2048, 2048, 2048, offsets[2]],
            #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
        ]
    

    time.sleep(4)
    node.publish_ordered_positions(data_to_send)

    offsets = [0,0,0,0]
    data_to_send = [
            [finger_names.index("middle"), 2800, 2048, 3000, 2048, offsets[1]],
            [finger_names.index("thumb"), 4000, 10, 2048, 3000, offsets[3]],
            # [finger_names.index("thumb"), 3016, 591, 2980, 2483, offsets[3]],
            [finger_names.index("index"), 2800, 2048, 3000, 2048, offsets[0]],
            [finger_names.index("ring"), 2800, 2048, 3000, 2048, offsets[2]],
            #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
        ]
    

    time.sleep(1.5)
    # time.sleep(2)
    node.publish_ordered_positions(data_to_send)

    offsets = [0,0,0,0]

    data_to_send = [
            [finger_names.index("middle"), 2048, 2048, 2048, 2048,offsets[1]],
            [finger_names.index("thumb"), 2048, 1024, 2048, 2048,offsets[3]],
            [finger_names.index("index"), 2048, 2048, 2048, 2048,offsets[0]],
            [finger_names.index("ring"), 2048, 2048, 2048, 2048,offsets[2]],
            #[finger_names.index("thumb"), 2550, 630, 2935, 3030]
        ]
    
    time.sleep(9.0)
    node.publish_ordered_positions(data_to_send)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

