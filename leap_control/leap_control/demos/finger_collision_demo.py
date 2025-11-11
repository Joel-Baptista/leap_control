import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import re
import numpy as np
import time
import copy

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

DESCRIPTION = "Closure modes:\n0 -> Thumb under fingers (collision)\n1 -> Thumb under fingers (free)\n2 -> Thumb over fingers (collision)\n3 -> Thumb over fingers (free)"

def append_offsets(finger_joints, offsets):
    fj = copy.deepcopy(finger_joints)
    for joints in fj:
        joints.append(offsets[joints[0]])

    return fj

finger_names = ["index", "middle", "ring", "thumb"]
UNDER_THUMB = [
    [finger_names.index("middle"), 3486, 2048, 2369, 2929],
    [finger_names.index("thumb"), 3016, 591, 2980, 2483],
    [finger_names.index("index"), 3486, 2048, 2369, 2929],
    [finger_names.index("ring"), 3486, 2048, 2369, 2929],
]

OPEN_HAND = [
    [finger_names.index("middle"), 2048, 2048, 2048, 2048],
    [finger_names.index("thumb"), 2048, 1024, 2048, 2048],
    [finger_names.index("index"), 2048, 2048, 2048, 2048],
    [finger_names.index("ring"), 2048, 2048, 2048, 2048],
]

OVER_THUMB = [
    [finger_names.index("middle"), 4000, 2048, 2500, 2500],
    [finger_names.index("thumb"), 3016, 1200, 2980, 3000],
    [finger_names.index("index"), 4000, 2048, 2500, 2500],
    [finger_names.index("ring"), 4000, 2048, 2500, 2500],
]




def main(args=None):
    rclpy.init(args=args)
    node = SetPositions()
    finger_names = ["index", "middle", "ring", "thumb"]
    positions = []

    print(DESCRIPTION)
    
    try:
        while rclpy.ok():
            input_str = input("Input closure mode: ").strip()
            input_parts = input_str.split()


            print(input_parts)
            if not input_parts:
                continue
            finger_name = input_parts[0].lower()

            data_to_send = []
            offsets = [0,0,0,0]

    
            if input_parts[0].lower() == "0":
                node.publish_class(0)
                offsets = [300,300,300,0]
                data_to_send = append_offsets(UNDER_THUMB, offsets)
                
                node.publish_ordered_positions(data_to_send)


                time.sleep(2)


                data_to_send = append_offsets(OPEN_HAND, offsets)
                
                node.publish_ordered_positions(data_to_send)

            
            elif input_parts[0].lower() == "1":
                node.publish_class(0)
                offsets = [500,500,500,0]
                data_to_send = append_offsets(UNDER_THUMB, offsets)
                
                node.publish_ordered_positions(data_to_send)

                time.sleep(2)

                offsets = [0,0,0,500]

                data_to_send = append_offsets(OPEN_HAND, offsets)
                node.publish_ordered_positions(data_to_send)
            
            elif input_parts[0].lower() == "2":
                node.publish_class(2)
                offsets = [0,0,0,1000]
                data_to_send = append_offsets(OVER_THUMB, offsets)
                
                node.publish_ordered_positions(data_to_send)

                time.sleep(2)

                offsets = [0,0,0,1000]

                data_to_send = append_offsets(OPEN_HAND, offsets)
                node.publish_ordered_positions(data_to_send)
            
            elif input_parts[0].lower() == "3":
                node.publish_class(2)
                offsets = [0,0,0,1200]
                data_to_send = append_offsets(OVER_THUMB, offsets)
                
                node.publish_ordered_positions(data_to_send)


                time.sleep(2)
                

                offsets = [500,500,500,0]

                data_to_send = append_offsets(OPEN_HAND, offsets)
                node.publish_ordered_positions(data_to_send)

            else:
                print(f"Input '{input_str}' not defined")
                print(DESCRIPTION)
        
    except KeyboardInterrupt:
        print("Error")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

