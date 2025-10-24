import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ReadCurrentSubscriber(Node):
    def __init__(self):
        super().__init__('read_current_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            '/dynamixel/current',
            self.position_callback,
            10
        )
        self.subscription 
    
    def position_callback(self, msg):
        self.get_logger().info(f'Corrente atual: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ReadCurrentSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()