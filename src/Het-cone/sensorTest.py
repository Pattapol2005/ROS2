import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool

class SensorListener(Node):
    def __init__(self):
        super().__init__('sensor_listener_node')

        self.analog1_sub = self.create_subscription(
            Int32,
            '/sensor/analog1',
            self.analog1_callback,
            10)
        self.analog2_sub = self.create_subscription(
            Int32,
            '/sensor/analog2',
            self.analog2_callback,
            10)

        self.ir1_sub = self.create_subscription(
            Bool,
            '/sensor/ir1',
            self.ir1_callback,
            10)
        self.ir2_sub = self.create_subscription(
            Bool,
            '/sensor/ir2',
            self.ir2_callback,
            10)
        self.ir2_sub = self.create_subscription(
            Bool,
            '/sensor/ir3',
            self.ir3_callback,
            10)

    def analog1_callback(self, msg):
        self.get_logger().info(f'Analog1: {msg.data}')

    def analog2_callback(self, msg):
        self.get_logger().info(f'Analog2: {msg.data}')

    def ir1_callback(self, msg):
        self.get_logger().info(f'IR1: {"True" if msg.data else "False"}')

    def ir2_callback(self, msg):
        self.get_logger().info(f'IR2: {"True" if msg.data else "False"}')
    
    def ir3_callback(self, msg):
        self.get_logger().info(f'IR3: {"True" if msg.data else "False"}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





