import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserScanListener(Node):

    def __init__(self):
        super().__init__('laser_scan_listener')
        self.subscription = self.create_subscription(
            LaserScan,
            '/base/scan/unfiltered',
            self.listener_callback,
            10)
        self.subscription 

    def listener_callback(self, msg):
        angle_deg = -90
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - msg.angle_min) / msg.angle_increment)
        if 0 <= index < len(msg.ranges):
            angle_distance = msg.ranges[index]
            self.get_logger().info(f'Distance at {angle_deg}°: {angle_distance:.3f} m')

def main(args=None):
    rclpy.init(args=args)
    laser_scan_listener = LaserScanListener()
    rclpy.spin(laser_scan_listener)
    laser_scan_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
