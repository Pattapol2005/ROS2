import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tkinter as tk
import math
import tf_transformations

class AccurateRotateGUI(Node):
    def __init__(self):
        super().__init__('accurate_rotate_gui')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # หมุน - ต้องมาก่อน GUI!
        self.angular_speed = math.radians(30)
        self.start_yaw = None
        self.current_yaw = 0.0
        self.target_delta = 0.0
        self.rotating = False

        # GUI
        self.window = tk.Tk()
        self.window.title("🧭 Precise Rotation GUI")

        btn180 = tk.Button(self.window, text="🔄 หมุน 180°", command=lambda: self.start_rotation(180))
        btn180.pack(pady=10)

        btn360 = tk.Button(self.window, text="🔄 หมุน 360°", command=lambda: self.start_rotation(360))
        btn360.pack(pady=10)

        self.window.after(100, self.tk_loop)
        self.window.mainloop()


    def odom_callback(self, msg):
        # แปลง quaternion → yaw
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = tf_transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.current_yaw = yaw

    def start_rotation(self, degrees):
        if self.rotating:
            return
        self.start_yaw = self.current_yaw
        self.target_delta = math.radians(degrees)
        self.rotating = True
        self.get_logger().info(f"📍 เริ่มหมุน {degrees}° จากมุม {math.degrees(self.start_yaw):.2f}°")

    def yaw_diff(self, start, current):
        # คำนวณระยะทางเชิงมุมแบบหาค่า abs(delta yaw) (wraparound 360°)
        delta = current - start
        return abs(math.atan2(math.sin(delta), math.cos(delta)))

    def tk_loop(self):
        rclpy.spin_once(self, timeout_sec=0)

        if self.rotating:
            delta_yaw = self.yaw_diff(self.start_yaw, self.current_yaw)
            if delta_yaw < self.target_delta:
                twist = Twist()
                twist.angular.z = self.angular_speed
                self.publisher.publish(twist)
            else:
                self.publisher.publish(Twist())  # stop
                self.rotating = False
                self.get_logger().info(f"✅ หมุนครบ {math.degrees(self.target_delta):.0f}° แล้ว")

        self.window.after(10, self.tk_loop)

def main(args=None):
    rclpy.init(args=args)
    gui = AccurateRotateGUI()
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
