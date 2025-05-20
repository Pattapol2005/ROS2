#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time
import threading
import tkinter as tk

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # สร้าง Action Client เพื่อส่งเป้าหมายไปยัง Nav2
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # กำหนด Waypoints 3 จุด (X, Y, Theta)
        self.waypoints = [
            (-6.078758, 2.056818 , 0.1),  # จุดที่ 1
            (1.326935, 7.288975, 0.1), # จุดที่ 2
            (8.246890, 7.865981, 0.1) # จุดที่ 3
        ]
        self.current_index = 0  # เริ่มจากจุดที่ 1
        self.is_running = True  # ใช้ตรวจสอบสถานะการเดินทาง

        self.client.wait_for_server()
        self.navigate_to_next_waypoint()

    def navigate_to_next_waypoint(self):
        """ ส่งคำสั่งให้หุ่นยนต์ไปยัง Waypoint ถัดไป """
        if not self.is_running:
            return

        if self.current_index >= len(self.waypoints):
            self.current_index = 0  # รีเซ็ตให้เดินวนใหม่

        x, y, theta = self.waypoints[self.current_index]
        goal = NavigateToPose.Goal()

        # ตั้งค่า PoseStamped เป้าหมาย
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = theta  # ใช้ theta ใน quaternion
        
        self.get_logger().info(f'Navigating to waypoint {self.current_index + 1}: ({x}, {y})')

        update_gui_status(f'กำลังไปจุดที่ {self.current_index + 1}')

        # ส่งคำสั่งไปยัง Nav2
        self.client.send_goal_async(goal).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted!')

        goal_handle.get_result_async().add_done_callback(self.goal_completed_callback)

    def goal_completed_callback(self, future):
        if not self.is_running:
            return

        self.get_logger().info(f'Waypoint {self.current_index + 1} reached!')
        self.current_index += 1
        time.sleep(2)  # รอ 2 วินาทีก่อนส่งคำสั่งไปจุดถัดไป
        self.navigate_to_next_waypoint()

    def cancel_navigation(self):
        """ หยุดการนำทาง """
        self.is_running = False
        update_gui_status("ยกเลิกการนำทาง")

    def restart_navigation(self):
        """ เริ่มต้นใหม่จากจุดแรก """
        self.is_running = True
        self.current_index = 0
        update_gui_status("เริ่มต้นใหม่...")
        time.sleep(1)
        self.navigate_to_next_waypoint()

# ---------- Tkinter GUI ----------
def start_ros_node():
    global node
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def update_gui_status(text):
    """ อัปเดตข้อความใน GUI """
    status_label.config(text=text)

def on_cancel():
    """ เมื่อกดปุ่มยกเลิก """
    node.cancel_navigation()

def on_restart():
    """ เมื่อกดปุ่มเริ่มต้นใหม่ """
    node.restart_navigation()

# สร้างหน้าต่าง GUI
root = tk.Tk()
root.title("Waypoint Navigation")
root.geometry("300x150")

status_label = tk.Label(root, text="กำลังเริ่มต้น...", font=("Arial", 12))
status_label.pack(pady=10)

cancel_button = tk.Button(root, text="ยกเลิก", command=on_cancel, fg="white", bg="red", font=("Arial", 12))
cancel_button.pack(pady=5)

restart_button = tk.Button(root, text="เริ่มต้นใหม่", command=on_restart, fg="white", bg="green", font=("Arial", 12))
restart_button.pack(pady=5)

# รัน ROS 2 ใน Thread แยก
ros_thread = threading.Thread(target=start_ros_node, daemon=True)
ros_thread.start()

# เริ่ม GUI
root.mainloop()
