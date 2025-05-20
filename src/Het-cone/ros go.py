#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose

import tkinter as tk
from tf_transformations import quaternion_from_euler

class TeleopNav2Node(Node):
    def __init__(self):
        super().__init__('teleop_nav2_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.cmd_vel_timer = self.create_timer(0.1, self.publish_cmd_vel)
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._current_goal_handle = None  

    def publish_cmd_vel(self):
        """Callback ของ Timer ที่จะ Publish cmd_vel ตามค่าที่ตั้งไว้"""
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.cmd_vel_pub.publish(msg)

    def set_velocity(self, linear, angular):
        """เปลี่ยนค่าความเร็ว (m/s, rad/s) สำหรับ Teleop"""
        self.linear_vel = linear
        self.angular_vel = angular

    def send_goal(self, x, y, yaw_deg):
        """
        ส่งเป้าหมาย (x, y, yaw องศา) ให้ Nav2 ผ่าน ActionClient navigate_to_pose
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map' 
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)

        yaw_rad = math.radians(yaw_deg)
        q = quaternion_from_euler(0.0, 0.0, yaw_rad)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.nav_action_client.wait_for_server()
        self.get_logger().info(f"Sending goal => x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f} deg")
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback หลังส่ง Goal แล้วดูว่าได้รับการ Accept หรือ Reject"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal Rejected')
            self._current_goal_handle = None
            return

        self.get_logger().info('Goal Accepted!')
        self._current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback เมื่อ Nav2 นำทางเสร็จ (ถึงเป้า, ถูกยกเลิก, หรือมี error)"""
        result = future.result().result
        self.get_logger().info(f'Nav2 result: {result}')
        self._current_goal_handle = None
        self.get_logger().info('Navigation finished.')
        self.get_logger().info('Canceling current goal...')
        cancel_future = self._current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal cancel requested.')
        else:
            self.get_logger().warn('Failed to cancel goal or it was already completed.')
        self._current_goal_handle = None


class JoystickGUI:
    def __init__(self, root, node: TeleopNav2Node):
        self.root = root
        self.node = node
        self.root.title("ROS2 Teleop + Point 1 2 3")

        self.canvas_size = 300
        self.joy_radius = 100
        self.thumb_radius = 20
        self.center_x = self.canvas_size // 2
        self.center_y = self.canvas_size // 2

        self.max_linear_speed = 0.3   # m/s
        self.max_angular_speed = 0.5 # rad/s

        self.canvas = tk.Canvas(root, width=self.canvas_size, height=self.canvas_size, bg="white")
        self.canvas.pack(pady=5)

        self.canvas.create_oval(
            self.center_x - self.joy_radius, 
            self.center_y - self.joy_radius,
            self.center_x + self.joy_radius,
            self.center_y + self.joy_radius,
            outline="black", width=2
        )

        self.thumb = self.canvas.create_oval(
            self.center_x - self.thumb_radius,
            self.center_y - self.thumb_radius,
            self.center_x + self.thumb_radius,
            self.center_y + self.thumb_radius,
            fill="blue"
        )

        self.canvas.bind("<Button-1>", self.on_mouse_down)
        self.canvas.bind("<B1-Motion>", self.on_mouse_move)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_up)

        self.dragging = False

        btn_frame = tk.Frame(root)
        btn_frame.pack(pady=5)

        self.btn_nav = tk.Button(btn_frame, text="Go 1", 
                                 command=lambda: self.node.send_goal(0.27797596645355225,0.10646136140823364,0))
        self.btn_nav2 = tk.Button(btn_frame, text="Go 2", 
                                 command=lambda: self.node.send_goal(8.423438430786133,0.171136140823364,0))
        self.btn_nav3 = tk.Button(btn_frame, text="Go 3", 
                                 command=lambda: self.node.send_goal(12.23546768188477,-2.3001368489837646,0))
        self.btn_cancel = tk.Button(btn_frame, text="Cancel Nav")

        self.btn_nav.pack(side=tk.LEFT, padx=5)
        self.btn_nav2.pack(side=tk.LEFT, padx=5)
        self.btn_nav3.pack(side=tk.LEFT, padx=5)
        self.btn_cancel.pack(side=tk.LEFT, padx=5)

    # ---------------------- Joystick Event ----------------------
    def on_mouse_down(self, event):
        if self.is_inside_thumb(event.x, event.y):
            self.dragging = True

    def on_mouse_move(self, event):
        if not self.dragging:
            return

        dx = event.x - self.center_x
        dy = event.y - self.center_y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > self.joy_radius:
            scale = self.joy_radius / dist
            dx *= scale
            dy *= scale

        new_x = self.center_x + dx
        new_y = self.center_y + dy
        self.canvas.coords(
            self.thumb,
            new_x - self.thumb_radius,
            new_y - self.thumb_radius,
            new_x + self.thumb_radius,
            new_y + self.thumb_radius
        )
        normalized_dy = -dy / self.joy_radius
        normalized_dx = dx / self.joy_radius

        linear = normalized_dy * self.max_linear_speed
        angular = normalized_dx * self.max_angular_speed

        self.node.set_velocity(linear, angular)

    def on_mouse_up(self, event):
        self.dragging = False
        self.canvas.coords(
            self.thumb,
            self.center_x - self.thumb_radius,
            self.center_y - self.thumb_radius,
            self.center_x + self.thumb_radius,
            self.center_y + self.thumb_radius
        )
        self.node.set_velocity(0.0, 0.0)

    def is_inside_thumb(self, x, y):
        thumb_coords = self.canvas.coords(self.thumb)  # [x1, y1, x2, y2]
        cx_thumb = (thumb_coords[0] + thumb_coords[2]) / 2
        cy_thumb = (thumb_coords[1] + thumb_coords[3]) / 2
        r = self.thumb_radius
        dist = math.sqrt((x - cx_thumb)**2 + (y - cy_thumb)**2)
        return dist <= r

def main(args=None):
    rclpy.init(args=args)

    node = TeleopNav2Node()

    root = tk.Tk()
    gui = JoystickGUI(root, node)

    def spin_ros():
        rclpy.spin(node)

    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
