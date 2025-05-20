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
        
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_vel = 0.0
        self.cmd_vel_timer = self.create_timer(0.1, self.publish_cmd_vel)

        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._current_goal_handle = None  

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.linear.y = self.linear_y  
        msg.angular.z = self.angular_vel
        self.cmd_vel_pub.publish(msg)

    def set_velocity(self, linear_x, linear_y, angular):
        self.linear_x = linear_x
        self.linear_y = linear_y
        self.angular_vel = angular

    def stop_movement(self, event=None):
        self.set_velocity(0.0, 0.0, 0.0)

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
        result = future.result().result
        self.get_logger().info(f'Nav2 result: {result}')
        self._current_goal_handle = None
        self.get_logger().info('Navigation finished.')

    def cancel_goal(self):
        """ยกเลิก Navigation Goal"""
        if self._current_goal_handle is None:
            self.get_logger().warn('No current goal to cancel.')
            return
        
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
        self.root.title("Manual")

        self.max_linear_speed = 0.4

        # ====== ปุ่ม Manual Control ======
        control_frame = tk.Frame(root)
        control_frame.pack(pady=10)

        self.btn_up = tk.Button(control_frame, text="▲", width=5, height=2)
        self.btn_down = tk.Button(control_frame, text="▼", width=5, height=2)
        self.btn_left = tk.Button(control_frame, text="◀", width=5, height=2)
        self.btn_right = tk.Button(control_frame, text="▶", width=5, height=2)

        self.btn_up.grid(row=0, column=1, padx=5, pady=5)
        self.btn_left.grid(row=1, column=0, padx=5, pady=5)
        self.btn_right.grid(row=1, column=2, padx=5, pady=5)
        self.btn_down.grid(row=2, column=1, padx=5, pady=5)

        # กดปุ่มแล้วเคลื่อนที่, ปล่อยปุ่มแล้วหยุด
        self.btn_up.bind("<ButtonPress-1>", lambda e: self.node.set_velocity(self.max_linear_speed, 0.0, 0.0))
        self.btn_down.bind("<ButtonPress-1>", lambda e: self.node.set_velocity(-self.max_linear_speed, 0.0, 0.0))
        self.btn_left.bind("<ButtonPress-1>", lambda e: self.node.set_velocity(0.0, self.max_linear_speed, 0.0))
        self.btn_right.bind("<ButtonPress-1>", lambda e: self.node.set_velocity(0.0, -self.max_linear_speed, 0.0))

        self.btn_up.bind("<ButtonRelease-1>", self.node.stop_movement)
        self.btn_down.bind("<ButtonRelease-1>", self.node.stop_movement)
        self.btn_left.bind("<ButtonRelease-1>", self.node.stop_movement)
        self.btn_right.bind("<ButtonRelease-1>", self.node.stop_movement)

        # ====== ปุ่มนำทาง และ ยกเลิกนำทาง ======

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNav2Node()

    root = tk.Tk()
    gui = JoystickGUI(root, node)

    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
