import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Int32
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import time
import math
from tf_transformations import quaternion_from_euler
from rcl_interfaces.msg import ParameterValue, ParameterType
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.srv import SetParameters
import rclpy
from rcl_interfaces.msg import Parameter
from sensor_msgs.msg import LaserScan
from rclpy.executors import MultiThreadedExecutor
import pygame
import os
from std_msgs.msg import Float32


pygame.init()
pygame.mixer.init()
pygame.mixer.music.load(r"/home/robot-ros/ros2_ws/src/Het-cone/beep.mp3")
os.system("pactl set-sink-volume @DEFAULT_SINK@ 150%")


class IRController(Node):
    def __init__(self):
        super().__init__('center_controller')
        self.ir1 = None
        self.ir2 = None
        self.ir3 = None
        self.analog1 = None
        self.analog2 = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Bool, '/sensor/ir1', self.ir1_callback, 30)
        self.create_subscription(Bool, '/sensor/ir2', self.ir2_callback, 30)
        self.create_subscription(Bool, '/sensor/ir3', self.ir3_callback, 30)
        self.create_subscription(Int32, '/sensor/analog1', self.analog1_callback, 30)
        self.create_subscription(Int32, '/sensor/analog2', self.analog2_callback, 30)

    def ir1_callback(self, msg):
        self.ir1 = msg.data

    def ir2_callback(self, msg):
        self.ir2 = msg.data

    def ir3_callback(self, msg):
        self.ir3 = msg.data

    def analog1_callback(self, msg):
        self.analog1 = msg.data

    def analog2_callback(self, msg):
        self.analog2 = msg.data

    def callback_cmd_vel(self, x=0.0, y=0.0, z=0.0):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = z
        self.pub.publish(msg)

class GoalFeedback(Node):
    def __init__(self):
        super().__init__('goal_feedback_node')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_distance = None

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.current_distance = distance

    def send_goal(self, x, y, yaw_degrees):
        theta = math.radians(yaw_degrees)
        q = quaternion_from_euler(0, 0, theta)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self._client.wait_for_server()
        future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()

        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Goal finished")
        pygame.mixer.music.play()





class AMCLControl(Node):
    def __init__(self):
        super().__init__('amcl_control')
        self.cli = self.create_client(SetParameters, '/amcl/set_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for AMCL service...')

    def set_amcl_update_paused(self, pause=True):
        req = SetParameters.Request()

        if pause:
            update_d = 1000.0  # Very high distance to suppress updates
            update_a = 6.28    # Very high angle (approx 360°)
        else:
            update_d = 0.2     # Default or suitable value for your system
            update_a = 0.2

        req.parameters = [
            self._make_param('update_min_d', update_d),
            self._make_param('update_min_a', update_a)
        ]

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('AMCL update parameters set.')
    def _make_param(self, name, value):
        param = Parameter()
        param.name = name
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = value
        return param


class LaserScanListener(Node):
    def __init__(self):
        super().__init__('laser_scan_listener_temp')
        self.latest_scan = None
        self.subscription = self.create_subscription(
            LaserScan,
            '/base/scan/unfiltered',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.latest_scan = msg

    def get_distance_at_angle(self, angle_deg):
        if self.latest_scan is None:
            return float('nan')
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - self.latest_scan.angle_min) / self.latest_scan.angle_increment)
        if 0 <= index < len(self.latest_scan.ranges):
            return self.latest_scan.ranges[index]
        return float('nan')


def getDis(angle_deg, node):
    if node.latest_scan is None:
        return float('nan')

    angle_rad = math.radians(angle_deg)
    index = int((angle_rad - node.latest_scan.angle_min) / node.latest_scan.angle_increment)

    if 0 <= index < len(node.latest_scan.ranges):
        distance = node.latest_scan.ranges[index]
        if math.isinf(distance) or math.isnan(distance):
            return float('nan')
        return distance

    return float('nan')


def publish_initial_pose(x, y, yaw_degrees):
    node = rclpy.create_node('set_initial_pose')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()

    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0

    yaw_radians = math.radians(yaw_degrees)
    from tf_transformations import quaternion_from_euler
    q = quaternion_from_euler(0, 0, yaw_radians)
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]

    msg.pose.covariance[0] = 0.25  # x
    msg.pose.covariance[7] = 0.25  # y
    msg.pose.covariance[35] = 0.06853891945200942  # yaw

    for _ in range(10):  
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"Initial pose set at x={x}, y={y}, yaw={yaw_degrees}°")
    node.destroy_node()

def pause_amcl():
    amcl = AMCLControl()
    amcl.set_amcl_update_paused(pause=True)
    amcl.destroy_node()

def resume_amcl():
    amcl = AMCLControl()
    amcl.set_amcl_update_paused(pause=False)
    amcl.destroy_node()

def spin_nodes(nodes, stop_event):
    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        while not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        for node in nodes:
            executor.remove_node(node)

def moveForword(duration):
    node = IRController()

    try:
        start_time = time.time()
        while time.time() - start_time < duration:
            node.callback_cmd_vel(0.7, 0.0, 0.0)
            time.sleep(0.1)
        node.callback_cmd_vel(0.0, 0.0, 0.0)
    finally:
        node.destroy_node()

def moveBack(duration):
    node = IRController()

    try:
        start_time = time.time()
        while time.time() - start_time < duration:
            node.callback_cmd_vel(-0.7, 0.0, 0.0)
            time.sleep(0.1)
        node.callback_cmd_vel(0.0, 0.0, 0.0)
    finally:
        node.destroy_node()

def moveRight(duration):
    node = IRController()

    try:
        start_time = time.time()
        while time.time() - start_time < duration:
            node.callback_cmd_vel(0.0, -0.7, 0.0)
            time.sleep(0.1)
        node.callback_cmd_vel(0.0, 0.0, 0.0)
    finally:
        node.destroy_node()


def center(sensor_node, laser_node):
    try:
        while sensor_node.analog1 is None and sensor_node.analog2 is None and sensor_node.ir1 is None and sensor_node.ir2 is None:
            time.sleep(0.1)

        while sensor_node.ir3:
            while rclpy.ok():
                while getDis(180, laser_node) > 0.643:
                    sensor_node.callback_cmd_vel(0.5, 0.0, 0.0)
                    print(f"lidar: {getDis(180, laser_node)}")
                    time.sleep(0.1)
                pygame.mixer.music.play()
                sensor_node.callback_cmd_vel(0.0, 0.0, 0.0)
                time.sleep(0.1)

                if not sensor_node.ir2 and sensor_node.ir1:
                    while not sensor_node.ir2 and sensor_node.ir1:
                        sensor_node.callback_cmd_vel(0.0, 0.2, 0.0)
                        print(f"L: L={sensor_node.ir2} R={sensor_node.ir1}")
                        time.sleep(0.1)
                    sensor_node.callback_cmd_vel(0.0, -0.4, 0.0)
                    time.sleep(0.05)
                    sensor_node.callback_cmd_vel(0.0, 0.0, 0.0)
                    time.sleep(0.1)

                elif sensor_node.ir2 and not sensor_node.ir1:
                    while sensor_node.ir2 and not sensor_node.ir1:
                        sensor_node.callback_cmd_vel(0.0, -0.2, 0.0)
                        print(f"R: L={sensor_node.ir2} R={sensor_node.ir1}")
                        time.sleep(0.05)
                    sensor_node.callback_cmd_vel(0.0, 0.4, 0.0)
                    time.sleep(0.1)
                    sensor_node.callback_cmd_vel(0.0, 0.0, 0.0)
                    time.sleep(0.1)

                else:
                    if (sensor_node.ir1 and sensor_node.ir2) or (not sensor_node.ir1 and not sensor_node.ir2):
                        print(f"STOP: L={sensor_node.ir2} R={sensor_node.ir1}")
                        time.sleep(0.5)
                        if (sensor_node.ir1 and sensor_node.ir2) or (not sensor_node.ir1 and not sensor_node.ir2):
                            break

            while sensor_node.ir3 or (sensor_node.ir2 > 400):
                sensor_node.callback_cmd_vel(0.2, 0.0, 0.0)
                print(f"C: ={sensor_node.ir3}")
                time.sleep(0.1)
            pygame.mixer.music.play()
            sensor_node.callback_cmd_vel(0.0, 0.0, 0.0)

    finally:
        pass


def main(args=None):
    rclpy.init(args=args)
    laser_node = LaserScanListener()
    sensor_node = IRController()
    goal_feedback_node = GoalFeedback()

    stop_event = threading.Event()

    spin_thread_obj = threading.Thread(target=spin_nodes, args=([laser_node, sensor_node , goal_feedback_node ], stop_event))
    spin_thread_obj.start()

    try:
        resume_amcl()
        pygame.mixer.music.play()
        goal_feedback_node.send_goal(-0.81468093,2.8332419395446777,0) #room 1 box 
        pause_amcl()
        while (getDis(-90, laser_node) > 0.516) :
            sensor_node.callback_cmd_vel(0.0, 0.2, 0.0)
            time.sleep(0.05)
        sensor_node.callback_cmd_vel(0.0, 0.0, 0.0)
        time.sleep(1)
        #publish_initial_pose(-0.81468093,2.8332419395446777,0)
        center(sensor_node, laser_node)
        time.sleep(1)
        moveBack(0.4)
        moveRight(1.23)
        moveForword(0.4)
        #resume_amcl()
        #send_goal(-0.81468093,2.048139810562134,0)
        #publish_initial_pose(-0.81468093,2.048139810562134,0)
        #pause_amcl()
        center(sensor_node, laser_node)
        time.sleep(1)
        moveBack(0.4)
        moveRight(1.23)
        moveForword(0.4)
        #resume_amcl()
        #send_goal(-0.81468093,1.238612174987793,0)
        #publish_initial_pose(-0.81468093,1.238612174987793,0)
        #pause_amcl()
        center(sensor_node, laser_node)
        time.sleep(1)
        moveBack(0.4)
        moveRight(1.23)
        moveForword(0.4)
        #resume_amcl()
        #send_goal(-0.81468093,0.4416578710079193,0)
        #publish_initial_pose(-0.81468093,0.4416578710079193,0)
        #pause_amcl()
        center(sensor_node, laser_node)
        time.sleep(1)
        #resume_amcl()
        #send_goal(2.88,-1.95,0)
        #pause_amcl()
        center(sensor_node, laser_node)
        moveBack(0.4)
        resume_amcl()
        publish_initial_pose(-1.3386293829169593,0.4414943189483385,0) #Exit room 1 box 4 publish point
        goal_feedback_node.send_goal( -3.5744550659048163,0.520881203576909,0)
        while goal_feedback_node.current_distance is not None and goal_feedback_node.current_distance > 0.5 :
            time.sleep(0.1)
        goal_feedback_node.send_goal(-0.6590873003005981,-1.4398014545440674,0) #room2 box1
        pause_amcl()
        center(sensor_node, laser_node)
        moveBack(0.5)
        resume_amcl()

        goal_feedback_node.send_goal(-2.171846389770508 , -3.3408892154693604 , 270) #room 2 box2
        pause_amcl()
        center(sensor_node, laser_node)
        moveBack(0.6)
        resume_amcl()

        goal_feedback_node.send_goal(2.8417744636535645 , -1.9033788204193115 , 0) #room 3
        pause_amcl()
        center(sensor_node, laser_node)
        moveBack(0.5)
        resume_amcl()

        goal_feedback_node.send_goal(2.5555219650268555 , 1.8650868082046509 , 0) # room 4
        pause_amcl()
        while (getDis(180, laser_node) > 0.483) :
            sensor_node.callback_cmd_vel(0.3, 0.0, 0.0)
            time.sleep(0.05)
        sensor_node.callback_cmd_vel(0.0, 0.0, 0.0)
        moveBack(0.5)
        resume_amcl()

        goal_feedback_node.send_goal(0.040167699100179824,-2.9876335063219037,0)
        while goal_feedback_node.current_distance is not None and goal_feedback_node.current_distance > 0.3 :
            time.sleep(0.1)

        goal_feedback_node.send_goal(-3.7984583377838135 , 3.244971513748169 , -0.00299072265625) #home



    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        spin_thread_obj.join()
        laser_node.destroy_node()
        sensor_node.destroy_node()
        goal_feedback_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
