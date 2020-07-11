from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from std_srvs.srv import Empty
import sys
from turtlebot3_msgs.srv import Dqn
from turtlebot3_msgs.srv import Goal
from turtlebot3_rl.turtlebot3_gazebo import turtlebot3_gazebo
import numpy as np
import math
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
class RLEnvironment(Node):
    def __init__(self):
        super().__init__('rl_environment')

        """**************************************************************
                                Initialize variables
        **************************************************************"""
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.robot_pose_theta = 0.0

        self.action_size = 5
        self.done = False
        self.fail = False
        self.succeed = False

        self.goal_angle = 0.0
        self.goal_distance = 1.0
        self.init_goal_distance = 1.0
        self.scan_ranges = []
        self.min_obstacle_distance = 10.0

        self.local_step = 0

        self.stop_cmd_vel_timer = None
        self.angular_vel = [1.4, 0.7, 0.0, -0.7, -1.4]
        """************************************************************
                 Initialise publisher, subscribers, clients and services
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialize publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.subscriper_grp = MutuallyExclusiveCallbackGroup()
        # Initialize subscribers
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_sub_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_sub_callback, qos_profile_sensor_data)

        # Initialize client
        self.a = MutuallyExclusiveCallbackGroup()
        self.b = MutuallyExclusiveCallbackGroup()
        self.c = MutuallyExclusiveCallbackGroup()

        self.task_succeed_client = self.create_client(Goal, 'task_succeed',callback_group=self.a)
        self.task_failed_client = self.create_client(Goal, 'task_failed',callback_group=self.a)
        self.initialize_environment_client = self.create_client(Goal, 'initialize_env',callback_group=self.a)

        # Initialize service
        self.rl_agent_interface_service = self.create_service(Dqn, 'rl_agent_interface',
                                                              self.rl_agent_interface_callback)

        # initialize env
        self.initialize_environment()

    def initialize_environment(self):

        while not self.initialize_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for initialize the environment is not available, waiting ...')

        future = self.initialize_environment_client.call_async(Goal.Request())

        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response.success:
            self.get_logger().error('initialize service call failed')

        else:
            self.goal_pose_x = response.pose_x
            self.goal_pose_y = response.pose_y
            self.get_logger().info('goal initialized at [%f, %f]' % (self.goal_pose_x, self.goal_pose_y))

    def call_task_succeed(self):
        while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for task succeed is not available, waiting ...')

        future = self.task_succeed_client.call_async(Goal.Request())

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.goal_pose_x = response.pose_x
            self.goal_pose_y = response.pose_y
            self.get_logger().info('service for task succeed finished')
        else:
            self.get_logger().error('task succeed service call failed')



    def call_task_failed(self):
        while not self.task_failed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for task failed is not available, waiting ...')

        future = self.task_failed_client.call_async(Goal.Request())
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.goal_pose_x = response.pose_x
            self.goal_pose_y = response.pose_y
            self.get_logger().info('service for task failed finished')
        else:
            self.get_logger().error('task failed service call failed')


    def scan_sub_callback(self, scan):
        self.scan_ranges = []  # clear the list
        num_of_lidar_rays = len(scan.ranges)

        for i in range(num_of_lidar_rays):
            if scan.ranges[i] == float('Inf'):
                self.scan_ranges.append(3.5)
            elif np.isnan(scan.ranges[i]):
                self.scan_ranges.append(0)
            else:
                self.scan_ranges.append(scan.ranges[i])

        self.min_obstacle_distance = min(self.scan_ranges)

    def odom_sub_callback(self, msg):
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        _, _, self.robot_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        goal_distance = math.sqrt(
            (self.goal_pose_x - self.robot_pose_x) ** 2
            + (self.goal_pose_y - self.robot_pose_y) ** 2)

        path_theta = math.atan2(
            self.goal_pose_y - self.robot_pose_y,
            self.goal_pose_x - self.robot_pose_x)

        goal_angle = path_theta - self.robot_pose_theta
        if goal_angle > math.pi:
            goal_angle -= 2 * math.pi

        elif goal_angle < -math.pi:
            goal_angle += 2 * math.pi

        self.goal_distance = goal_distance
        self.goal_angle = goal_angle

    def calculate_state(self):
        state = list()
        state.append(float(self.goal_distance))
        state.append(float(self.goal_angle))
        for var in self.scan_ranges:
            state.append(float(var))
        self.local_step += 1
        #print(self.min_obstacle_distance)
        # Succeed
        if self.goal_distance < 0.20:  # unit: m
            print("Goal Reached")
            self.succeed = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())  # robot stop
            self.local_step = 0
            self.call_task_succeed()

        # Fail
        if self.min_obstacle_distance < 0.25:  # unit: m
            print("Collision happened")
            self.fail = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())  # robot stop
            self.local_step = 0
            self.call_task_failed()

        if self.local_step == 500:
            print("Time out! :(")
            self.done = True
            self.local_step = 0
            self.call_task_failed()

        return state

    def calculate_reward(self):
        yaw_reward = 1 - 2 * math.sqrt(math.fabs(self.goal_angle / math.pi))

        distance_reward = (2 * self.init_goal_distance) / \
                          (self.init_goal_distance + self.goal_distance) - 1

        # Reward for avoiding obstacles
        if self.min_obstacle_distance < 0.25:
            obstacle_reward = -2
        else:
            obstacle_reward = 0

        reward = yaw_reward + distance_reward + obstacle_reward

        # + for succeed, - for fail
        if self.succeed:
            reward += 5
        elif self.fail:
            reward -= -10
        #print('reward:{}'.format(reward))

        return reward

    def rl_agent_interface_callback(self, request, response):
        action = request.action
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = self.angular_vel[action]
        self.cmd_vel_pub.publish(twist)
        if self.stop_cmd_vel_timer is None:
            self.stop_cmd_vel_timer = self.create_timer(1.2, self.timer_callback)
            # self.get_logger().info('Time was none')
        else:
            self.destroy_timer(self.stop_cmd_vel_timer)
            self.stop_cmd_vel_timer = self.create_timer(1.2, self.timer_callback)
            # self.get_logger().info('Timer was not none')

        response.state = self.calculate_state()
        response.reward = self.calculate_reward()
        response.done = self.done

        if self.done is True:
            self.done = False
            self.succeed = False
            self.fail = False

        if request.init is True:
            self.init_goal_distance = math.sqrt(
                (self.goal_pose_x - self.robot_pose_x) ** 2
                + (self.goal_pose_y - self.robot_pose_y) ** 2)

        return response

    def timer_callback(self):
        self.get_logger().info('Stop called')
        self.cmd_vel_pub.publish(Twist())
        self.destroy_timer(self.stop_cmd_vel_timer)

    def euler_from_quaternion(self, quat):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=sys.argv[1]):
    rclpy.init(args=args)

    rl_environment = RLEnvironment()


    while True:
        rclpy.spin_once(rl_environment)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
