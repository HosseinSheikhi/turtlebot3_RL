import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from std_srvs.srv import Empty
from turtlebot3_msgs.srv import Goal
from geometry_msgs.msg import Pose

import os
import random
import sys


class GazeboInterface(Node):
    def __init__(self, stage):
        super().__init__('gazebo_interface')
        """**************************************************************
                            Initialize variables
        **************************************************************"""
        # Environment stage
        self.stage = int(stage)

        # Read the 'Goal' Entity Model
        self.entity_name = 'Goal'
        self.entity = None
        self.open_entity()

        self.entity_pose_x = 0.5
        self.entity_pose_y = 0.0

        """*************************************************************
                Initialize publisher, clients and services
        *************************************************************"""

        # Initialize clients
        self.delete_entity_client = self.create_client(DeleteEntity, 'delete_entity')
        self.spawn_entity_client = self.create_client(SpawnEntity, 'spawn_entity')
        self.reset_simulation_client = self.create_client(Empty, 'reset_simulation')

        self.clbk_grp = MutuallyExclusiveCallbackGroup()
        # Initialize services
        self.initialize_env_service = self.create_service(Goal, 'initialize_env', self.initialize_env_callback, callback_group=self.clbk_grp)
        self.task_succeed_service = self.create_service(Goal, 'task_succeed', self.task_succeed_callback, callback_group=self.clbk_grp)
        self.task_failed_service = self.create_service(Goal, 'task_failed', self.task_failed_callback, callback_group=self.clbk_grp)

    def open_entity(self):
        entity_dir_path = os.path.dirname(os.path.realpath(__file__))
        entity_dir_path = entity_dir_path.replace(
            'turtlebot3_rl/turtlebot3_rl/turtlebot3_gazebo',
            '/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_dqn_world/goal_box'
        )
        entity_path = os.path.join(entity_dir_path, 'model.sdf')
        self.entity = open(entity_path, 'r').read()

    def reset_simulation(self):
        reset_req = Empty.Request()

        # check connection to the service server
        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for reset_simulation is not available, waiting ...')

        self.reset_simulation_client.call_async(reset_req)

    def delete_entity(self):
        delete_req = DeleteEntity.Request()
        delete_req.name = self.entity_name

        # check connection to the service server
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for delete_entity is not available, waiting ...')

        self.delete_entity_client.call_async(delete_req)

    def spawn_entity(self):
        entity_pose = Pose()
        entity_pose.position.x = self.entity_pose_x
        entity_pose.position.y = self.entity_pose_y

        spawn_req = SpawnEntity.Request()
        spawn_req.name = self.entity_name
        spawn_req.xml = self.entity
        spawn_req.initial_pose = entity_pose

        # check connection to the service server
        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for spawn_entity is not available, waiting ...')

        self.spawn_entity_client.call_async(spawn_req)

    def task_succeed_callback(self, request, response):
        self.delete_entity()
        self.generate_goal_pose()
        self.spawn_entity()
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        self.get_logger().info('A new goal generated')
        return response

    def task_failed_callback(self, request, response):
        self.delete_entity()
        self.reset_simulation()
        self.generate_goal_pose()
        self.spawn_entity()
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        self.get_logger().info('Environment reset')
        return response

    def initialize_env_callback(self, request, response):
        self.delete_entity()
        self.reset_simulation()
        self.spawn_entity()
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        self.get_logger().info('Environment initialized')
        return response

    def generate_goal_pose(self):
        if self.stage != 4:
            self.entity_pose_x = random.randrange(-15, 16) / 10
            self.entity_pose_y = random.randrange(-15, 16) / 10
        else:
            goal_pose_list = [[1.0, 0.0], [2.0, -1.5], [0.0, -2.0], [2.0, 2.0], [0.8, 2.0],
                              [-1.9, 1.9], [-1.9, 0.2], [-1.9, -0.5], [-2.0, -2.0], [-0.5, -1.0]]
            rand_index = random.randint(0, 10)
            self.entity_pose_x = goal_pose_list[rand_index][0]
            self.entity_pose_y = goal_pose_list[rand_index][1]


def main(args=sys.argv[1]):
    rclpy.init(args=args)
    gazebo_interface = GazeboInterface(args)
    while True:
        rclpy.spin_once(gazebo_interface,timeout_sec=0.1)
        #gazebo_interface.get_logger().info('iterate')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
