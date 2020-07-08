import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose

import os
import random
import sys


def ROS2_INFO(logger, msg):
    logger.info(msg)


def ROS2_WARN(logger, msg):
    logger.warn(msg)

def ROS2_ERROR(logger, msg):
    logger.error(msg)



class RLGazebo(Node):
    def __init__(self, stage):
        super().__init__('rl_gazebo_interface')

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

        self.init_state = False

        """*************************************************************
                Initialize publisher, clients and services
        *************************************************************"""

        # Initialize clients
        self.delete_entity_client = self.create_client(DeleteEntity, 'delete_entity')
        self.spawn_entity_client = self.create_client(SpawnEntity, 'spawn_entity')
        self.reset_simulation_client = self.create_client(Empty, 'reset_simulation')

    def open_entity(self):
        entity_dir_path = os.path.dirname(os.path.realpath(__file__))
        entity_dir_path = entity_dir_path.replace(
            'turtlebot3_RL/turtlebot3_RL/turtlebot3_dqn_agent',
            '/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_dqn_world/goal_box'
        )
        entity_path = os.path.join(entity_dir_path, 'model.sdf')
        self.entity = open(entity_path, 'r').read()

    def reset_simulation(self):
        reset_req = Empty.Request()

        # check connection to the service server
        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            ROS2_WARN(self.get_logger(), 'service for reset_simulation is not available, waiting ...')

        future = self.reset_simulation_client.call_async(reset_req)

        # do not leave till the task is done
        rclpy.spin_until_future_complete(self, future)

    def delete_entity(self):
        delete_req = DeleteEntity.Request().name
        delete_req.name = self.entity_name

        # check connection to the service server
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            ROS2_WARN(self.get_logger(), 'service for delete_entity is not available, waiting ...')

        future = self.delete_entity_client.call_async(delete_req)

        # do not leave till the task is done
        rclpy.spin_until_future_complete(self, future)

        if not future.result().success:
            ROS2_ERROR(self.get_logger(), 'delete service call failed')
        else:
            ROS2_INFO(self.get_logger(), 'delete service call succeed')

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
            ROS2_WARN(self.get_logger(), 'service for spawn_entity is not available, waiting ...')

        future = self.spawn_entity_client.call_async(spawn_req)

        # check the task completion
        rclpy.spin_until_future_complete(self, future)

        if not future.result().success:
            ROS2_ERROR(self.get_logger(), 'spawn service call failed')
        else:
            ROS2_INFO(self.get_logger(), 'delete service call succeed')

def main():
    print('Hi from turtlebot3_RL.')


if __name__ == '__main__':
    main()
