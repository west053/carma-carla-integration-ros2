#!/usr/bin/env python3
# Copyright 2025 Will Varner, UGA MSC Lab (for ROS 2 migration of this file)
# Original file Copyright (c) 2020 Intel Corporation (MIT License)
# Core data processing logic derived from original carla-ros-bridge actor.py
# (Copyright (c) 2018-2019 Intel Corporation, MIT License)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
ROS 2 Node to publish odometry information, by migrating the original
CARLA ROS 1 bridge's OdometrySensor class functionality.
This node assumes CARLA server (e.g., 0.10.0+) is running and
that it will operate passively with CARLA's native ROS 2 tick if CARLA
is launched with the --ros2 flag.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import carla
import math 
import traceback

from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist # For type hinting

# Assuming transforms.py is in the same Python package directory
# This 'transforms' module should be the one based on carla_common.transforms
from . import transforms

class OdometrySensor(Node): # Changed inheritance from PseudoActor to rclpy.node.Node
    """
    ROS 2 Node version of the original OdometrySensor.
    Publishes odometry for a specified ego vehicle.
    The original 'parent' and 'node' arguments to __init__ are replaced
    by internal CARLA connection and ROS 2 node functionalities.
    """
    def __init__(self):
        # The original __init__ took uid, name, parent, node.
        # 'name' for node name, 'parent' for role_name/ego_vehicle, 'node' for ROS functionalities
        # These will now be handled by ROS 2 parameters and self.
        # The original 'uid' and 'name' for the PseudoActor are not directly used here,
        # node name and role_name parameter serve similar identification purposes.
        super().__init__('odometry_sensor_migrated_node') # ROS 2 Node name

        # Declare ROS 2 Parameters
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('role_name', 'hero') # Used to identify the ego vehicle
        self.declare_parameter('update_frequency', 20.0)
        self.declare_parameter('carla_timeout', 10.0)
        self.declare_parameter('world_frame_id', 'map') 
        self.declare_parameter('child_frame_id_prefix', '') 

        # Get Parameters
        self.host = self.get_parameter('carla_host').get_parameter_value().string_value
        self.port = self.get_parameter('carla_port').get_parameter_value().integer_value
        self.role_name = self.get_parameter('role_name').get_parameter_value().string_value
        self.update_frequency = self.get_parameter('update_frequency').get_parameter_value().double_value
        self.carla_timeout = self.get_parameter('carla_timeout').get_parameter_value().double_value
        self.world_frame_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        child_frame_id_prefix = self.get_parameter('child_frame_id_prefix').get_parameter_value().string_value
        # child_frame_id was self.parent.get_prefix() in ROS 1 odom_sensor.py
        self.child_frame_id = child_frame_id_prefix + self.role_name if child_frame_id_prefix else self.role_name

        self.client: carla.Client | None = None
        self.world: carla.World | None = None
        self.ego_vehicle: carla.Vehicle | None = None # This node now finds its own ego vehicle

        # ROS 2 Publisher
        # Original: self.odometry_publisher = node.new_publisher(Odometry, self.get_topic_prefix(), qos_profile=10)
        qos = QoSProfile(
            depth=10, 
            reliability=ReliabilityPolicy.RELIABLE, 
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE 
        )
        # Topic name construction similar to original bridge's get_topic_prefix()
        # which often used parent.get_topic_prefix() or derived from parent role_name.
        topic_name = f'/carla/{self.role_name}/odometry' 
        self.odometry_publisher = self.create_publisher(Odometry, topic_name, qos)

        self._connect_to_carla() # Internal method to connect

        if self.world:
            self.timer = self.create_timer(1.0 / self.update_frequency, self.update) # Timer calls original 'update' method name
            self.get_logger().info(
                f"OdometrySensor (ROS 2 Migrated) for '{self.role_name}' initialized. Publishing to {topic_name} at {self.update_frequency} Hz."
            )
            self.get_logger().info(f"Odometry Frame ID: '{self.world_frame_id}', Child Frame ID: '{self.child_frame_id}'")

        else:
            self.get_logger().error("CARLA connection failed during init. Node will not publish odometry.")

    def _connect_to_carla(self):
        """Establishes a passive connection to CARLA. Does NOT change world settings."""
        self.get_logger().info(f"Attempting passive connection to CARLA at {self.host}:{self.port}...")
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(self.carla_timeout)
            self.world = self.client.get_world()
            self.world.get_snapshot() 
            self.get_logger().info("Passively connected to CARLA server.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}\n{traceback.format_exc()}")
            self.world = None
            self.client = None
            
    def _find_ego_vehicle(self) -> bool:
        """Finds the ego vehicle based on role_name."""
        if not self.world: return False
        try:
            for actor in self.world.get_actors():
                if isinstance(actor, carla.Vehicle) and actor.attributes.get('role_name') == self.role_name:
                    self.ego_vehicle = actor
                    self.get_logger().info(f"Ego vehicle '{self.role_name}' found (ID: {self.ego_vehicle.id}).")
                    return True
        except RuntimeError as e:
            self.get_logger().warn(f"RuntimeError while finding ego vehicle: {e}")
        self.ego_vehicle = None
        return False

    # These helper methods replicate the core data-processing logic from the original bridge's actor.py
    def _get_current_ros_pose(self) -> Pose | None:
        """Gets current pose from self.ego_vehicle and converts to ROS Pose."""
        if not self.ego_vehicle or not self.ego_vehicle.is_alive: 
            self.get_logger().debug("Ego vehicle not valid for pose.")
            return None
        return transforms.carla_transform_to_ros_pose(
            self.ego_vehicle.get_transform())

    def _get_current_ros_twist_rotated(self) -> Twist | None:
        """Gets current twist from self.ego_vehicle, rotates linear to child frame, and converts to ROS Twist."""
        if not self.ego_vehicle or not self.ego_vehicle.is_alive: 
            self.get_logger().debug("Ego vehicle not valid for twist.")
            return None
        return transforms.carla_velocity_to_ros_twist(
            self.ego_vehicle.get_velocity(),           # carla.Vector3D (world frame)
            self.ego_vehicle.get_angular_velocity(), # carla.Vector3D (local frame, degrees/s)
            self.ego_vehicle.get_transform().rotation  # carla.Rotation (to rotate linear velocity to child frame)
        )

    # Kept original 'update' method name, now as a timer callback
    # Original signature: update(self, frame, timestamp)
    # New signature: update(self) as frame/timestamp come from ROS 2 clock
    def update(self): 
        """
        Function (now a ROS 2 timer callback) to update and publish odometry.
        This mirrors the logic of the original OdometrySensor.update()
        and the data fetching/transformation from the 'parent' actor object in the old bridge.
        """
        if not self.world:
            self.get_logger().warn("No CARLA world, attempting to reconnect...")
            self._connect_to_carla()
            if not self.world: # If still no connection, skip this cycle
                self.get_logger().error("Reconnect to CARLA failed. Skipping odometry update cycle.")
                return
            
        if not self.ego_vehicle or not self.ego_vehicle.is_alive:
            if not self._find_ego_vehicle():
                # self.get_logger().debug(f"Ego vehicle '{self.role_name}' not found this cycle. Skipping odometry.") # Can be spammy
                return
        
        try:
            # Create header (original used self.parent.get_msg_header which took frame and timestamp)
            # In ROS 2, we get the timestamp from the node's clock. 'frame' is not directly used.
            header = Header()
            header.stamp = self.get_clock().now().to_msg() 
            header.frame_id = self.world_frame_id # From parameter

            odometry_msg = Odometry(header=header)
            # Original used self.parent.get_prefix() for child_frame_id
            odometry_msg.child_frame_id = self.child_frame_id 

            # Original used self.parent.get_current_ros_pose() and self.parent.get_current_ros_twist_rotated()
            # We now call our internal helper methods that replicate that logic.
            current_pose = self._get_current_ros_pose()
            current_twist = self._get_current_ros_twist_rotated()

            if current_pose is None or current_twist is None:
                self.get_logger().warn("Failed to get pose or twist for ego vehicle. Actor might be invalid or not found.")
                self.ego_vehicle = None # Attempt to re-find next time
                return

            odometry_msg.pose.pose = current_pose
            odometry_msg.twist.twist = current_twist
            
            # Add placeholder covariances (important for ROS 2 best practice)
            # Variances for x, y, z, R, P, Y for pose
            odometry_msg.pose.covariance[0]=0.001; odometry_msg.pose.covariance[7]=0.001; odometry_msg.pose.covariance[14]=0.001
            odometry_msg.pose.covariance[21]=0.0001; odometry_msg.pose.covariance[28]=0.0001; odometry_msg.pose.covariance[35]=0.001
            # Variances for vx, vy, vz, wx, wy, wz for twist
            odometry_msg.twist.covariance[0]=0.001; odometry_msg.twist.covariance[7]=0.001; odometry_msg.twist.covariance[14]=0.001
            odometry_msg.twist.covariance[21]=0.0001; odometry_msg.twist.covariance[28]=0.0001; odometry_msg.twist.covariance[35]=0.001

            self.odometry_publisher.publish(odometry_msg)
            # self.get_logger().debug("Odometry message published.") # Can be spammy

        except RuntimeError as e: # Catch CARLA specific errors if actor becomes invalid mid-access
            if self.ego_vehicle and not self.ego_vehicle.is_alive:
                 self.get_logger().warn(f"Ego vehicle '{self.role_name}' became invalid during update. Will re-find.")
            else: # Other runtime errors from CARLA
                 self.get_logger().error(f"RuntimeError in odometry update: {e}\n{traceback.format_exc()}")
            self.ego_vehicle = None # Reset to trigger re-finding
        except Exception as e:
            self.get_logger().error(f"Unexpected error in odometry update: {e}\n{traceback.format_exc()}")
            self.ego_vehicle = None # Reset to ensure re-finding attempt
        
    # The old destroy() method functionality for the publisher is typically handled
    # by the ROS 2 Node's garbage collection when the node is destroyed.
    # The original PseudoActor.destroy() might have done more if it managed other resources.
    # We'll rely on the standard ROS 2 destroy_node.

    # The old get_blueprint_name() static method was for the ActorFactory.
    # It's not directly used when this class is a standalone ROS 2 node,
    # but can be kept for reference or if a similar factory pattern is adopted.
    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor (from original bridge).
        Not directly used by this standalone ROS 2 node unless by a factory.
        :return: name
        """
        return "sensor.pseudo.odom"

    def destroy_node(self): # Standard ROS 2 node cleanup
        self.get_logger().info(f"Shutting down {self.get_name()}...")
        if hasattr(self, 'timer') and self.timer and not self.timer.is_canceled():
            self.timer.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OdometrySensor() # Using the original class name OdometrySensor
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info(f"{node.get_name()} shutting down (KeyboardInterrupt).")
    except Exception as e:
        if node: node.get_logger().fatal(f"Unhandled exception in {node.get_name()}: {e}\n{traceback.format_exc()}")
        else: 
            print(f"[FATAL] Unhandled exception during {OdometrySensor.__name__} node initialization: {e}")
            traceback.print_exc()
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()