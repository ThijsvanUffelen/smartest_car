#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from smartcar_msgs.msg import Status
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from rclpy.time import Time
from geometry_msgs.msg import Twist

class JointStatePublisherNode(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Create subscribers
        self.status_sub = self.create_subscription(
            Status,
            '/smart_car/vehicle_status',
            self.status_callback,
            10
        )
        
        # Add Twist subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create publishers
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Create TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Initialize variables
        self.steering_angle = 0.0
        self.vehicle_speed = 0.0
        self.prev_time = self.get_clock().now()
        self.wheel_angles = [0.0] * 6
        
        self.wheel_radius = 0.032  # From URDF
        self.wheelbase_length = 0.257  # From URDF
        self.wheelbase_width = 0.17  # From URDF
        self.steering_gear_ratio = 2.5
        
        # Create timer for publishing joint states
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz
        
        # Joint names from URDF
        self.joint_names = [
            'front_left_wheel_steer_joint',
            'front_right_wheel_steer_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint'
        ]

    def status_callback(self, msg):
        self.steering_angle = msg.steering_angle_rad * self.steering_gear_ratio
        self.vehicle_speed = msg.engine_speed_rpm * 2.0 * math.pi * self.wheel_radius / 60.0

    def cmd_vel_callback(self, msg):
        if abs(msg.angular.z) > 0.001:
            # Reduce the steering angle scaling factor
            self.steering_angle = msg.angular.z * 0.3  # Changed from 0.5 to 0.3
        else:
            self.steering_angle = 0.0
            
        # Limit the maximum steering angle to prevent instability
        max_steering_angle = 0.6  # About 34 degrees
        self.steering_angle = max(min(self.steering_angle, max_steering_angle), -max_steering_angle)
        
        # Set vehicle speed from linear velocity
        self.vehicle_speed = msg.linear.x

    def publish_joint_states(self):
        joint_state = JointState()
        
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now
        
        # Calculate wheel rotations based on vehicle speed
        distance = self.vehicle_speed * dt
        angle_increment = distance / self.wheel_radius

        # Calculate inner and outer wheel angles for steering
        if abs(self.steering_angle) > 0.001:
            turning_radius = self.wheelbase_length / math.tan(abs(self.steering_angle))
            inner_angle = math.atan(self.wheelbase_length / (turning_radius - (self.wheelbase_width / 2)))
            outer_angle = math.atan(self.wheelbase_length / (turning_radius + (self.wheelbase_width / 2)))
            
            if self.steering_angle > 0:
                steer_angles = [outer_angle, inner_angle]
            else:
                steer_angles = [-inner_angle, -outer_angle]
        else:
            steer_angles = [0.0, 0.0]

        # Update cumulative wheel angles
        for i in range(2, 6):  # Indexes 2-5 are wheel rotation joints
            self.wheel_angles[i] += angle_increment

        # Create joint state message
        joint_state.header.stamp = now.to_msg()
        joint_state.header.frame_id = 'base_link'
        joint_state.name = self.joint_names
        
        # Set positions
        joint_state.position = [
            steer_angles[0],          # front_left_steer
            steer_angles[1],          # front_right_steer
            self.wheel_angles[2],     # front_left_wheel
            self.wheel_angles[3],     # front_right_wheel
            self.wheel_angles[4],     # back_left_wheel
            self.wheel_angles[5]      # back_right_wheel
        ]
        
        # Set velocities
        wheel_angular_velocity = self.vehicle_speed / self.wheel_radius
        joint_state.velocity = [
            0.0,                    # front_left_steer
            0.0,                    # front_right_steer
            wheel_angular_velocity, # front_left_wheel
            wheel_angular_velocity, # front_right_wheel
            wheel_angular_velocity, # back_left_wheel
            wheel_angular_velocity  # back_right_wheel
        ]
        joint_state.effort = [0.0] * len(self.joint_names)
        
        # Publish joint states
        self.joint_pub.publish(joint_state)
        
        # Publish wheel transforms
        self.publish_wheel_transforms(now)

    def publish_wheel_transforms(self, now):
        # Helper function to create transform
        def create_transform(name, x, y, z, roll, pitch, yaw):
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = name
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            
            # Convert Euler angles to quaternion
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            t.transform.rotation.w = cr * cp * cy + sr * sp * sy
            t.transform.rotation.x = sr * cp * cy - cr * sp * sy
            t.transform.rotation.y = cr * sp * cy + sr * cp * sy
            t.transform.rotation.z = cr * cp * sy - sr * sp * cy
            
            return t

        # Calculate wheel positions
        half_length = self.wheelbase_length / 2
        half_width = self.wheelbase_width / 2
        wheel_z = 0.025  # wheel hub height from URDF

        # Publish transforms for all wheels
        transforms = []
        
        # Front wheels with steering
        transforms.append(create_transform('front_left_wheel_link', 
                                        half_length, half_width, wheel_z, 
                                        0, 0, self.steering_angle))
        transforms.append(create_transform('front_right_wheel_link', 
                                        half_length, -half_width, wheel_z, 
                                        0, 0, self.steering_angle))
        
        # Back wheels
        transforms.append(create_transform('back_left_wheel_link', 
                                        -half_length, half_width, wheel_z, 
                                        0, 0, 0))
        transforms.append(create_transform('back_right_wheel_link', 
                                        -half_length, -half_width, wheel_z, 
                                        0, 0, 0))
        
        # Broadcast all transforms
        for transform in transforms:
            self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
