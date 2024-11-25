#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64
from nav_msgs.msg import Odometry
from smartcar_msgs.msg import Status
import math

class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')
        self.steering_angle_rad = 0.0
        self.subscription = self.create_subscription(
            Status,
            '/smart_car/vehicle_status',
            self.vehicle_status_callback,
            10)
        self.publisher = self.create_publisher(Odometry, '/smart_car/wheel/odom', 10)
        self.timer = self.create_timer(0.1, self.publish_odometry)
        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.p = 0.0
        self.wheel_diameter = 0.064
        self.wheel_base = 0.257
        self.engine_speed_rpm = 0.0
        self.battery_voltage_mv = 0.0
        self.battery_current_ma = 0.0
        self.battery_percentage = 0.0

    def vehicle_status_callback(self, msg):
        self.battery_voltage_mv = msg.battery_voltage_mv
        self.battery_current_ma = msg.battery_current_ma
        self.battery_percentage = msg.battery_percentage
        self.steering_angle_rad = msg.steering_angle_rad
        self.engine_speed_rpm = msg.engine_speed_rpm

    def engine_speed_callback(self, msg):
        self.engine_speed_rpm = msg.data

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        v = (self.engine_speed_rpm * math.pi * self.wheel_diameter) / 60
        w = (v / self.wheel_base) * math.tan(self.steering_angle_rad)

        self.p += w * dt
        self.x += v * math.cos(self.p) * dt
        self.y += v * math.sin(self.p) * dt

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.p / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.p / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # Modify how we create and assign covariance matrices
        pose_covariance = [0.0] * 36
        twist_covariance = [0.0] * 36
        
        # Set diagonal elements for pose covariance
        position_variance = 1.0
        orientation_variance = 0.5
        unused_variance = 100000.0
        linear_velocity_variance = 0.1
        angular_velocity_variance = 0.1

        # Position XYZ
        pose_covariance[0] = position_variance
        pose_covariance[7] = position_variance
        pose_covariance[14] = position_variance

        # Rotation RPY
        pose_covariance[21] = orientation_variance
        pose_covariance[28] = orientation_variance
        pose_covariance[35] = orientation_variance

        # Linear XYZ velocity
        twist_covariance[0] = linear_velocity_variance
        twist_covariance[7] = linear_velocity_variance
        twist_covariance[14] = linear_velocity_variance

        # Angular RPY velocity
        twist_covariance[21] = angular_velocity_variance
        twist_covariance[28] = angular_velocity_variance
        twist_covariance[35] = angular_velocity_variance

        # Convert lists to tuples before assignment
        odom.pose.covariance = tuple(pose_covariance)
        odom.twist.covariance = tuple(twist_covariance)

        self.publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
