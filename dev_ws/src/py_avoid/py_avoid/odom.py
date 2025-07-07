#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from zenmav.core import Zenmav
import numpy as np

from tf_transformations import (
    quaternion_multiply, quaternion_about_axis
)

# constant rotation:  -90° about +Z  (X→Y , Y→–X)
ROT_Z_NEG90 = quaternion_about_axis(-np.pi/2, (0, 0, 1))

class LaserOdomSubscriber(Node):

    def __init__(self):
        super().__init__('laser_odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/state_estimation',
            self.odom_callback,
            10)
        self.get_logger().info('Subscribed to /laser_odometry')
        #nav = Zenmav(ip = '127.0.0.1:14551')

        self.publisher = self.create_publisher(Odometry, '/mavros/odometry/out', 10)
        pub_Freq = 15 
        self.pub_timer = self.create_timer(1/pub_Freq, self.publish_odometry)
        self.pose = None

    def publish_odometry(self):
        if self.pose is not None:
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            
            # Fill in the position
            msg.pose.pose.position.x = self.py
            msg.pose.pose.position.y =  -self.px
            msg.pose.pose.position.z = self.pz
            
            # Fill in the orientation (quaternion)
            q_in = np.array([self.ox, self.oy, self.oz, self.ow])

            q_ros = quaternion_multiply(q_in, ROT_Z_NEG90)


            msg.pose.pose.orientation.x = q_ros[0]
            msg.pose.pose.orientation.y = q_ros[1]
            msg.pose.pose.orientation.z = q_ros[2]
            msg.pose.pose.orientation.w = q_ros[3]
            
            # Fill in the linear velocity
            msg.twist.twist.linear.x = self.lvy
            msg.twist.twist.linear.y = - self.lvx
            msg.twist.twist.linear.z = self.lvz

            sig_pos_xy  = 0.03          # m
            sig_pos_z   = 0.03
            sig_ang_rp  = float(np.deg2rad(5))
            sig_ang_yaw = float(np.deg2rad(3))

            pose_cov = [
                float(sig_pos_xy**2), 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, float(sig_pos_xy**2), 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, float(sig_pos_z**2), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, float(sig_ang_rp**2), 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, float(sig_ang_rp**2), 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, float(sig_ang_yaw**2)
            ]

            msg.pose.covariance = pose_cov

            sig_speed_xy  = 0.05    # metres
            sig_speed_z   = 0.05
            sig_speed_ang_rp  = 0.1
            sig_speed_ang_yaw = 0.1

            msg.twist.covariance = [
                float(sig_speed_xy**2), 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, float(sig_speed_xy**2), 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, float(sig_speed_z**2), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, float(sig_speed_ang_rp**2), 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, float(sig_speed_ang_rp**2), 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, float(sig_speed_ang_yaw**2)
            ]

            msg.twist.twist.angular.x = float('nan')
            msg.twist.twist.angular.y = float('nan')
            msg.twist.twist.angular.z = float('nan')

            self.publisher.publish(msg)
            #self.get_logger().info(f'PUBLISHED')

    def odom_callback(self, msg: Odometry):
        stamp = msg.header.stamp              # builtin_interfaces/Time
        t_sec, t_nsec = stamp.sec, stamp.nanosec
        parent = msg.header.frame_id
        #self.get_logger().info(f'[{t_sec}.{t_nsec:09}] frame={parent}')
        child = msg.child_frame_id
        #self.get_logger().info(f'child_frame_id={child}')
        # Position
        self.pose = msg.pose.pose.position
        self.px = self.pose.x
        self.py = self.pose.y
        self.pz = self.pose.z
        self.get_logger().info(f'Position → x: {self.px:.3f}, y: {self.py:.3f}, z: {self.pz:.3f}')
        #self.pose_cov = np.array(msg.pose.covariance).reshape(6, 6)
        #self.get_logger().info(f'Pose Covariance:\n{self.pose_cov}')
        # Orientation (quaternion)
        self.ow = msg.pose.pose.orientation.w
        self.ox = msg.pose.pose.orientation.x
        self.oy = msg.pose.pose.orientation.y
        self.oz = msg.pose.pose.orientation.z
        #self.get_logger().info(f'Orientation → x: {ox:.3f}, y: {oy:.3f}, z: {oz:.3f}, w: {ow:.3f}')

        # Linear velocity
        self.lvx = msg.twist.twist.linear.x
        self.lvy = msg.twist.twist.linear.y
        self.lvz = msg.twist.twist.linear.z
        #self.get_logger().info(f'Linear Vel → x: {lvx:.3f}, y: {lvy:.3f}, z: {lvz:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = LaserOdomSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
