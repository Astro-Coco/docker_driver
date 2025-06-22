#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from zenmav.core import Zenmav
import numpy as np
from tf_transformations import quaternion_multiply, quaternion_about_axis
class LaserOdomSubscriber(Node):

    def __init__(self):
        super().__init__('laser_odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/state_estimation',
            self.odom_callback,
            10)
        self.get_logger().info('Subscribed to /laser_odometry')
        nav = Zenmav(ip = '127.0.0.1:14551')

        self.publisher = self.create_publisher(Odometry, '/mavros/odometry/out', 10)
        self.pub_timer = self.create_timer(0.1, self.publish_odometry)
    
    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Fill in the position
        msg.pose.pose.position.x = self.py
        msg.pose.pose.position.y = - self.px
        msg.pose.pose.position.z = self.pz
        
        # Fill in the orientation (quaternion)
        msg.pose.pose.orientation.w = self.ow
        msg.pose.pose.orientation.x = self.ox
        msg.pose.pose.orientation.y = self.oy
        msg.pose.pose.orientation.z = self.oz

        msg.pose.covariance = np.zeros(36).tolist()  # 6x6 covariance matrix flattened to 1D list
        
        # Fill in the linear velocity
        msg.twist.twist.linear.x = self.lvy
        msg.twist.twist.linear.y = - self.lvx
        msg.twist.twist.linear.z = self.lvz
        
        # Fill in the angular velocity
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0

        

    def odom_callback(self, msg: Odometry):
        stamp = msg.header.stamp              # builtin_interfaces/Time
        t_sec, t_nsec = stamp.sec, stamp.nanosec
        parent = msg.header.frame_id
        self.get_logger().info(f'[{t_sec}.{t_nsec:09}] frame={parent}')
        child = msg.child_frame_id
        self.get_logger().info(f'child_frame_id={child}')
        # Position
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.pz = msg.pose.pose.position.z
        self.get_logger().info(f'Position → x: {px:.3f}, y: {py:.3f}, z: {pz:.3f}')
        self.pose_cov = np.array(msg.pose.covariance).reshape(6, 6)
        #self.get_logger().info(f'Pose Covariance:\n{pose_cov}')
        # Orientation (quaternion)
        self.ow = msg.pose.pose.orientation.w
        self.ox = msg.pose.pose.orientation.x
        self.oy = msg.pose.pose.orientation.y
        self.oz = msg.pose.pose.orientation.z
        self.get_logger().info(f'Orientation → x: {ox:.3f}, y: {oy:.3f}, z: {oz:.3f}, w: {ow:.3f}')

        # Linear velocity
        self.lvx = msg.twist.twist.linear.x
        self.lvy = msg.twist.twist.linear.y
        self.lvz = msg.twist.twist.linear.z
        self.get_logger().info(f'Linear Vel → x: {lvx:.3f}, y: {lvy:.3f}, z: {lvz:.3f}')
        self.twist_cov = np.array(msg.twist.covariance).reshape(6, 6)
        #self.get_logger().info(f'Twist Covariance:\n{twist_cov}')

        # Angular velocity
        avx = msg.twist.twist.angular.x
        avy = msg.twist.twist.angular.y
        avz = msg.twist.twist.angular.z
        self.get_logger().info(f'Angular Vel → x: {avx:.3f}, y: {avy:.3f}, z: {avz:.3f}')

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
