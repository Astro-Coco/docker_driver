#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from zenmav.core import Zenmav
class LaserOdomSubscriber(Node):

    def __init__(self):
        super().__init__('laser_odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/state_estimation',
            self.odom_callback,
            10)
        self.get_logger().info('Subscribed to /laser_odometry')
        print('Trying to connect to MAVLink... ?')
        nav = Zenmav(ip = '127.0.0.1:14551')
        print('CONNECTED')
        self.connected = 1
    def odom_callback(self, msg: Odometry):
        # Position
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        self.get_logger().info(f'Position → x: {px:.3f}, y: {py:.3f}, z: {pz:.3f}')

        # Orientation (quaternion)
        ow = msg.pose.pose.orientation.w
        ox = msg.pose.pose.orientation.x
        oy = msg.pose.pose.orientation.y
        oz = msg.pose.pose.orientation.z
        self.get_logger().info(f'Orientation → x: {ox:.3f}, y: {oy:.3f}, z: {oz:.3f}, w: {ow:.3f}')

        # Linear velocity
        lvx = msg.twist.twist.linear.x
        lvy = msg.twist.twist.linear.y
        lvz = msg.twist.twist.linear.z
        self.get_logger().info(f'Linear Vel → x: {lvx:.3f}, y: {lvy:.3f}, z: {lvz:.3f}')

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
