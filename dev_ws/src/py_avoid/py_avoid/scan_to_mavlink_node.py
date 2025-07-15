import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        self.sub = self.create_subscription(LaserScan, '/scan',
                                            self.pub_cb, 10)
        self.pub = self.create_publisher(LaserScan,
                                         '/mavros/obstacle/send', 10)
        self.get_logger().info('Scan Relay Node has been started.')
    def pub_cb(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ScanRelay())
