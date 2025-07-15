import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import ObstacleDistance
import numpy as np

class ScanToMavlinkNode(Node):
    def __init__(self):
        super().__init__('scan_to_mavlink_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(ObstacleDistance, '/mavros/obstacle/send', 10)

    def scan_callback(self, msg):
        dist_msg = ObstacleDistance()
        dist_msg.header = msg.header
        dist_msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        dist_msg.min_distance = int(msg.range_min * 100)
        dist_msg.max_distance = int(msg.range_max * 100)
        deg100 = int((msg.angle_increment * 180.0 / 3.14159) * 100)
        dist_msg.increment = min(deg100, 255)
        ranges = np.clip(np.array(msg.ranges), msg.range_min, msg.range_max)
        distances_cm = (ranges * 100).astype(np.uint16)
        if len(distances_cm) > 72:
            distances_cm = distances_cm[::len(distances_cm)//72][:72]
        dist_msg.distances = distances_cm.tolist()
        dist_msg.angle_offset = 0
        dist_msg.frame = 0
        self.publisher.publish(dist_msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = ScanToMavlinkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()