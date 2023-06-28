import rclpy
from rclpy.node import Node
from roboid import *
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import math


class LidarPublisher_class(Node):
    def __init__(self, b):
        self.b = b
        super().__init__("laser")
        self.publisher_ = self.create_publisher(LaserScan, "scan", 10)
        self.timer = self.create_timer(1, self.lidar_publish)

    def lidar_publish(self):
        lidar_values = self.b.lidar()

        msg = LaserScan()
        msg.header = Header()
        msg.header.frame_id = "laser"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = math.radians(0)
        msg.angle_max = math.radians(359)
        msg.angle_increment = math.radians(1)
        msg.range_min = float(0)
        msg.range_max = float(65534 / 1000)
        msg.ranges = [i / 1000 for i in lidar_values]

        self.publisher_.publish(msg)
        self.get_logger().info("Published LiDAR value: %s" % msg.ranges)


def main(args=None, beagle_instance = None):
    if beagle_instance is None:
        beagle_instance = Beagle()
    beagle_instance.start_lidar()
    beagle_instance.wait_until_lidar_ready()
    rclpy.init(args=args)
    lidar_publish = LidarPublisher_class(beagle_instance)
    try:
        rclpy.spin(lidar_publish)
    finally:
        lidar_publish.destroy_node()
        rclpy.shutdown()
        beagle_instance.dispose()


if __name__ == "__main__":
    main()
