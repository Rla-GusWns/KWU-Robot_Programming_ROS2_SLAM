import rclpy
from rclpy.node import Node
from roboid import *
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


class lidar(Node):
    def __init__(self, b):
        self.b = b
        super().__init__("lidar")
        self.publisher_ = self.create_publisher(LaserScan, "/scan", 10)
        self.timer = self.create_timer(1, self.lidar_publish)

    def lidar_publish(self):
        lidar_values = self.b.lidar()
        self.b._draw_lidar_chart

        msg = LaserScan()
        msg.header = Header()
        msg.header.frame_id = "laser"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.range_min = float(0)
        msg.range_max = float(65534 / 1000)
        msg.ranges = [i / 1000 for i in lidar_values]

        self.publisher_.publish(msg)
        self.get_logger().info("Published LiDAR value: %s" % msg.ranges)


def main(args=None):
    b = Beagle()
    b.start_lidar()
    b.wait_until_lidar_ready()
    rclpy.init(args=args)
    lidar_publish = lidar(b)
    rclpy.spin(lidar_publish)
    lidar_publish.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
