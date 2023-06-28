import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")
        self.subscription = self.create_subscription(
            LaserScan, "lidar", self.lidar_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def lidar_callback(self, msg):
        # 받은 LiDAR 데이터 처리 로직을 구현합니다.
        # 예시로 받은 데이터의 첫 번째 range 값을 출력하는 부분을 작성합니다.
        if len(msg.ranges) > 0:
            first_range = msg.ranges[0]
            self.get_logger().info(f"Received LiDAR range: {first_range}")


def main(args=None):
    rclpy.init(args=args)
    subscriber = LidarSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
