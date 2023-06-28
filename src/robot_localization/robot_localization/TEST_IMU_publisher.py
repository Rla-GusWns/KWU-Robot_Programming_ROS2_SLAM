import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from roboid import *
from std_msgs.msg import Header
import math
from tf_transformations import quaternion_from_euler
class IMUPublisher_class(Node):
    def __init__(self, b):
        self.b = b
        super().__init__("IMU_publisher")
        self.publisher = self.create_publisher(Imu, "imu_topic", 10)
        self.timer = self.create_timer(1.0, self.publish_imu)

    def publish_imu(self):
        index, x, y, z = self.b.accelerometer()  # 요거는 단위가 g임. 하지만 Acc는 m/s^2을 사용
        index, x1, y1, z1 = self.b.gyroscope()

        msg = Imu()
        msg.header = Header()
        msg.header.frame_id = "imu"
        msg.header.stamp = self.get_clock().now().to_msg()

        acceleration = Vector3()
        acceleration.x = x * 9.81
        acceleration.y = y * 9.81
        acceleration.z = z * 9.81
        msg.linear_acceleration = acceleration


        angular = Vector3()
        angular.x = math.radians(x1)
        angular.y = math.radians(y1)
        angular.z = math.radians(z1)
        msg.angular_velocity = angular


        orientation = Quaternion()
        ox, oy, oz, ow = quaternion_from_euler(angular.x, angular.y, angular.z)

        orientation.x = ox
        orientation.y = oy
        orientation.z = oz
        orientation.w = ow
        msg.orientation = orientation

        self.publisher.publish(msg)

        self.get_logger().info("Published value: %s" % msg)


def main(args=None, beagle_instance = None):

    if (beagle_instance is None):
        beagle_instance = Beagle()
    rclpy.init(args=args)
    imu_publisher = IMUPublisher_class(beagle_instance)
    try:
        rclpy.spin(imu_publisher)
    finally:
        beagle_instance.dispose()
        imu_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
