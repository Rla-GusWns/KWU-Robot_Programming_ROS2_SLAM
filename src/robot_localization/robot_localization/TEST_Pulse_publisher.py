import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from roboid import *
import math

import time
class PulsePublisher_class(Node):

    timer_interval = 1
    pulses_per_one_revolution = 1900 # 대략
    wheel_radius = 0.033 # 3.3 cm
    total_degree = 2 * math.pi
    scale_factor = total_degree/pulses_per_one_revolution
    def __init__(self, b):
        self.b = b
        super().__init__("encoder_publisher")
        self.publisher = self.create_publisher(Float32MultiArray, "encoder_topic", 10)
        self.left_encoder_sum = 0.0
        self.right_encoder_sum = 0.0
        self.timer = self.create_timer(self.timer_interval, self.publish_encoder_data)

    def publish_encoder_data(self):


        # 엔코더 값은 정수 -2147483648 ~ 2147483647 초기 0 의 정수 값을 가진다
        pulse_right_diff = self.b.left_encoder()
        pulse_left_diff = self.b.right_encoder()


        self.left_encoder_sum += pulse_left_diff
        self.right_encoder_sum += pulse_right_diff
        # 최종 이동거리


        # w = d theta / dts

        # v = r w
        print (self.left_encoder_sum, self.right_encoder_sum)
        # s = r * theta # 누적


        left_theta = pulse_right_diff * self.scale_factor #self.total_degree * (pulse_right_diff / self.pulses_per_one_revolution)
        right_theta = pulse_left_diff * self.scale_factor #self.total_degree * (pulse_left_diff / self.pulses_per_one_revolution)

        total_left_theta = self.left_encoder_sum * self.scale_factor
        total_right_theta = self.right_encoder_sum * self.scale_factor


        left_angular_velocity = left_theta / self.timer_interval
        right_angular_velocity = right_theta / self.timer_interval


        left_velocity_mps = left_angular_velocity * self.wheel_radius
        right_velocity_mps = right_angular_velocity * self.wheel_radius

        left_distance_m = self.wheel_radius * total_left_theta
        right_distance_m = self.wheel_radius * total_right_theta
        msg = Float32MultiArray()
        msg.data = [left_distance_m, right_distance_m, left_velocity_mps, right_velocity_mps]

        self.publisher.publish(msg)

        self.get_logger().info("Published Encoder value: " , msg)



        self.b.reset_encoder()


def main(args=None, beagle_instance = None):



    # 1886 1875

    if beagle_instance is None:
        beagle_instance = Beagle()
    #대
    rclpy.init(args=args)
    encoder_publisher = PulsePublisher_class(beagle_instance)

    try:
        rclpy.spin(encoder_publisher)
    except KeyboardInterrupt as e:
        print (e)
    finally:
        beagle_instance.dispose()
        encoder_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
