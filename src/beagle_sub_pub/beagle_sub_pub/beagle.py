
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Vector3, Quaternion, Twist
from tf_transformations import quaternion_from_euler
from msg_interface.msg import Arithmetic


from roboid import *
import math
import time
import yaml

import threading

import pandas as pd

class BeagleSubPub_class(Node):

    encoder_pulses_per_one_revolution = 1910 # 대략
    wheel_radius = 0.0325 # 3.25 cm

    # sensor update and publish interval
    encoder_timer_interval = 0.1
    imu_timer_interval = 0.1
    lidar_timer_interval = 0.1



    encoder_total_degree = 2 * math.pi
    scale_factor = encoder_total_degree/encoder_pulses_per_one_revolution


 #   count_lidar_timer = lidar_timer_interval//timer_interval
 #   current_lidar_count = 0


    def __init__(self, b, config_file):
        super().__init__("beagle_sensors_motors")
        self.left_encoder_sum = 0.0
        self.right_encoder_sum = 0.0
        self.b = b

        self.accel = [[0 for _ in range(4)] for _ in range(16)]
        self.gyro = [[0 for _ in range(4)] for _ in range(16)]
        self.b.listen_accelerometer(self.accel_update) # accel listener 등록
        self.b.listen_gyroscope(self.gyro_update) # gyro listener 등록

        # Calibrate
        self.error_acc_x = 0.0
        self.error_acc_y = 0.0
        self.error_acc_z = 0.0

        self.error_gyr_x = 0.0
        self.error_gyr_y = 0.0
        self.error_gyr_z = 0.0


        # 실제 정확한 엔코더 time interval을 위해서
        self.encoder_prev_time = None

        #gyro 와 가속도의 평균값 같은 timestamp 에 여러개의 가속도를 찍어서 정확도를 높인다.

        # lidar
        self.lidar_publisher = self.create_publisher(LaserScan, "scan", 10)
        self.timer1 = self.create_timer(self.lidar_timer_interval, self.publish_lidar_data)

        self.frame_id = config_file["frame_id"]
        self.angle_min = config_file["angle_min"]
        self.angle_max = config_file["angle_max"]
        self.angle_increment = config_file["angle_increment"]
        self.range_min = config_file["range_min"]
        self.range_max = config_file["range_max"]

        # imu
        self.imu_publisher = self.create_publisher(Imu, "imu_topic", 10)
        self.timer2 = self.create_timer(self.imu_timer_interval, self.publish_imu_data)
        
        # encoder 반드시 모터보다 엔코더가 먼저 실행 되어야 한다.
        self.encoder_publisher = self.create_publisher(Float32MultiArray, "encoder_topic", 10)
        self.timer3 = self.create_timer(self.encoder_timer_interval, self.publish_encoder_data)


        self.motor_subscriber = self.create_subscription(Float32MultiArray, "motor_topic", self.motor_callback, 10)
        #self.twist_subscriber = self.create_subscription(Twist, "cmd_vel", self.teleop_callback, 10)


	# keyboard
        self.subscription = self.create_subscription(Arithmetic, "keyboard", self.keyboard_callback, 10)
        



        
    def accel_update(self, index, timestamp, x, y, z):
        self.accel[index] = [x, y, z, timestamp]
    def gyro_update(self, index, timestamp, x, y, z):
        self.gyro[index] = [x, y, z, timestamp]

    def motor_callback(self, msgs):
        motor_value = msgs.data

        left_motor_speed = motor_value[0]
        right_motor_speed = motor_value[1]

        self.b.wheels(left_motor_speed, right_motor_speed)
        print ('motor_subscribtion')
    def publish_lidar_data(self):
        lidar_values = self.b.lidar()

        msg = LaserScan()
        msg.header = Header()
        msg.header.frame_id = self.frame_id

        msg.angle_min = math.radians(self.angle_min)
        msg.angle_max = math.radians(self.angle_max)
        msg.angle_increment = math.radians(self.angle_increment)
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        msg.ranges = [i / 1000 for i in lidar_values]


        self.get_logger().info("Published LiDAR value: {0}".format(msg.ranges[50])) 
        msg.header.stamp = self.get_clock().now().to_msg() # 항상 publish 전에 붙어있어야 된다. 그 사이에 연산이 있으면 time stamp 차이가 음수가 될 수 있음
        self.lidar_publisher.publish(msg)

    def publish_imu_data(self):
        # 가속도

        df_accel = pd.DataFrame(self.accel)
        df_gyro = pd.DataFrame(self.gyro)

        df_accel.dropna()
        df_gyro.dropna()

        a_x = df_accel[0].mean()
        a_y = df_accel[1].mean()
        a_z = df_accel[2].mean()
        a_t = df_accel[3].mean()

        g_x = df_gyro[0].mean()
        g_y = df_gyro[1].mean()
        g_z = df_gyro[2].mean()
        g_t = df_gyro[3].mean()




        msg = Imu()
        msg.header = Header()
        msg.header.frame_id = "imu"


        acceleration = Vector3()
        acceleration.x = (a_x + self.error_acc_x) * 9.81
        acceleration.y = (a_y + self.error_acc_y) * 9.81
        acceleration.z = (a_z + self.error_acc_z) * 9.81
        msg.linear_acceleration = acceleration


        angular = Vector3()
        angular.x = math.radians(g_x + self.error_gyr_x)
        angular.y = math.radians(g_y + self.error_gyr_y)
        angular.z = math.radians(g_z + self.error_gyr_z)
        msg.angular_velocity = angular

        #print (acceleration.x, acceleration.y, acceleration.z)
        orientation = Quaternion()
        ox, oy, oz, ow = quaternion_from_euler(angular.x, angular.y, angular.z)

        orientation.x = ox
        orientation.y = oy
        orientation.z = oz
        orientation.w = ow
        msg.orientation = orientation


        self.get_logger().info("Imu published value : %s %s" % (msg.angular_velocity, msg.linear_acceleration))
        msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_publisher.publish(msg)




    def publish_encoder_data(self):


        if self.encoder_prev_time is not None:
            # 엔코더 값은 정수 -2147483648 ~ 2147483647 초기 0 의 정수 값을 가진다
            pulse_left_diff = self.b.left_encoder()
            pulse_right_diff = self.b.right_encoder()
            encoder_actual_interval = time.perf_counter() - self.encoder_prev_time

            assert(encoder_actual_interval > 0)
            self.left_encoder_sum += pulse_left_diff
            self.right_encoder_sum += pulse_right_diff
            # 최종 이동거리
            # w = d theta / dts

            # v = r w

            # s = r * theta # 누적


            left_theta = pulse_left_diff * self.scale_factor #self.total_degree * (pulse_right_diff / self.pulses_per_one_revolution)
            right_theta = pulse_right_diff * self.scale_factor #self.total_degree * (pulse_left_diff / self.pulses_per_one_revolution)

            total_left_theta = self.left_encoder_sum * self.scale_factor
            total_right_theta = self.right_encoder_sum * self.scale_factor


            left_angular_velocity = left_theta / encoder_actual_interval
            right_angular_velocity = right_theta / encoder_actual_interval


            left_velocity_mps = left_angular_velocity * self.wheel_radius
            right_velocity_mps = right_angular_velocity * self.wheel_radius

            left_distance_m = self.wheel_radius * total_left_theta
            right_distance_m = self.wheel_radius * total_right_theta
            msg = Float32MultiArray()
            msg.data = [left_distance_m, right_distance_m, left_velocity_mps, right_velocity_mps]


            self.get_logger().info('Encoder published value %s ' % str(msg.data))
            self.encoder_publisher.publish(msg)
            self.b.reset_encoder()
        self.encoder_prev_time = time.perf_counter()


    def keyboard_callback(self, msg):
        data = msg.argument
        # self.get_logger().info("Received KeyBoard value: {0}".format(data))

        if data == 1:
            self.b.move_forward(0.2, 10)
            print("w")
        elif data == 2:
            self.b.move_backward(0.2, 10)
            print("s")
        elif data == 3:
            self.b.turn_left(0.1, 5)
            print("a")
        elif data == 4:
            self.b.turn_right(0.1, 5)
            print("d")
        else:
            print("NO WASD")




def main(args=None):

    config_file = "src/beagle_sub_pub/beagle_sub_pub/lidar_config.yaml"  # Path to the YAML configuration file

    with open(config_file, "r") as file:
        config = yaml.safe_load(file)

    # 1886 1875map 값이 안받아지는 문제가 있어서 다시 업데이트
    beagle_instance = Beagle()
    beagle_instance.reset()
    beagle_instance.start_lidar()
    beagle_instance.wait_until_lidar_ready()

    try:
        rclpy.init(args=args)
    finally:
        rclpy.shutdown()
        rclpy.init(args=args)
    publisher = BeagleSubPub_class(beagle_instance, config)

    try:

        rclpy.spin(publisher)
    except KeyboardInterrupt as e:
        print (e)
    finally:
        beagle_instance.dispose()
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
