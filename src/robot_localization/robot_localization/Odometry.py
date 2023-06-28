import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import math
from std_msgs.msg import Header


class OdometryCalculator(Node):
    def __init__(self):
        self.imu_linear_distance = 0.0
        self.imu_angular_distance = 0.0
        self.encoder_angular_distance = 0.0
        self.encoder_linear_distance = 0.0
        self.prev_encoder_values = [0, 0]
        self.imu_prev_msg = None
        self.imu_average_linear_velocity = 0.0 #초기 속도는 0이라 가정
        
        self.rotate_radius = 0.05 # 회전 중심으로부터 바퀴 까지의 거리 (바퀴에 연결되어 있는 나사 부분)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        super().__init__("odometry_calculator")
        self.encoder_subscriber = self.create_subscription(Float32MultiArray, "encoder_topic", self.encoder_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, "imu_topic", self.imu_callback, 10)
        self.odometry_publisher = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Odometry")
    def time_stamp_diff(self, current_stamp, previous_stamp):
        # 현재 시간 - 이전 시간

        # 이전 방법인 각각 문자열로 변경후 float으로 바꾼 뒤 빼는 경우 소수점 손실이 일어나 current_stamp > previous_stamp 인 경우에도
        # current_stamp - previous_stamp < 0 인 경우가 발생했다.

        # 이를 해결 하기 위해 정수 부분과 소수 부분을 그대로 살려서 뺄샘을 진행하도록 한다.


        # 현재시간과 이전 시간이 각각 정수 부분과 소수 부분의 정수형 형태로 주어졌다
        
        # 소수의 연산중에서 소수를 모두 정수 형태로 바꿔서
        # 이를 뺀 다음에 다시 소수로 변환하는 방법이 있다.
        
        # 이를 위해 소수를 정수의 형태로 바꾸기 위해서는 두 개의 숫자의 소수 부분을 똑같이 맞춰줘야 한다.

        # 두 시간의 차를 계산한 정수에서 소수로 바꿀때는 10 ^ (소수 부분 길이의 최대값)으로 나눈다.


        # 현재 시간 정보 구하기
        current_stamp_sec = current_stamp.sec
        current_stamp_sec_str = str(current_stamp_sec)
        current_stamp_sec_len = len(current_stamp_sec_str)

        current_stamp_nanosec = current_stamp.nanosec
        current_stamp_nanosec_str = str(current_stamp_nanosec)
        current_stamp_nanosec_len = len(current_stamp_nanosec_str)

        #이전 시간 정보 구하기
        previous_stamp_sec = previous_stamp.sec
        previous_stamp_sec_str = str(previous_stamp_sec)
        previous_stamp_sec_len = len(previous_stamp_sec_str)

        previous_stamp_nanosec = previous_stamp.nanosec
        previous_stamp_nanosec_str = str(previous_stamp_nanosec)
        previous_stamp_nanosec_len = len(previous_stamp_nanosec_str)


        max_floating_point = max(current_stamp_nanosec_len, previous_stamp_nanosec_len)

        # integer로 변환하여 계산하기 위해서는 소수점의 자리수를 맞춰 줘야 한다. nanosec는 소수점 자리가 아니라 그냥 숫자이므로 앞에다가 최대 자리수에 맞게 0을 붙인다. 
        

        current_stamp_integer = int(current_stamp_sec_str + '0' * (max_floating_point - current_stamp_nanosec_len) + str(current_stamp_nanosec)) 
        previous_stamp_integer = int(previous_stamp_sec_str + '0' * (max_floating_point - previous_stamp_nanosec_len) + str(previous_stamp_nanosec))
        

        #print (max_floating_point ,"current_stamp_integer", current_stamp_integer , "previous_stamp_integer", previous_stamp_integer)
        difference_time = current_stamp_integer - previous_stamp_integer

        difference_time_float = difference_time/pow(10, max_floating_point) # 다시 소수점 자리에 위치


        return difference_time_float
    def encoder_callback(self, msg):
        encoder_values = msg.data
        delta_encoder_left = encoder_values[0] - self.prev_encoder_values[0]
        delta_encoder_right = encoder_values[1] - self.prev_encoder_values[1]
        self.prev_encoder_values = encoder_values

        self.encoder_linear_distance = (
            delta_encoder_left + delta_encoder_right
        ) / 2.0  # Calculate the average linear distance
        self.encoder_angular_distance = (
            delta_encoder_right - delta_encoder_left
        ) / (2.0*self.rotate_radius)  # Calculate the average angular distance
        
        self.calculate_weight_odometry()

    def imu_callback(self, msg):
        if self.imu_prev_msg is not None:

            imu_second = self.time_stamp_diff(msg.header.stamp, self.imu_prev_msg.header.stamp)
            print (msg.header.stamp.sec, msg.header.stamp.nanosec, self.imu_prev_msg.header.stamp.sec, self.imu_prev_msg.header.stamp.nanosec)
            #imu_prev_sec = self.time_stamp_to_second(self.imu_prev_msg.header.stamp)
            #imu_current_sec = self.time_stamp_to_second(msg.header.stamp)
            #imu_second = imu_current_sec - imu_prev_sec # 보내는 간격이 짧을 수록 오차가 작아진다.
            self.get_logger().info("{0} {1}".format(msg.header.stamp, self.imu_prev_msg.header.stamp))
            self.get_logger().info("{0}".format(imu_second))

            assert(imu_second > 0)
            

            # 평균 전진 이동 속도 = s0  + v0 t + (1/2) * (평균 x 축 가속도) * 시간^2 
            imu_average_linear_acceleration = (msg.linear_acceleration.x + self.imu_prev_msg.linear_acceleration.x)/2.0
            self.imu_average_linear_velocity += imu_average_linear_acceleration * (imu_second/2)

            # 평균 이동 각도 = 평균 각속도 * 시간
            self.imu_average_angular_velocity = (self.imu_prev_msg.angular_velocity.z + msg.angular_velocity.z)/2.0


            #self.imu_linear_distance = 0.0
            self.get_logger().info("IMU Sampling time {0}".format(imu_second))
            self.imu_linear_distance = self.imu_average_linear_velocity * (imu_second/2)
            self.imu_angular_distance = self.imu_average_angular_velocity * (imu_second/2)
            #print ('시간 차이 :%lf, 회전 각도 %lf, 전진 이동 거리 %lf' % (imu_second, self.imu_angular_distance, self.imu_linear_distance))
            #print ('raw accel : %lf %lf %lf' % (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z ))
            self.calculate_weight_odometry()
        self.imu_prev_msg = msg





        # if self.prev_imu_orientation is not None:
        #     delta_orientation = self.calculate_delta_orientation(
        #         imu_orientation, self.prev_imu_orientation
        #     )

            
        #     self.imu_angular_distance = delta_orientation

        #     self.calculate_weight_odometry()
        # self.prev_imu_orientation = imu_orientation

    def calculate_weight_odometry (self):
        linear_distance = 1.0 * self.encoder_linear_distance + 0.0 * self.imu_linear_distance
        #바퀴가 sleep 일어날 것을 고려해서 lidar의 변화가 없으면 linear_distance를 0으로 하는게 낫지 않을까
        #평소 바퀴 속도일때 (현재 lidar 변화율 / 평균 lidar 변화율) * linear_distance


        #imu가 더 정확하다고 가정
        angular_distance = 0.1 * self.encoder_angular_distance + 0.9 * self.imu_angular_distance

        self.get_logger().info("Encoder 추정 : 전진 이동 거리 {0}, 각도 거리 {1}".format(self.encoder_linear_distance, self.encoder_angular_distance))
        self.get_logger().info("IMU 추정 : 전진 이동 거리 {0}, 각도 거리 {1} rad".format(self.imu_linear_distance, self.imu_angular_distance))
        self.get_logger().info("통합 추정 : 전진 이동 거리 {0}, 각도 거리 {1} rad".format(linear_distance, angular_distance))

        self.calculate_odometry(linear_distance, angular_distance)
    def calculate_delta_orientation(self, current_orientation, previous_orientation):
        # Calculate the change in orientation given current and previous IMU orientations
        current_quaternion = (
            current_orientation.x,
            current_orientation.y,
            current_orientation.z,
            current_orientation.w,
        )
        previous_quaternion = (
            previous_orientation.x,
            previous_orientation.y,
            previous_orientation.z,
            previous_orientation.w,
        )

        current_euler = self.quaternion_to_euler(current_quaternion)
        previous_euler = self.quaternion_to_euler(previous_quaternion)

        delta_orientation = current_euler[2] - previous_euler[2]
        delta_orientation = math.atan2(
            math.sin(delta_orientation), math.cos(delta_orientation)
        )

        return delta_orientation

    def calculate_odometry(self, linear_distance, angular_distance):
        self.theta += angular_distance
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.x += linear_distance * math.cos(self.theta)
        self.y += linear_distance * math.sin(self.theta)

        # Publish odometry
        odometry_msg = Odometry()
        odometry_msg.header = Header()

        odometry_msg.header.frame_id = 'map'
        odometry_msg.child_frame_id = 'odom'

        # Set the position
        odometry_msg.pose.pose.position.x = self.x
        odometry_msg.pose.pose.position.y = self.y
        odometry_msg.pose.pose.position.z = 0.0

        # Set the orientation as a quaternion
        quaternion = self.euler_to_quaternion(0.0, 0.0, self.theta) #z축을 기준으로 얼마나 회전했는가
        odometry_msg.pose.pose.orientation.x = quaternion[0]
        odometry_msg.pose.pose.orientation.y = quaternion[1]
        odometry_msg.pose.pose.orientation.z = quaternion[2]
        odometry_msg.pose.pose.orientation.w = quaternion[3]

        # Set the linear velocity
        odometry_msg.twist.twist.linear.x = linear_distance
        odometry_msg.twist.twist.linear.y = 0.0
        odometry_msg.twist.twist.linear.z = 0.0

        # Set the angular velocity
        odometry_msg.twist.twist.angular.x = 0.0
        odometry_msg.twist.twist.angular.y = 0.0
        odometry_msg.twist.twist.angular.z = angular_distance

        # Publish the odometry message
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        self.odometry_publisher.publish(odometry_msg)

        # Publish the transform between "odom" and "base_link" frames
        transform = TransformStamped()

        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(transform)

    def euler_to_quaternion(self, roll, pitch, yaw):
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        return quaternion

    def quaternion_to_euler(self, quaternion):
        euler = euler_from_quaternion(quaternion)
        return euler


def main(args=None):
    try:
        rclpy.init(args=args)
    finally:
        rclpy.shutdown()
        rclpy.init(args=args)

    
    odometry_calculator = OdometryCalculator()
    try:
        rclpy.spin(odometry_calculator)
    finally:
        odometry_calculator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
