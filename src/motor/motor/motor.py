import rclpy
from rclpy.node import Node
from roboid import *
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import pandas as pd
#import keyboard # 루트 계정 필요 루트로 돌리면 다른 라이브러리 못씀
class motor(Node):
    def __init__(self):
        super().__init__("motor")
        self.x_r = 0
        self.y_r = 0
        self.z_r = 0
        self.subscriber1 = self.create_subscription(LaserScan, "scan", self.lidar_callback, 10)
        #self.subscriber2 = self.create_subscription(Odometry, "odom", self.odometry_callback, 10)
        self.publisher_ = self.create_publisher(Float32MultiArray, "motor_topic", 10)
        #self.t1 = threading.Thread(target=self.input_parameter)
        #self.t1.start()
        self.input_parameter()

        #self.timer = self.create_timer(1, self.motor_publish)

    # def odometry_callback(self, msg):
    #     self.x_r, self.y_r, self.z_r = euler_from_quaternion(msg.pose.pose.orientation)
        
    def lidar_callback(self, msg):

        max_motor_speed = 20
        sensor_start_distance = 0.08
        
        default_motor_speed = 20
        list_msg_ranges = list(msg.ranges)
        minimum_range = min(list_msg_ranges)

        allow_min_theta_error = 1
        allow_min_r_error = 0.1 # forward v
        min_lidar_value = 0.08

        front = 0
        left = front + 90
        back = left + 90
        right = back + 90


        front_lidar_mean = (pd.Series(list_msg_ranges[0:45]).mean() + pd.Series(list_msg_ranges[314:359]).mean())/2.0
        left_lidar_mean = pd.Series(list_msg_ranges[0:179]).mean()
        right_lidar_mean = pd.Series(list_msg_ranges[180:359]).mean()

        diff = left_lidar_mean - right_lidar_mean

        if (front_lidar_mean < min_lidar_value): # 너무 가까우면 왼쪽과 오른쪽 중 더 먼 곳으로 이동
            if (left_lidar_mean > right_lidar_mean):
                self.motor_publish(20, 0)
            elif (left_lidar_mean == right_lidar_mean):
                self.motor_publish(20, -20)
            else:
                self.motor_publish(0, 20)
        elif (diff > 0.5): #왼쪽이 훨씬 먼 경우
            self.motor_publish(10, 5)
        elif (diff > 0.25): #왼쪽이 약간 더 먼경우
            self.motor_publish(7, 5)
        elif (diff > 0.2):
            self.motor_publish(6, 5)
        elif (abs(diff) < 0.05):
            self.motor_publish(5, 5)
        elif (diff < -0.2): # 오른쪽이 더 큰 경우
            self.motor_publish(5, 6)
        elif (diff < -0.25):
            self.motor_publish(5, 7)
        elif (diff < -0.5):
            self.motor_publish(5, 10)



        # r_scaler = max_motor_speed * min_lidar_value / sensor_start_distance

        # target_vector = [[0, 0] for _ in range(360)]
        
        # if (minimum_range <= sensor_start_distance): # 원 안으로 들어왔을때
        #     for i in range(360):
        #         if msg.ranges[i] <= sensor_start_distance: # 원안에 들어와 있는게 어떤 것이 있는지 확인

        #             if msg.ranges[i] <= 0:
        #                 msg.ranges[i] = min_lidar_value

        #             r = r_scaler*(sensor_start_distance/msg.ranges[i])
        #             theta = 0

        #             # 목표 세타는 장애물의 반대편이다.
        #             if (i >= 180):
        #                 theta = i - 180
        #             else:
        #                 theta = i + 180

        #             # 계산의 편의를 위해 x, y로 변환


        #             #print (r, theta)
        #             x = r * math.cos(math.radians(theta))
        #             y = r * math.sin(math.radians(theta))
        #             target_vector[i][0] = x
        #             target_vector[i][1] = y
        #              #print (x, y)
        #     df = pd.DataFrame(target_vector)
        #     #왼쪽을 0도라고 생각하고 있다.
        #     #print (df[df[0] > 0])
        #     target_x = df[0].mean()
        #     target_y = df[1].mean()

        #     target_r = math.sqrt(target_x*target_x + target_y*target_y)
        #     target_theta = math.degrees(math.atan2(target_y, target_x)) # 0~180, -180 출력
            
        #     print ("target_r, target_theta", target_r, target_theta)
        #     print ("target_x target_y ", target_x, target_y)
        #     if (abs(target_theta) <= allow_min_theta_error or target_r <= allow_min_r_error):
        #         self.motor_publish(default_motor_speed, default_motor_speed)
        #         # left_mean = sum(list_msg_ranges[0:179])/180
        #         # right_mean = sum(list_msg_ranges[180:359])/180
        #         # minus = left_mean - right_mean
        #         # if (minus > 0):
        #         #     self.motor_publish(10, 0)
        #         # elif (minus == 0):
        #         #     self.motor_publish (10, 10)
        #         # else:
        #         #     self.motor_publish(0, 10)
        #         #self.motor_publish(default_motor_speed * (left_mean/right_mean), default_motor_speed * (right_mean/left_mean))
        #     else:
        #         # 각도 차이가 클 수록 많이 돌 수 있게

        #         self.motor_publish(-default_motor_speed*(target_theta/180), default_motor_speed*(target_theta/180))
        # else:
        #     left_mean = sum(list_msg_ranges[0:179])/180
        #     right_mean = sum(list_msg_ranges[180:359])/180
        #     diff = left_mean - right_mean

        #     magnitude = 2
        #     if (diff < 0):
        #         self.motor_publish(default_motor_speed - default_motor_speed/magnitude, default_motor_speed)
        #     elif (diff == 0):
        #         self.motor_publish(default_motor_speed, default_motor_speed)
        #     else:
        #         self.motor_publish(default_motor_speed, default_motor_speed - default_motor_speed/magnitude)



        # minimum_range_index = list_msg_ranges.index(minimum_range)

        # if (minimum_range_index > 180):
        #     minimum_range_degree = minimum_range_index - 360
        # else:
        #     minimum_range_degree = minimum_range_index

        
        # left_motor =  max_motor_speed * (minimum_range_degree / 360) + default_motor_speed * math.sin(abs(minimum_range_degree/2))
        # right_motor = - max_motor_speed * (minimum_range_degree / 360) + default_motor_speed * math.sin(abs(minimum_range_degree/2))


        # print (left_motor, right_motor)
        # self.motor_publish(left_motor, right_motor)
        
        # front = 0
        # left = front + 90
        # back = left + 90
        # right = back + 90
        # print (msg.ranges[front], msg.ranges[left], msg.ranges[back], msg.ranges[right])

        # motor_speed = 10
        # distance_filter = 0.2 # map 사이즈가 폭이 16cm 라서

        # if (msg.ranges[front] < distance_filter and msg.ranges[left] < distance_filter and msg.ranges[right]< distance_filter): #모두 막혀있는 경우 뒤로 회전
        #     if (msg.ranges[left] > msg.ranges[right]):
        #         self.motor_publish(-motor_speed, motor_speed)
        #     else:
        #         self.motor_publish(motor_speed, motor_speed)
        # elif (msg.ranges[front] < distance_filter):
        #     #왼쪽 또는 오른쪽을 비교 한뒤 더 긴 곳으로 간다.
        #     if (msg.ranges[left] > msg.ranges[right]):
        #         self.motor_publish(0, motor_speed)
        #     else:
        #         self.motor_publish(motor_speed, 0)
        # elif (msg.ranges[left] < distance_filter): # 내 왼쪽에 장애물이 있는 경우
        #     self.motor_publish(motor_speed, 0) #오른쪽으로 튼다.
        # elif (msg.ranges[right] < distance_filter):#오른쪽에 있으면 왼쪽으로 꺾는다.
        #     self.motor_publish(0, motor_speed)
        # elif (msg.ranges[back] < distance_filter):# 뒤쪽에 있으면 조금더 빨리 간다.
        #     self.motor_publish(motor_speed+5, motor_speed+5)

        # else: # 그냥 괜찮은 경우
        #     self.motor_publish(motor_speed, motor_speed)
        # if (minimum_ranges < 0.5): #50 cm 보다 작다면
        #     minimum_index = list_msg_ranges.index(minimum_ranges)
            

    def motor_publish(self, left, right):
        msg = Float32MultiArray()
        msg.data = [float(left), float(right)]

        self.publisher_.publish(msg)
    def motor_stop(self):
        self.motor_publish(0, 0)

def main(args=None):
    try:
        rclpy.init(args=args)
    finally:
        rclpy.shutdown()
        rclpy.init(args=args)
    motor_publish = motor()
    try:
        rclpy.spin(motor_publish)
    finally:
        motor_publish.motor_stop()
        rclpy.shutdown()
        motor_publish.destroy_node()


if __name__ == "__main__":
    main()
