#!/usr/bin/env python3

from msg_interface.msg import Arithmetic
import rclpy
from rclpy.node import Node

# 어차피 ros2에서 구동할꺼 바로 termios랑 tty import
import sys, select, termios, tty

Notification = """
Control Beagle
---------------------------
Moving around:
        w
   a    s    d
        x

w: 1초간 앞으로 전진
s: 1초간 뒤로 후진
a: 왼쪽으로 1초간 rotate
d: 오른쪽으로 1초간 rotate
x: end

CTRL-C to quit
"""

# 계획은 이거 어차피 앞으로 가고 뒤로 가고 다 구현이 되어있는데 angular, linear로 할 필요가 없다는 거지
# 그래서 w a s d 입력 되었을 때 정수 a 에 1 2 3 4 넣어서 publish 예정
# subscriber에서 a 의 값에 따라서 전진 후진 rotate 할 예정


class TeleopNode(Node):
    def __init__(self):
        super().__init__("beagle_teleop")
        self.publisher_ = self.create_publisher(Arithmetic, "keyboard", 10)

    # 키보드 입력을 확인하여 키보드 입력이 있을 때 보냄
    def getKey(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        print(Notification)  # 설명부분 출력하고
        a = 0
        while True:
            key = self.getKey()  # 입력된 값을 받아서 비교
            if key == "w":
                a = 1
            elif key == "s":
                a = 2
            elif key == "a":
                a = 3
            elif key == "d":
                a = 4
            elif key == "x":
                break
            elif key == "\x03":
                break
            else:
                a = 0

            msg = Arithmetic()
            msg.stamp = self.get_clock().now().to_msg()
            msg.argument = a
            self.publisher_.publish(msg)
            self.get_logger().info("Publish Keyboard value: {0}".format(msg.argument))


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    teleop_node.run()
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
