import rclpy
from msg_interface.msg import Arithmetic
from rclpy.node import Node
from roboid import *


class KeyBoardSubscriber(Node):
    def __init__(self, b):
        self.b = b
        super().__init__("keyboard_subscriber")
        self.subscription = self.create_subscription(
            Arithmetic, "keyboard", self.callback, 10
        )
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        data = msg.argument
        # self.get_logger().info("Received KeyBoard value: {0}".format(data))

        if data == 1:
            self.b.move_forward(0.1)
            print("w")
        elif data == 2:
            self.b.move_backward(0.1)
            print("s")
        elif data == 3:
            self.b.turn_left(0.1)
            print("a")
        elif data == 4:
            self.b.turn_right(0.1)
            print("d")
        else:
            print("NO WASD")


def main(args=None, beagle_instance=None):
    if beagle_instance is None:
        beagle_instance = Beagle()
    rclpy.init(args=args)
    subscriber = KeyBoardSubscriber(beagle_instance)
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
