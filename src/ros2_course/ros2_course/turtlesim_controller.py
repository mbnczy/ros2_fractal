import math
import rclpy
from rclpy.node import Node


class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')


    def go_straight(self, speed, distance):
        # Implement straght motion here


def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
