import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
from turtlesim.msg import Pose

class TurtlesimController(Node):

    def __init__(self, speed:float, omega:float):
        super().__init__('turtlesim_controller')
        self.speed = speed
        self.omega = omega
        self.pose = None
        self.twist_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cb_pose,
            10)
        self.wait_for_pose()

    def cb_pose(self, msg):
            self.pose = msg

    def wait_for_pose(self):
        while self.pose is None and rclpy.ok():
            self.get_logger().info('waiting for Pose. . .')
            rclpy.spin_once(self)

    def go_prop_controller(self,dx,dy,diff,gain,vel_msg, distance):
        pos = self.is_dest_in_front(dx,dy)
        #self.get_logger().info(""+str(diff))
        while abs(diff) >=0.01:
            diff = math.sqrt((dx - self.pose.x) ** 2 + (dy - self.pose.y) ** 2)
            dest_ahead = self.is_dest_in_front(dx,dy)

            if pos != dest_ahead:
                distance *= -1
                pos = dest_ahead
                self.get_logger().info("Changed direction")

            target_speed = diff * gain
            target_speed = math.copysign(target_speed, distance)
            target_speed = min(max(target_speed, -self.speed), self.speed)
            vel_msg.linear.x = target_speed

            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self)
        return vel_msg

    def turn_prop_controller(self, target_angle, diff, gain,vel_msg):
        while abs(diff) > 0.015 and rclpy.ok():
            vel_msg.angular.z = math.radians(min(max(diff * gain, -self.omega), self.omega))
            self.twist_pub.publish(vel_msg)

            #with normalization
            current_angle = math.degrees(self.pose.theta) % 360
            diff = target_angle - current_angle

            if abs(diff) > 180:
                diff = (diff + 180) % 360 - 180

            rclpy.spin_once(self)
        return vel_msg

    def is_dest_in_front(self, dest_x, dest_y):
        angle_to_dest = math.atan2(dest_y - self.pose.y, dest_x - self.pose.x)
        angle_diff = math.degrees(abs(angle_to_dest - self.pose.theta))
        #normalize to range -180,180
        angle_diff = (angle_diff + 180) % 360 - 180

        return abs(angle_diff) < 90

    def controlled_turn(self, angle):
        self.wait_for_pose()

        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = math.radians(self.omega) if angle > 0 else -math.radians(self.omega)

        #with normalization
        current_angle = math.degrees(self.pose.theta) % 360
        target_angle = (current_angle + angle) % 360

        gain = 4

        diff = target_angle - current_angle

        if abs(diff) > 180:
            diff = (diff + 180) % 360 - 180

        vel_msg = self.turn_prop_controller(target_angle,diff,gain,vel_msg)

        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)

    def controlled_go_straight(self, distance):
        self.wait_for_pose()

        #check direction coordinates
        dx = self.pose.x + distance * math.cos(self.pose.theta)
        dy = self.pose.y + distance * math.sin(self.pose.theta)
        diff = math.sqrt((dx - self.pose.x) ** 2 + (dy - self.pose.y) ** 2)

        # Create and publish msg
        vel_msg = Twist()
        vel_msg.linear.x = self.speed if distance > 0 else -self.speed
        vel_msg.angular.z = 0.0

        gain=1.5
        vel_msg = self.go_prop_controller(dx,dy,diff,gain,vel_msg,distance)
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)

    def go_straight(self, speed, distance):
        self.wait_for_pose()

        #check direction coordinates
        dx = self.pose.x + distance * math.cos(self.pose.theta)
        dy = self.pose.y + distance * math.sin(self.pose.theta)
        diff = math.sqrt((dx - self.pose.x) ** 2 + (dy - self.pose.y) ** 2)

        # Create and publish msg
        vel_msg = Twist()
        if distance > 0:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = -speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        # Calculate time
        T = distance / speed

        # Set loop rate
        loop_rate = self.create_rate(100)

        # Publish first msg and note time when to stop
        self.twist_pub.publish(vel_msg)
        # self.get_logger().info('Turtle started.')
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        while (self.get_clock().now() < when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            # self.get_logger().info('On its way...')
            rclpy.spin_once(self)   # loop rate

        # turtle arrived, set velocity to 0
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)
        # self.get_logger().info('Arrived to destination.')


    def turn(self, omega, angle):
        self.wait_for_pose()
        # Create and publish msg for turning
        vel_msg = Twist()
        vel_msg.angular.z = omega

        # Calculate time for turning
        T_turn = angle / omega

        # Set loop rate
        loop_rate = self.create_rate(100)

        # Publish first msg and note time when to stop turning
        self.twist_pub.publish(vel_msg)
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T_turn)

        # Publish msg while the calculated time for turning is up
        while (self.get_clock().now() < when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self)   # loop rate

        # Stop turning
        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)

    def sierpinski_triangle(self, level, size):
        self.controlled_go_straight(-5)
        self.triangle(size, level)

    def triangle(self, size, level):
        self.get_logger().info('draw triangle on level:'+str(level))
        if level == 0:
            for _ in range(3):
                self.controlled_go_straight(size)
                self.controlled_turn(-120)
        else:
            self.triangle(size / 2, level - 1)
            self.controlled_go_straight(size / 2)
            self.triangle(size / 2, level - 1)
            self.controlled_turn(-120)
            self.controlled_go_straight(size / 2)
            self.controlled_turn(120)
            self.triangle(size / 2, level - 1)
            self.controlled_turn(120)
            self.controlled_go_straight(size / 2)
            self.controlled_turn(-120)


def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController(18.0,60.0)
    #tc.controlled_go_straight(2.0)
    #tc.controlled_turn(120)

    tc.sierpinski_triangle(4, 8)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
