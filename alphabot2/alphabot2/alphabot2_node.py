import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

class MovementTester(Node):

    def __init__(self):
        super().__init__('movement_tester')
        self.pub = self.create_publisher(TwistStamped, '/diff_controller_alphabot2/cmd_vel', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.state = 0
        self.get_logger().info('MovementTester node has been started.')
        #self.move(0.5, 0.0)

    def move(self, x, th):
        # x: <0.02, 0.85>
        # th: z toho vznikne dvojnasobok v ros2_control (cize z 5 tu bude 10 v command)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'telo'  # Frame tracking the movement origin

        msg.twist.linear.x = x
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = th

        self.pub.publish(msg)

    def timer_callback(self):
        if self.state == 0:
            self.get_logger().info('  Move forward')
            self.move(0.5, 0.0)
            self.timer.cancel()
            self.timer = self.create_timer(2.0, self.timer_callback)
            self.state = 1
        elif self.state == 1:
            self.get_logger().info('  Move backward')
            self.move(-0.5, 0.0)
            self.timer.cancel()
            self.timer = self.create_timer(2.0, self.timer_callback)
            self.state = 2
        elif self.state == 2:
            self.get_logger().info('  Rotate')
            self.move(0.0, 1.0)
            self.timer.cancel()
            self.timer = self.create_timer(2.0, self.timer_callback)
            self.state = 3
        elif self.state == 3:
            self.get_logger().info('  Stop')
            self.move(0.0, 0.0)
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    try:
        with rclpy.init(args=args):
            movement_tester = MovementTester()

            rclpy.spin(movement_tester)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()


