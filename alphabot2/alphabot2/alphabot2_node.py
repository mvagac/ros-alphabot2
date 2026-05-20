import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_controller_alphabot2/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('TwistStamped publisher node has been started.')
        #self.posun(0.5, 0.0)

    def posun(self, x, th):
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

        self.publisher_.publish(msg)

    def timer_callback(self):
        self.posun(0.5, 0.0)
        self.get_logger().info('Publishing...')


def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_publisher = MinimalPublisher()

            rclpy.spin(minimal_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()


