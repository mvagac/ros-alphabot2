#!/usr/bin/env python3

import csv
import math
import threading
import time
from typing import List, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class Nav2BenchmarkNode(Node):
    def __init__(self):
        super().__init__('nav2_benchmark_node')

        # publisher to send initial pose
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # subscriber to receive global plan computed by Nav2
        self._plan_sub = self.create_subscription(Path, '/plan', self._plan_callback, 10)
        # action client for Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.output_csv = 'nav2_performance_metrics.csv'
        self.goals_list: List[Tuple[float, float, float]] = [
            # X, Y, Yaw
            #(0.26, 0.23, 0.0),
            (0.22, 0.15, 0.0),
            (0.24, -0.1, -0.6),

        ]
        self.current_path_length = 0.0

        self._set_initial_pose(-0.216, 0.065, -0.013)
        time.sleep(3.0)

        self.get_logger().info('Nav2BenchmarkNode node has been started.')

    def _set_initial_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # standard initial covariance matrix used by RViz2 (6x6 matrix flattened to 36 elements)
        msg.pose.covariance[0] = 0.25  # X variance
        msg.pose.covariance[7] = 0.25  # Y variance
        msg.pose.covariance[35] = 0.068  # Yaw variance

        self.get_logger().info(
            f'Publishing initial pose: x={x}, y={y}, yaw={yaw}'
        )
        self._pose_pub.publish(msg)

    def _plan_callback(self, msg: Path):
        self.get_logger().info(f'  Path plan received')
        if len(msg.poses) < 2:
            return

        total_length = 0.0
        for i in range(1, len(msg.poses)):
            p1 = msg.poses[i - 1].pose.position
            p2 = msg.poses[i].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dz = p2.z - p1.z
            total_length += math.sqrt(dx * dx + dy * dy + dz * dz)

        self.current_path_length = total_length

    def create_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert yaw to quaternion (Z-axis rotation)
        #TODO !@#
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def run_benchmark(self):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 Action Server not available! Exiting...')
            return

        self.get_logger().info(f'Starting benchmark across {len(self.goals_list)} goal poses...')

        metrics_data = []
        for index, (x, y, yaw) in enumerate(self.goals_list):
            self.get_logger().info(f'  Goal {index + 1}/{len(self.goals_list)}: [{x}, {y}, {yaw}]')

            self.current_path_length = 0.0

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.create_pose_stamped(x, y, yaw)

            t_start = self.get_clock().now()
            t_start_sec = t_start.nanoseconds / 1e9

            # send goal and wait
            send_goal_future = self._action_client.send_goal_async(goal_msg)
            while not send_goal_future.done():
                time.sleep(0.01)
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f'    Goal {index + 1} was rejected by server.')
                continue

            # get goal result
            get_result_future = goal_handle.get_result_async()
            while not get_result_future.done():
                time.sleep(0.01)

            t_end = self.get_clock().now()
            t_end_sec = t_end.nanoseconds / 1e9

            # process results
            result = get_result_future.result()
            status = result.status

            duration_sec = t_end_sec - t_start_sec

            if status == GoalStatus.STATUS_SUCCEEDED:
                status_str = 'STATUS_SUCCEEDED'
            elif status == GoalStatus.STATUS_ABORTED:
                status_str = 'STATUS_ABORTED'
            elif status == GoalStatus.STATUS_CANCELED:
                status_str = 'STATUS_CANCELED'
            else:
                status_str = f'UNKNOWN_{status}'

            time.sleep(0.1)
            path_length = self.current_path_length
            if duration_sec > 0 and status == GoalStatus.STATUS_SUCCEEDED:
                v_avg = path_length / duration_sec
            else:
                v_avg = 0.0

            # store results
            self.get_logger().info(
                f'Goal {index + 1} Result: {status_str} | '
                f'Time: {duration_sec:.2f} s | '
                f'Path Length: {path_length:.3f} m | '
                f'Speed: {v_avg:.3f} m/s'
            )

            metrics_data.append({
                'goal_id': index + 1,
                'target_x': x,
                'target_y': y,
                'status': status_str,
                't_start_s': t_start_sec,
                't_end_s': t_end_sec,
                'duration_s': duration_sec,
                'path_length_m': path_length,
                'operational_speed_m_s': v_avg
            })

            time.sleep(1.0)

        if not metrics_data:
            return

        # save to csv
        keys = metrics_data[0].keys()
        with open(self.output_csv, 'w', newline='') as output_file:
            dict_writer = csv.DictWriter(output_file, fieldnames=keys)
            dict_writer.writeheader()
            dict_writer.writerows(metrics_data)
        self.get_logger().info(f'Benchmark performance data saved to {self.output_csv}')

        # print summary
        successful_runs = [m for m in metrics_data if m['status'] == 'STATUS_SUCCEEDED']
        success_rate = (len(successful_runs) / len(self.goals_list)) * 100.0

        if successful_runs:
            avg_time = sum(m['duration_s'] for m in successful_runs) / len(successful_runs)
            avg_speed = sum(m['operational_speed_m_s'] for m in successful_runs) / len(successful_runs)
            avg_path = sum(m['path_length_m'] for m in successful_runs) / len(successful_runs)
        else:
            avg_time, avg_speed, avg_path = 0.0, 0.0, 0.0

        self.get_logger().info('=== BENCHMARK SUMMARY ===')
        self.get_logger().info(f'Total Goals Attempted: {len(self.goals_list)}')
        self.get_logger().info(f'Navigation Success Rate: {success_rate:.1f}%')
        self.get_logger().info(f'Average Goal Completion Time: {avg_time:.2f} s')
        self.get_logger().info(f'Average Path Length: {avg_path:.3f} m')
        self.get_logger().info(f'Average Operational Speed: {avg_speed:.3f} m/s')


def main(args=None):
    rclpy.init(args=args)
    node = Nav2BenchmarkNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_benchmark()
    except KeyboardInterrupt:
        node.get_logger().info('Benchmark interrupted by user.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

