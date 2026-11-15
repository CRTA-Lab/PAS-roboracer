"""
FOLLOW THE GAP - step 1: read the laser scan
============================================

New in this step
    * subscribe to the lidar topic `scan`
    * work out the angle of every beam

What you should see
    Once per second the terminal prints how many beams the lidar has, which
    angles they cover, how far the wall straight ahead is and where the
    closest obstacle is. Drive the car with the keyboard and watch the
    numbers change.

Run
    ros2 run gap_follow gap1
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ReactiveFollowGap(Node):

    # ============================================================
    #  SETUP
    # ============================================================
    def __init__(self):
        super().__init__('reactive_node')

        # ---- input ----
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10)

    # ============================================================
    #  PIPELINE - runs once for every laser scan
    # ============================================================
    def lidar_callback(self, scan):
        ranges, angles = self.read_scan(scan)

        ahead = np.argmin(np.abs(angles))   # the beam closest to angle 0
        closest = np.argmin(ranges)
        self.get_logger().info(
            f'{len(ranges)} beams from {math.degrees(angles[0]):.0f} deg '
            f'to {math.degrees(angles[-1]):.0f} deg '
            f'(one beam every {math.degrees(scan.angle_increment):.2f} deg) | '
            f'straight ahead: {ranges[ahead]:.2f} m | '
            f'closest: {ranges[closest]:.2f} m '
            f'at {math.degrees(angles[closest]):.0f} deg',
            throttle_duration_sec=1.0)

    # ============================================================
    #  READ THE SCAN                                    <-- NEW
    # ============================================================
    def read_scan(self, scan):
        """Return the distance and the angle of every beam."""
        ranges = np.array(scan.ranges)
        # angle 0 is straight ahead, positive angles are to the left
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        return ranges, angles


# ============================================================
#  MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
