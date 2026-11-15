"""
FOLLOW THE GAP - step 2: preprocess the scan
============================================

New in this step
    * keep only the beams in front of the car
    * replace `inf` (the beam hit nothing) with a number
    * cap every distance at `max_range`: further than that is "far"

What you should see
    In RViz, `/scan_proc` covers only the half circle in front of the car
    and stops `max_range` metres away. The terminal prints how many beams
    are left and the furthest and closest distance.

Run
    ros2 run gap_follow gap2
    ros2 run gap_follow gap2 --ros-args -p max_range:=3.0

RViz (Add -> By topic)
    /scan_proc       LaserScan   the beams we look at, after preprocessing
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from gap_follow import viz


class ReactiveFollowGap(Node):

    # ============================================================
    #  SETUP
    # ============================================================
    def __init__(self):
        super().__init__('reactive_node')

        # ---- parameters (can be changed from the command line) ----
        self.declare_parameter('max_range', 8.0)       # [m]   see PREPROCESS   <-- NEW
        self.max_range = self.get_parameter('max_range').value

        # ---- constants ----
        self.fov = math.radians(180.0)   # [rad] we only look at the front half   <-- NEW

        # ---- input ----
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10)

        # ---- RViz ----
        self.proc_pub = self.create_publisher(LaserScan, 'scan_proc', 10)  # <-- NEW

    # ============================================================
    #  PIPELINE - runs once for every laser scan
    # ============================================================
    def lidar_callback(self, scan):
        ranges, angles = self.read_scan(scan)
        ranges, angles = self.preprocess_lidar(ranges, angles)  # <-- NEW
        self.visualise(scan, ranges, angles)

        self.get_logger().info(
            f'{len(ranges)} beams kept | '
            f'furthest: {ranges.max():.2f} m | closest: {ranges.min():.2f} m',
            throttle_duration_sec=1.0)

    # ============================================================
    #  READ THE SCAN
    # ============================================================
    def read_scan(self, scan):
        """Return the distance and the angle of every beam."""
        ranges = np.array(scan.ranges)
        # angle 0 is straight ahead, positive angles are to the left
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        return ranges, angles

    # ============================================================
    #  PREPROCESS                                   <-- NEW
    # ============================================================
    def preprocess_lidar(self, ranges, angles):
        """Keep the beams in front of the car and cap the far ones."""
        front = np.abs(angles) <= self.fov / 2
        ranges = ranges[front]
        angles = angles[front]
        # a beam that hits nothing reports `inf` (or `nan`) -> "very far"
        ranges = np.nan_to_num(ranges, nan=self.max_range, posinf=self.max_range)
        # everything further than `max_range` counts as equally far
        ranges = np.minimum(ranges, self.max_range)
        return ranges, angles

    # ============================================================
    #  VISUALISE (RViz only)
    # ============================================================
    def visualise(self, scan, ranges, angles):
        self.proc_pub.publish(viz.scan_msg(scan, ranges, angles))


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
