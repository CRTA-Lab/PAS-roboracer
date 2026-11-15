"""
FOLLOW THE GAP - step 4: find the max gap
=========================================

New in this step
    * group the non-zero beams that are next to each other into gaps
    * pick the gap with the most beams

What you should see
    In RViz the max gap is green and the other gaps are grey. With the
    closest point on a side wall there is only one gap. Drive towards an
    obstacle in the middle of the hallway (map `levine_obs`): the bubble
    splits the free space in two and the wider side turns green.

Run
    ros2 run gap_follow gap4

RViz (Add -> By topic)
    /scan_proc       LaserScan   the beams we look at, after preprocessing
    /scan_free       LaserScan   free space: the safety bubble is cut out
    /closest         Marker      red dot on the closest point
    /gaps            Marker      max gap in green, other gaps in grey
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from gap_follow import viz


class ReactiveFollowGap(Node):

    # ============================================================
    #  SETUP
    # ============================================================
    def __init__(self):
        super().__init__('reactive_node')

        # ---- parameters (can be changed from the command line) ----
        self.declare_parameter('max_range', 8.0)       # [m]   see PREPROCESS
        self.declare_parameter('bubble_radius', 0.3)   # [m]   see SAFETY BUBBLE
        self.max_range = self.get_parameter('max_range').value
        self.bubble_radius = self.get_parameter('bubble_radius').value

        # ---- constants ----
        self.fov = math.radians(180.0)   # [rad] we only look at the front half

        # ---- input ----
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10)

        # ---- RViz ----
        self.proc_pub = self.create_publisher(LaserScan, 'scan_proc', 10)
        self.free_pub = self.create_publisher(LaserScan, 'scan_free', 10)
        self.closest_pub = self.create_publisher(Marker, 'closest', 10)
        self.gaps_pub = self.create_publisher(Marker, 'gaps', 10)  # <-- NEW

    # ============================================================
    #  PIPELINE - runs once for every laser scan
    # ============================================================
    def lidar_callback(self, scan):
        ranges, angles = self.read_scan(scan)
        ranges, angles = self.preprocess_lidar(ranges, angles)
        free, closest = self.safety_bubble(ranges, scan.angle_increment)
        gaps = self.find_gaps(free)                            # <-- NEW
        gap = self.find_max_gap(gaps)                          # <-- NEW
        self.visualise(scan, ranges, free, angles, closest, gaps, gap)

        sizes = ', '.join(str(len(g)) for g in gaps)
        self.get_logger().info(
            f'{len(gaps)} gaps, beams in each: {sizes} | max gap: {len(gap)}',
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
    #  PREPROCESS
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
    #  SAFETY BUBBLE
    # ============================================================
    def safety_bubble(self, ranges, angle_increment):
        """
        Find the closest point and set every beam around it to 0.

        Returns the "free space" ranges (0 = not free) and the index of
        the closest point.
        """
        closest = int(np.argmin(ranges))
        distance = max(ranges[closest], 0.05)
        # how many beams does `bubble_radius` metres cover at that distance?
        width = math.ceil(self.bubble_radius / (distance * angle_increment))

        free = ranges.copy()
        free[max(closest - width, 0):closest + width + 1] = 0.0
        return free, closest

    # ============================================================
    #  FIND MAX GAP                                 <-- NEW
    # ============================================================
    def find_gaps(self, free):
        """
        Return a list of gaps; every gap is an array of beam indices.

        A gap is a run of non-zero beams next to each other.
        """
        nonzero = np.where(free > 0.0)[0]
        if nonzero.size == 0:
            # the bubble covers everything -> no gap, keep going straight
            return [np.array([len(free) // 2])]
        # start a new gap wherever two non-zero beams are not neighbours
        breaks = np.where(np.diff(nonzero) != 1)[0] + 1
        return np.split(nonzero, breaks)

    def find_max_gap(self, gaps):
        """Return the gap with the most beams."""
        return max(gaps, key=len)

    # ============================================================
    #  VISUALISE (RViz only)
    # ============================================================
    def visualise(self, scan, ranges, free, angles, closest, gaps, gap):
        self.proc_pub.publish(viz.scan_msg(scan, ranges, angles))
        self.free_pub.publish(viz.scan_msg(scan, free, angles))
        self.closest_pub.publish(
            viz.closest_marker(scan, ranges[closest], angles[closest]))
        self.gaps_pub.publish(
            viz.gaps_marker(scan, free, angles, gaps, chosen=gap))


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
