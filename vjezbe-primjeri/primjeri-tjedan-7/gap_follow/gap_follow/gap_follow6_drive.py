"""
FOLLOW THE GAP - step 6: drive
==============================

New in this step
    * turn the angle of the best point into a steering command
    * publish it, together with a constant speed, on the topic `drive`

What you should see
    The car drives on its own, following the arrow.

    Three things are still wrong. Next to a corner or an obstacle the arrow
    can point so close to the edge that the car clips it (step 7). The car
    takes corners at full speed, and when you press Ctrl+C it keeps rolling
    with the last command it got (step 8).

Run
    ros2 run gap_follow gap6
    ros2 run gap_follow gap6 --ros-args -p speed:=0.5

RViz (Add -> By topic)
    /scan_proc       LaserScan   the beams we look at, after preprocessing
    /scan_free       LaserScan   free space: the safety bubble is cut out
    /closest         Marker      red dot on the closest point
    /gaps            Marker      max gap in green, other gaps in grey
    /target          Marker      arrow to the best point
"""
import math

from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from rclpy.signals import SignalHandlerOptions

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
        self.declare_parameter('far_ratio', 0.95)      # [-]   see FIND BEST POINT
        self.declare_parameter('speed', 1.0)           # [m/s] driving speed   <-- NEW
        self.max_range = self.get_parameter('max_range').value
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.far_ratio = self.get_parameter('far_ratio').value
        self.speed = self.get_parameter('speed').value

        # ---- constants ----
        self.fov = math.radians(180.0)   # [rad] we only look at the front half
        self.max_steer = 0.34            # [rad] steering limit of the car   <-- NEW

        # ---- input ----
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10)

        # ---- output ----                                     # <-- NEW
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        # ---- RViz ----
        self.proc_pub = self.create_publisher(LaserScan, 'scan_proc', 10)
        self.free_pub = self.create_publisher(LaserScan, 'scan_free', 10)
        self.closest_pub = self.create_publisher(Marker, 'closest', 10)
        self.gaps_pub = self.create_publisher(Marker, 'gaps', 10)
        self.target_pub = self.create_publisher(Marker, 'target', 10)

    # ============================================================
    #  PIPELINE - runs once for every laser scan
    # ============================================================
    def lidar_callback(self, scan):
        ranges, angles = self.read_scan(scan)
        ranges, angles = self.preprocess_lidar(ranges, angles)
        free, closest = self.safety_bubble(ranges, scan.angle_increment)
        gaps = self.find_gaps(free)
        gap = self.find_max_gap(gaps)
        best = self.find_best_point(free, gap)
        steer, speed = self.drive(angles[best])                # <-- NEW
        self.visualise(scan, ranges, free, angles, closest, gaps, gap, best)

        self.get_logger().info(
            f'best point at {round(math.degrees(angles[best]))} deg -> '
            f'steer {round(math.degrees(steer))} deg '
            f'(limit {math.degrees(self.max_steer):.0f} deg) | '
            f'speed {speed:.1f} m/s',
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
    #  FIND MAX GAP
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
    #  FIND BEST POINT
    # ============================================================
    def find_best_point(self, free, gap):
        """
        Return the index of the beam to aim at.

        Naive: the furthest beam in the gap. Many beams are equally far
        (all the capped ones), so we take the widest group of far beams
        and aim at its middle.
        """
        depth = free[gap]
        far = gap[depth >= self.far_ratio * depth.max()]
        # split the far beams into groups of neighbours (same trick as gaps)
        breaks = np.where(np.diff(far) != 1)[0] + 1
        widest = max(np.split(far, breaks), key=len)
        return widest[len(widest) // 2]

    # ============================================================
    #  DRIVE                                        <-- NEW
    # ============================================================
    def drive(self, target_angle):
        """Steer towards the best point at a constant speed."""
        # the wheels cannot turn further than `max_steer`
        steer = float(np.clip(target_angle, -self.max_steer, self.max_steer))
        self.publish_drive(steer, self.speed)
        return steer, self.speed

    def publish_drive(self, steer, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = steer
        msg.drive.speed = speed
        self.drive_pub.publish(msg)

    # ============================================================
    #  VISUALISE (RViz only, does not change how the car drives)
    # ============================================================
    def visualise(self, scan, ranges, free, angles, closest, gaps, gap, best):
        self.proc_pub.publish(viz.scan_msg(scan, ranges, angles))
        self.free_pub.publish(viz.scan_msg(scan, free, angles))
        self.closest_pub.publish(
            viz.closest_marker(scan, ranges[closest], angles[closest]))
        self.gaps_pub.publish(
            viz.gaps_marker(scan, free, angles, gaps, chosen=gap))
        self.target_pub.publish(
            viz.target_marker(scan, free[best], angles[best]))


# ============================================================
#  MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ReactiveFollowGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_drive(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
