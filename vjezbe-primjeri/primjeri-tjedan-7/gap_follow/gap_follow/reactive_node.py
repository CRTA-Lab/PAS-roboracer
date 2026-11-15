"""
FOLLOW THE GAP - step 8 (final): drive smoothly and stop cleanly
================================================================

This is the node Lab 4 asks for: it reads `/scan`, publishes `/drive` and
is started by the two launch files, one per map.

New in this step
    * the car slows down in sharp turns
    * every number worth tuning is a parameter, so each map can have its
      own values (see the files in `launch/`)
    * Ctrl+C sends a stop command, so the car does not keep rolling

What you should see
    The car drives laps on its own, brakes before corners and stops when
    you press Ctrl+C.

Run
    ros2 run gap_follow reactive_node
    ros2 run gap_follow reactive_node --ros-args -p speed:=2.0
    ros2 launch gap_follow levine_blocked_launch.py
    ros2 launch gap_follow levine_obs_launch.py

RViz (Add -> By topic)
    /scan_proc       LaserScan   the beams we look at, after preprocessing
    /scan_extended   LaserScan   the same beams with obstacles widened
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
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


from gap_follow import viz


class ReactiveFollowGap(Node):

    # ============================================================
    #  SETUP
    # ============================================================
    def __init__(self):
        super().__init__('reactive_node')

        # ---- parameters (set from the command line or a launch file) ----
        self.declare_parameter('speed', 1.0)           # [m/s] top speed
        self.declare_parameter('max_range', 8.0)       # [m]   see PREPROCESS
        self.declare_parameter('bubble_radius', 0.3)   # [m]   see SAFETY BUBBLE
        self.declare_parameter('clearance', 0.18)      # [m]   see EXTEND
        self.declare_parameter('far_ratio', 0.95)      # [-]   see BEST POINT
        self.speed = self.get_parameter('speed').value
        self.max_range = self.get_parameter('max_range').value
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.clearance = self.get_parameter('clearance').value
        self.far_ratio = self.get_parameter('far_ratio').value

        # ---- constants ----
        self.fov = math.radians(180.0)   # [rad] we only look at the front half
        self.disparity = 0.1             # [m]   jump that counts as an edge
        self.max_steer = 0.34            # [rad] steering limit of the car

        # ---- input ----
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10)

        # ---- output ----
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        # ---- RViz ----
        self.proc_pub = self.create_publisher(LaserScan, 'scan_proc', 10)
        self.extended_pub = self.create_publisher(LaserScan, 'scan_extended', 10)
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
        extended = self.extend_disparities(ranges, scan.angle_increment)
        free, closest = self.safety_bubble(extended, scan.angle_increment)
        gaps = self.find_gaps(free)
        gap = self.find_max_gap(gaps)
        best = self.find_best_point(free, gap)
        steer, speed = self.drive(angles[best])
        self.visualise(
            scan, ranges, extended, free, angles, closest, gaps, gap, best)

        self.get_logger().info(
            f'steer {round(math.degrees(steer))} deg -> '
            f'speed {speed:.2f} m/s '
            f'({100 * speed / self.speed:.0f} % of top speed)',
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
    #  EXTEND DISPARITIES
    # ============================================================
    def extend_disparities(self, ranges, angle_increment):
        """
        Widen every obstacle edge by half a car width.

        The lidar is a point, the car is not. Wherever the distance jumps
        between two neighbouring beams (a "disparity") there is an edge the
        car could clip, so the beams next to it on the far side are pulled
        in to the distance of the near side.
        """
        extended = ranges.copy()
        for i in range(len(ranges) - 1):
            jump = ranges[i + 1] - ranges[i]
            if abs(jump) <= self.disparity:
                continue

            close = max(min(ranges[i], ranges[i + 1]), 0.05)
            # how many beams does `clearance` metres cover at that distance?
            width = math.ceil(self.clearance / (close * angle_increment))

            if jump > 0:
                # far side is at the higher indices -> extend forward
                start, end = i + 1, min(i + 1 + width, len(ranges))
            else:
                # far side is at the lower indices -> extend backward
                start, end = max(i - width + 1, 0), i + 1
            extended[start:end] = np.minimum(extended[start:end], close)
        return extended

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
    #  DRIVE
    # ============================================================
    def drive(self, target_angle):
        """Steer towards the best point and pick a speed for that steering."""
        steer = float(np.clip(target_angle, -self.max_steer, self.max_steer))

        # slow down in sharp turns                             # <-- NEW
        if abs(steer) > 0.2:
            speed = self.speed * 0.5
        elif abs(steer) > 0.1:
            speed = self.speed * 0.75
        else:
            speed = self.speed

        self.publish_drive(steer, speed)
        return steer, speed

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
    def visualise(self, scan, ranges, extended, free, angles,
                  closest, gaps, gap, best):
        self.proc_pub.publish(viz.scan_msg(scan, ranges, angles))
        self.extended_pub.publish(viz.scan_msg(scan, extended, angles))
        self.free_pub.publish(viz.scan_msg(scan, free, angles))
        self.closest_pub.publish(
            viz.closest_marker(scan, extended[closest], angles[closest]))
        self.gaps_pub.publish(
            viz.gaps_marker(scan, free, angles, gaps, chosen=gap))
        self.target_pub.publish(
            viz.target_marker(scan, free[best], angles[best]))


# ============================================================
#  MAIN
# ============================================================
def main(args=None):
    # handle Ctrl+C ourselves so the context is still alive      # <-- NEW
    # to send a stop command
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
