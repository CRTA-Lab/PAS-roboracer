"""
WALL FOLLOW - step 2: where is the wall?
========================================

New in this step
    * two points are enough to describe a straight wall. From `a` and `b`
      we compute
          alpha     how much the car is turned relative to the wall
                    (0 = parallel, + = heading away from the wall)
          distance  the real, perpendicular distance to the wall
    * publish the result so RViz can draw it

What you should see
    In RViz: the two beams (grey), the wall through the two points they hit
    (orange) and the shortest line from the car to the wall (green).
    Turn the car on the spot: the green line keeps its length.

    The car does not move. Now place it 0.7 m from the wall with its nose
    pointing AT the wall. The green line says "0.7 m, perfect", but the car
    would hit the wall in a moment. The distance right now is not enough.
    Step 3 fixes that.

Run
    ros2 run wall_follow wall2

RViz topics (Add -> By topic)
    /wall_line       Marker      beams, wall and distance to the wall
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from wall_follow import viz


class WallFollow(Node):

    # ============================================================
    #  SETUP
    # ============================================================
    def __init__(self):
        super().__init__('wall_follow_node')

        # ---- constants: the lidar ----
        self.lidar_angle_min = -135.0    # [deg] angle of beam number 0
        self.lidar_resolution = 0.25     # [deg] angle between two beams

        # ---- constants: the two beams we use (0 = ahead, + = left) ----
        self.beam_a_angle = 45.0         # [deg] forward-left
        self.beam_b_angle = 90.0         # [deg] straight left

        # ---- input ----
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # ---- RViz ----
        self.wall_pub = self.create_publisher(Marker, 'wall_line', 10)

    # ============================================================
    #  PIPELINE - runs once for every laser scan
    # ============================================================
    def scan_callback(self, scan):
        a, b = self.read_beams(scan)
        alpha, distance = self.find_wall(a, b)
        self.visualise(scan, a, b, alpha, distance)

        self.get_logger().info(
            f'a ({self.beam_a_angle:.0f} deg) = {a:.2f} m    '
            f'b ({self.beam_b_angle:.0f} deg) = {b:.2f} m',
            throttle_duration_sec=1.0)

    # ============================================================
    #  READ TWO BEAMS
    # ============================================================
    def beam_index(self, angle):
        """Return the position in `scan.ranges` of the beam at `angle` [deg]."""
        return round((angle - self.lidar_angle_min) / self.lidar_resolution)

    def read_beams(self, scan):
        """Return the distances measured by beam a and beam b."""
        a = float(scan.ranges[self.beam_index(self.beam_a_angle)])
        b = float(scan.ranges[self.beam_index(self.beam_b_angle)])
        return a, b

    # ============================================================
    #  FIND THE WALL                                    <-- NEW
    # ============================================================
    def find_wall(self, a, b):
        """Return the angle of the car to the wall and the distance to it."""
        theta = math.radians(self.beam_b_angle - self.beam_a_angle)
        alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
        distance = b * np.cos(alpha)
        return alpha, distance

    # ============================================================
    #  VISUALISE (RViz only)                            <-- NEW
    # ============================================================
    def visualise(self, scan, a, b, alpha, distance):
        self.wall_pub.publish(viz.wall_marker(
            scan,
            a, math.radians(self.beam_a_angle),
            b, math.radians(self.beam_b_angle),
            alpha, distance))


# ============================================================
#  MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = WallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
