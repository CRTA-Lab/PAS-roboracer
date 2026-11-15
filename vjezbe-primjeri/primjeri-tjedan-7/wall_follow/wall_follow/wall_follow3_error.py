"""
WALL FOLLOW - step 3: look ahead, compute the error
===================================================

New in this step
    * predict the distance to the wall `look_ahead` metres further down
      the road, assuming the car keeps going straight
    * error = the distance we want - the distance we predict

What you should see
    One line per second, for example

        distance = 0.70 m, alpha = -20.0 deg -> future 0.36 m, error = +0.34 m

    Repeat the test of step 2 (0.7 m from the wall, nose at the wall):
    the distance is still "perfect", but the error is not zero any more.
        error > 0   the car will be too close to the wall
        error < 0   the car will be too far from the wall

    The car does not move: we know the error, but nothing turns it into a
    steering command. Step 4 fixes that.

Run
    ros2 run wall_follow wall3
    ros2 run wall_follow wall3 --ros-args -p wall_dist:=1.0 -p look_ahead:=0.5

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

        # ---- parameters (can be changed while the node runs) ----
        self.declare_parameter('wall_dist', 0.7)     # [m] distance we want
        self.declare_parameter('look_ahead', 1.0)    # [m] how far we predict

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
        future_distance, error = self.get_error(alpha, distance)
        self.visualise(scan, a, b, alpha, distance)

        self.get_logger().info(
            f'distance = {distance:.2f} m, '
            f'alpha = {math.degrees(alpha):+.1f} deg -> '
            f'future {future_distance:.2f} m, error = {error:+.2f} m',
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
    #  FIND THE WALL
    # ============================================================
    def find_wall(self, a, b):
        """Return the angle of the car to the wall and the distance to it."""
        theta = math.radians(self.beam_b_angle - self.beam_a_angle)
        alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
        distance = b * np.cos(alpha)
        return alpha, distance

    # ============================================================
    #  LOOK AHEAD AND COMPUTE THE ERROR                 <-- NEW
    # ============================================================
    def get_error(self, alpha, distance):
        """Return the predicted distance to the wall and the error."""
        wall_dist = self.get_parameter('wall_dist').value
        look_ahead = self.get_parameter('look_ahead').value
        # driving `look_ahead` metres at angle `alpha` to the wall
        # changes the distance by look_ahead * sin(alpha)
        future_distance = distance + look_ahead * np.sin(alpha)
        error = wall_dist - future_distance
        return future_distance, error

    # ============================================================
    #  VISUALISE (RViz only)
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
