"""
WALL FOLLOW - step 4: PID controller
====================================

New in this step
    * a PID controller turns the error into a steering angle
          P  reacts to the error right now
          I  reacts to an error that stays for a long time
          D  reacts to how fast the error changes
    * the steering angle is only drawn in RViz, it is NOT sent to the car

What you should see
    In RViz a blue arrow shows where the controller wants to steer.
    Move the car closer to the wall: the arrow turns away from the wall.
    Move it away from the wall: the arrow turns towards the wall.

    The car does not move, so the error never goes away. Watch the arrow
    for a minute: it slowly keeps turning, because the I term keeps adding
    up the same error. The controller is shouting louder and louder, but
    nobody is listening. Step 5 fixes that.

Run
    ros2 run wall_follow wall4

RViz topics (Add -> By topic)
    /wall_line       Marker      beams, wall and distance to the wall
    /steering        Marker      arrow at the steering angle we would send
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

        # ---- constants: the PID controller ----
        self.kp = 1.0                    # [rad/m]      gain of the P term
        self.ki = 0.005                  # [rad/(m*s)]  gain of the I term
        self.kd = 0.001                  # [rad*s/m]    gain of the D term
        self.integral_limit = 20.0       # [rad] above this the I term restarts

        # ---- memory of the PID controller (kept between two scans) ----
        self.integral = 0.0              # [rad] the I term so far
        self.prev_error = 0.0            # [m]   error of the previous scan
        self.prev_time = None            # time of the previous scan

        # ---- input ----
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # ---- RViz ----
        self.wall_pub = self.create_publisher(Marker, 'wall_line', 10)
        self.steering_pub = self.create_publisher(Marker, 'steering', 10)

    # ============================================================
    #  PIPELINE - runs once for every laser scan
    # ============================================================
    def scan_callback(self, scan):
        a, b = self.read_beams(scan)
        alpha, distance = self.find_wall(a, b)
        future_distance, error = self.get_error(alpha, distance)
        steering_angle = self.pid_control(error)
        self.visualise(scan, a, b, alpha, distance, steering_angle)

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
    #  LOOK AHEAD AND COMPUTE THE ERROR
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
    #  PID CONTROLLER                                   <-- NEW
    # ============================================================
    def pid_control(self, error):
        """
        Return the steering angle [rad] for this error (+ = left).

        The very first scan returns None: the I and D terms need the time
        since the previous scan, and there is no previous scan yet.
        """
        now = self.get_clock().now()
        steering_angle = None

        if self.prev_time is not None:
            dt = (now.nanoseconds - self.prev_time.nanoseconds) / 1e9   # [s]

            proportional = self.kp * error
            self.integral = self.integral + self.ki * dt * error
            if self.integral > self.integral_limit:
                self.integral = 0.0
            derivative = self.kd * (error - self.prev_error) / dt

            # the wall is on the left: error > 0 (too close) -> steer right
            steering_angle = -proportional - self.integral - derivative

        self.prev_error = error
        self.prev_time = now
        return steering_angle

    # ============================================================
    #  VISUALISE (RViz only)                            <-- NEW
    # ============================================================
    def visualise(self, scan, a, b, alpha, distance, steering_angle):
        self.wall_pub.publish(viz.wall_marker(
            scan,
            a, math.radians(self.beam_a_angle),
            b, math.radians(self.beam_b_angle),
            alpha, distance))
        if steering_angle is not None:
            self.steering_pub.publish(
                viz.steering_marker(scan, steering_angle))


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
