"""
WALL FOLLOW - the finished node
===============================

The car follows the wall on its LEFT side with a PID controller.
This is the same code as step 5 (`wall_follow5_drive.py`); the lesson
files `wall_follow1_...` to `wall_follow5_...` build it up one idea at a time.

    scan -> two beams -> wall angle and distance -> look ahead, error
         -> PID -> steering angle -> drive

What you should see
    The car drives on its own, about `wall_dist` metres from the left wall.
    Press Ctrl+C: the car stops.

Run
    ros2 run wall_follow wall_follow
    ros2 run wall_follow wall_follow --ros-args -p wall_dist:=1.0 -p look_ahead:=0.5

    Change the speed while it drives (the node listens on `vehicle_speed`):
    ros2 topic pub --once /vehicle_speed ackermann_msgs/msg/AckermannDrive "{speed: 1.5}"

RViz topics (Add -> By topic)
    /wall_line       Marker      beams, wall and distance to the wall
    /steering        Marker      arrow at the steering angle we send
"""
import math

from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
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

        # ---- constants: driving ----
        self.vehicle_speed = 1.0         # [m/s] until `vehicle_speed` says else
        self.drive_frame = 'base_link'   # frame written into the command
        self.stop_repeats = 3            # send "stop" this many times

        # ---- input ----
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.speed_sub = self.create_subscription(
            AckermannDrive, 'vehicle_speed', self.speed_callback, 10)

        # ---- output ----
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, 'drive', 10)

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
        if steering_angle is not None:
            self.drive(steering_angle)
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
    #  PID CONTROLLER
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
    #  DRIVE
    # ============================================================
    def speed_callback(self, msg):
        """Another node (or `ros2 topic pub`) can change the speed."""
        self.vehicle_speed = msg.speed

    def drive(self, steering_angle):
        """Send the steering angle and the speed to the car."""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.drive_frame
        msg.drive.steering_angle = float(steering_angle)
        msg.drive.speed = float(self.vehicle_speed)
        self.drive_pub.publish(msg)

    def stop(self):
        """Tell the car to stand still (used when the node is closed)."""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.drive_frame
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 0.0
        for _ in range(self.stop_repeats):
            self.drive_pub.publish(msg)

    # ============================================================
    #  VISUALISE (RViz only)
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
    # SignalHandlerOptions.NO: ROS does not shut down on Ctrl+C by itself,
    # so we still have time to send "stop" to the car.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C: stopping the car')
        node.stop()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
