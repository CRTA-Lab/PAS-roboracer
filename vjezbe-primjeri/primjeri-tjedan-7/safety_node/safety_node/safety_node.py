"""
SAFETY NODE - automatic emergency braking
=========================================

The node watches the laser scan and the speed of the car. For every beam it
computes the time to collision (TTC): how many seconds until the car hits
what that beam sees, if the car keeps driving as it does now.

The TTC math is the one from F1TENTH lab 2
(https://github.com/f1tenth/f1tenth_labs/tree/legacy/lab2):

    TTC = r / max(-r_dot, 0)
    r      distance measured by the beam
    r_dot  how fast that distance changes = -speed * cos(beam angle)

If the smallest TTC is below `ttc_threshold`, the node brakes: it sends
speed 0 on `drive` for every scan.

    scan + odom -> TTC of every beam -> smallest TTC -> brake?

The brake is held
    A stopped car has no TTC, so "brake only while there is danger" would let
    the car go the moment it stands still, and the controller would drive it
    at the obstacle again. So once the node brakes, it keeps braking until a
    human takes over and sends a manual command:

        simulator   keyboard teleop   (`cmd_vel`)
        real car    joystick          (`teleop`)

    A manual command with speed 0 and steering 0 does not count.

Simulator and real car
    The speed comes from `ego_racecar/odom` in the simulator and from `odom`
    on the real car. The node listens on both, so the same command works.

Run
    ros2 run safety_node safety_node
    ros2 run safety_node safety_node --ros-args -p ttc_threshold:=1.0

Test in the simulator (needs the drive mux, see drive_mux/README.md)
    ros2 launch drive_mux sim_mux_launch.py
    ros2 run safety_node safety_node --ros-args -r drive:=safety
    ros2 topic pub -r 20 /drive ackermann_msgs/msg/AckermannDriveStamped \
        "{drive: {speed: 2.0}}"
  The car must stop in front of the wall and stay there. To release it,
  drive it away by hand:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SafetyNode(Node):

    # ============================================================
    #  SETUP
    # ============================================================
    def __init__(self):
        super().__init__('safety_node')

        # ---- parameters (can be changed while the node runs) ----
        self.declare_parameter('ttc_threshold', 0.5)   # [s] brake below this

        # ---- constants ----
        self.drive_frame = 'base_link'   # frame written into the command

        # ---- memory ----
        self.speed = 0.0                 # [m/s] last speed from odometry
        self.braking = False             # True = brake is held

        # ---- input ----
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sim_sub = self.create_subscription(
            Odometry, 'ego_racecar/odom', self.odom_callback, 10)   # simulator
        self.odom_car_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)               # real car
        self.keyboard_sub = self.create_subscription(
            Twist, 'cmd_vel', self.keyboard_callback, 10)           # simulator
        self.joystick_sub = self.create_subscription(
            AckermannDriveStamped, 'teleop', self.joystick_callback, 10)  # car

        # ---- output ----
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, 'drive', 10)

    # ============================================================
    #  SPEED OF THE CAR
    # ============================================================
    def odom_callback(self, msg):
        """Remember the forward speed (+ = forward, - = reverse)."""
        self.speed = msg.twist.twist.linear.x

    # ============================================================
    #  PIPELINE - runs once for every laser scan
    # ============================================================
    def scan_callback(self, scan):
        ttc = self.min_ttc(scan)
        if ttc < self.get_parameter('ttc_threshold').value:
            if not self.braking:
                self.get_logger().warn(
                    f'TTC = {ttc:.2f} s at {self.speed:.2f} m/s -> BRAKE, '
                    'held until a manual command')
            self.braking = True
        if self.braking:
            self.brake()

    # ============================================================
    #  MANUAL COMMAND - a human takes over, let go of the brake
    # ============================================================
    def keyboard_callback(self, msg):
        """Keyboard teleop in the simulator."""
        self.manual_command(msg.linear.x, msg.angular.z)

    def joystick_callback(self, msg):
        """Joystick on the real car."""
        self.manual_command(msg.drive.speed, msg.drive.steering_angle)

    def manual_command(self, speed, steering):
        if speed == 0.0 and steering == 0.0:
            return                       # an idle joystick is not a human
        if self.braking:
            self.get_logger().info('manual command -> brake released')
        self.braking = False

    # ============================================================
    #  TIME TO COLLISION
    # ============================================================
    def min_ttc(self, scan):
        """Return the smallest TTC [s] over all beams (inf = no danger)."""
        r = np.array(scan.ranges)
        angles = scan.angle_min + np.arange(len(r)) * scan.angle_increment

        # r_dot: the speed of the car projected onto each beam [m/s]
        # (negative = the distance shrinks = we drive towards that point)
        r_dot = -self.speed * np.cos(angles)

        # max(-r_dot, 0): only beams we are driving towards count
        closing_speed = np.maximum(-r_dot, 0.0)

        # skip beams with no measurement and beams we are not driving towards
        valid = (np.isfinite(r)
                 & (r >= scan.range_min)
                 & (r <= scan.range_max)
                 & (closing_speed > 0.0))
        if not np.any(valid):
            return np.inf
        return float(np.min(r[valid] / closing_speed[valid]))

    # ============================================================
    #  BRAKE
    # ============================================================
    def brake(self):
        """Tell the car to stand still."""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.drive_frame
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 0.0
        self.drive_pub.publish(msg)


# ============================================================
#  MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
