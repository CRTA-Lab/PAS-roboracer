"""
DRIVE MUX - who is allowed to drive the car
===========================================

Several nodes want to drive the car at the same time: a controller
(wall_follow, gap_follow, ...) and the safety node. A mux (multiplexer) has
many inputs and one output. It decides whose command reaches the motor.

    drive  (controller, low priority)  --+
                                         +--> MUX --> ackermann_cmd --> car
    safety (safety node, high priority) -+

Rule
    A `safety` command always goes through. After it, the mux throws away
    every `drive` command for `safety_timeout` seconds. The safety node sends
    a command for every scan while there is danger, so the car stays locked
    for as long as the danger lasts and `safety_timeout` seconds after it.

The real car has the same thing (the `ackermann_mux` package). The simulator
does not, so there the brake command of the safety node and the commands of
the controller arrive mixed on one topic and the car never really stops.

Run (simulator and mux together, the simulator then listens on ackermann_cmd)
    ros2 launch drive_mux sim_mux_launch.py

Run (the mux alone)
    ros2 run drive_mux drive_mux
    ros2 run drive_mux drive_mux --ros-args -p safety_timeout:=1.0
"""
from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class DriveMux(Node):

    # ============================================================
    #  SETUP
    # ============================================================
    def __init__(self):
        super().__init__('drive_mux')

        # ---- parameters (can be changed while the node runs) ----
        # [s] how long `drive` stays blocked after the last `safety` command
        self.declare_parameter('safety_timeout', 0.5)

        # ---- memory ----
        self.last_safety_time = None     # time of the last `safety` command

        # ---- input ----
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, 'drive', self.drive_callback, 10)
        self.safety_sub = self.create_subscription(
            AckermannDriveStamped, 'safety', self.safety_callback, 10)

        # ---- output ----
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped, 'ackermann_cmd', 10)

    # ============================================================
    #  HIGH PRIORITY - always goes through
    # ============================================================
    def safety_callback(self, msg):
        if not self.safety_active():
            self.get_logger().warn('safety took over, drive is blocked')
        self.last_safety_time = self.get_clock().now()
        self.cmd_pub.publish(msg)

    # ============================================================
    #  LOW PRIORITY - goes through only while safety is silent
    # ============================================================
    def drive_callback(self, msg):
        if self.safety_active():
            return
        if self.last_safety_time is not None:
            self.get_logger().info('safety is silent, drive is back')
            self.last_safety_time = None
        self.cmd_pub.publish(msg)

    # ============================================================
    #  IS SAFETY IN CONTROL?
    # ============================================================
    def safety_active(self):
        """Return True if a `safety` command came in the last moments."""
        if self.last_safety_time is None:
            return False
        timeout = Duration(
            seconds=self.get_parameter('safety_timeout').value)
        return self.get_clock().now() - self.last_safety_time < timeout


# ============================================================
#  MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = DriveMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
