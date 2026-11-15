"""
WALL FOLLOW - step 1: two beams
===============================

New in this step
    * out of the 1080 beams of the lidar we read only two:
      `b` points straight to the left (90 deg),
      `a` points to the left and forward (45 deg)

What you should see
    One line per second with the two distances, for example

        a (45 deg) = 1.32 m    b (90 deg) = 0.91 m

    The car does not move. Turn it on the spot (RViz: "2D Pose Estimate")
    and watch `b`: it changes although the car is just as far from the wall
    as before. So `b` alone is NOT the distance to the wall.
    Step 2 fixes that.

Run
    ros2 run wall_follow wall1

RViz topics
    (none yet)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


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

    # ============================================================
    #  PIPELINE - runs once for every laser scan
    # ============================================================
    def scan_callback(self, scan):
        a, b = self.read_beams(scan)

        self.get_logger().info(
            f'a ({self.beam_a_angle:.0f} deg) = {a:.2f} m    '
            f'b ({self.beam_b_angle:.0f} deg) = {b:.2f} m',
            throttle_duration_sec=1.0)

    # ============================================================
    #  READ TWO BEAMS                                   <-- NEW
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
