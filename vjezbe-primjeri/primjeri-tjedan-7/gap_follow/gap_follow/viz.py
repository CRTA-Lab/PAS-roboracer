"""
RViz helpers for the gap_follow lessons.

Nothing in this file is part of the gap-follow algorithm. It only turns
numbers into messages that RViz can draw, so the lesson files stay short.

    scan_msg()       -> LaserScan  (a processed copy of the scan)
    closest_marker() -> Marker     (a red dot on the closest point)
    gaps_marker()    -> Marker     (the free beams, coloured per gap)
    target_marker()  -> Marker     (an arrow from the lidar to the target)
"""
import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

# Markers disappear this long after the node stops publishing them.
LIFETIME = Duration(sec=0, nanosec=500_000_000)

# One colour per gap (used when no gap has been chosen yet).
PALETTE = [
    (0.12, 0.47, 0.71),   # blue
    (1.00, 0.50, 0.05),   # orange
    (0.58, 0.40, 0.74),   # purple
    (0.09, 0.75, 0.81),   # cyan
    (0.89, 0.47, 0.76),   # pink
    (0.74, 0.74, 0.13),   # olive
]
GREEN = (0.10, 0.85, 0.20)
GREY = (0.55, 0.55, 0.55)
RED = (0.90, 0.10, 0.10)


def _color(rgb):
    return ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0)


def _point(distance, angle):
    """Polar (distance, angle) in the lidar frame -> x, y point."""
    return Point(
        x=float(distance * math.cos(angle)),
        y=float(distance * math.sin(angle)),
        z=0.0,
    )


def scan_msg(scan, ranges, angles):
    """Build a LaserScan that shows `ranges` measured at `angles`."""
    out = LaserScan()
    out.header = scan.header
    out.angle_min = float(angles[0])
    out.angle_max = float(angles[-1])
    out.angle_increment = scan.angle_increment
    out.range_min = scan.range_min
    out.range_max = scan.range_max
    out.ranges = [float(r) for r in ranges]
    return out


def closest_marker(scan, distance, angle):
    """Draw a red dot on the closest point."""
    marker = Marker()
    marker.header = scan.header
    marker.ns = 'closest'
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position = _point(distance, angle)
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.15
    marker.scale.y = 0.15
    marker.scale.z = 0.15
    marker.color = _color(RED)
    marker.lifetime = LIFETIME
    return marker


def gaps_marker(scan, ranges, angles, gaps, chosen=None):
    """
    Draw every free beam as a dot at the distance `ranges` gives it.

    Without `chosen`, each gap gets its own colour.
    With `chosen`, that gap is green and all the others are grey.
    """
    marker = Marker()
    marker.header = scan.header
    marker.ns = 'gaps'
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.06
    marker.scale.y = 0.06
    marker.lifetime = LIFETIME

    for n, gap in enumerate(gaps):
        if chosen is None:
            color = _color(PALETTE[n % len(PALETTE)])
        elif gap is chosen:
            color = _color(GREEN)
        else:
            color = _color(GREY)
        for i in gap:
            marker.points.append(_point(ranges[i], angles[i]))
            marker.colors.append(color)
    return marker


def target_marker(scan, distance, angle):
    """Draw an arrow from the lidar to the point the car is aiming at."""
    marker = Marker()
    marker.header = scan.header
    marker.ns = 'target'
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.05   # shaft diameter
    marker.scale.y = 0.12   # head diameter
    marker.scale.z = 0.15   # head length
    marker.color = _color(GREEN)
    marker.lifetime = LIFETIME
    marker.points = [Point(), _point(distance, angle)]
    return marker
