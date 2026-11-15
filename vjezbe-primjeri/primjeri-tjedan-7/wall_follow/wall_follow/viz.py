"""
RViz helpers for the wall_follow lessons.

Nothing in this file is part of the wall-follow algorithm. It only turns
numbers into messages that RViz can draw, so the lesson files stay short.

    wall_marker()     -> Marker   (the two beams, the wall, the distance to it)
    steering_marker() -> Marker   (an arrow at the commanded steering angle)

All angles are in radians, measured in the lidar frame:
0 is straight ahead, positive is to the left.
"""
import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

# Markers disappear this long after the node stops publishing them.
LIFETIME = Duration(sec=0, nanosec=500_000_000)

GREY = (0.55, 0.55, 0.55)
ORANGE = (1.00, 0.50, 0.05)
GREEN = (0.10, 0.85, 0.20)
BLUE = (0.12, 0.47, 0.71)

WALL_OVERHANG = 0.5    # [m] how far the wall line is drawn past the two hits
ARROW_LENGTH = 1.0     # [m] length of the steering arrow


def _color(rgb):
    return ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0)


def _point(distance, angle):
    """Polar (distance, angle) in the lidar frame -> x, y point."""
    return Point(
        x=float(distance * math.cos(angle)),
        y=float(distance * math.sin(angle)),
        z=0.0,
    )


def _marker(scan, ns, marker_type):
    """Return a marker in the lidar frame with the common fields filled in."""
    marker = Marker()
    marker.header = scan.header
    marker.ns = ns
    marker.id = 0
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.lifetime = LIFETIME
    return marker


def _all_finite(*numbers):
    return all(math.isfinite(n) for n in numbers)


def wall_marker(scan, a, a_angle, b, b_angle, alpha, distance):
    """
    Draw what the car knows about the wall.

        grey    the two beams `a` and `b`
        orange  the wall: a straight line through the two points they hit
        green   the shortest (perpendicular) line from the lidar to the wall
    """
    marker = _marker(scan, 'wall', Marker.LINE_LIST)
    marker.scale.x = 0.03   # line width
    if not _all_finite(a, b, alpha, distance):
        marker.action = Marker.DELETE   # RViz cannot draw inf or NaN
        return marker

    origin = Point()
    hit_a = _point(a, a_angle)
    hit_b = _point(b, b_angle)

    # make the wall line a bit longer than the piece between the two hits
    dx, dy = hit_a.x - hit_b.x, hit_a.y - hit_b.y
    length = math.hypot(dx, dy)
    if length > 0.0:
        dx, dy = dx / length * WALL_OVERHANG, dy / length * WALL_OVERHANG
    wall_start = Point(x=hit_b.x - dx, y=hit_b.y - dy, z=0.0)
    wall_end = Point(x=hit_a.x + dx, y=hit_a.y + dy, z=0.0)

    # the perpendicular leaves the lidar `alpha` away from beam b
    foot = _point(distance, b_angle + alpha)

    segments = [
        (origin, hit_a, GREY),
        (origin, hit_b, GREY),
        (wall_start, wall_end, ORANGE),
        (origin, foot, GREEN),
    ]
    for start, end, rgb in segments:
        marker.points += [start, end]
        marker.colors += [_color(rgb), _color(rgb)]
    return marker


def steering_marker(scan, steering_angle):
    """Draw an arrow from the lidar that points at the steering angle."""
    marker = _marker(scan, 'steering', Marker.ARROW)
    marker.scale.x = 0.05   # shaft diameter
    marker.scale.y = 0.12   # head diameter
    marker.scale.z = 0.15   # head length
    marker.color = _color(BLUE)
    if not _all_finite(steering_angle):
        marker.action = Marker.DELETE   # RViz cannot draw inf or NaN
        return marker
    marker.points = [Point(), _point(ARROW_LENGTH, steering_angle)]
    return marker
