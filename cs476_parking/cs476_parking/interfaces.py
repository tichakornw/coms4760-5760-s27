"""Message conversions for the parking demo.

/pose is the rear-axle pose in the fixed `map` frame, with a publication timestamp.
/cmd is our Control message: signed speed in m/s and virtual front-wheel angle in radians.
"""
import math

from cs476_interfaces.msg import Control
from geometry_msgs.msg import PoseStamped

FRAME_ID = "map"


def make_command(v, delta):
    return Control(speed=float(v), steering_angle=float(delta))


def command_values(msg, max_steer):
    """Speed and steering angle from a Control message; raises ValueError if either is invalid."""
    v, delta = float(msg.speed), float(msg.steering_angle)
    if not math.isfinite(v) or not math.isfinite(delta):
        raise ValueError("speed and steering_angle must be finite")
    if abs(delta) > max_steer + 1e-6:
        raise ValueError("steering_angle exceeds the demo car's limit")
    return v, delta


def make_pose(pose, stamp):
    x, y, theta = pose
    msg = PoseStamped()
    msg.header.frame_id = FRAME_ID
    msg.header.stamp = stamp
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.z = math.sin(theta / 2)
    msg.pose.orientation.w = math.cos(theta / 2)
    return msg


def planar_pose(msg):
    """x, y and heading from a PoseStamped in the map frame."""
    if msg.header.frame_id != FRAME_ID:
        raise ValueError(f"expected pose in {FRAME_ID!r}, got {msg.header.frame_id!r}")
    p, q = msg.pose.position, msg.pose.orientation
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        raise ValueError("pose fields must be finite")
    if max(abs(p.z), abs(q.x), abs(q.y)) > 1e-6:
        raise ValueError("expected a planar rear-axle pose")
    if abs(q.z * q.z + q.w * q.w - 1.0) > 1e-6:
        raise ValueError("orientation must be a unit quaternion")
    theta = 2 * math.atan2(q.z, q.w)
    return float(p.x), float(p.y), math.atan2(math.sin(theta), math.cos(theta))
