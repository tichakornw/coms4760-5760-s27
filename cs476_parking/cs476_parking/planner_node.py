"""Publish a stored open-loop maneuver as custom Control commands at nominally 20 Hz.

Wait for a valid PoseStamped at the expected start and a matched /cmd subscription.
"""
import math

from cs476_interfaces.msg import Control
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from cs476_parking.interfaces import make_command, planar_pose
from cs476_parking.parking_world import PARKING_WORLD

DT = 0.05            # seconds per command (20 Hz), the sim's step
STOP = (0.0, 0.0)
START_TOL = 0.05     # tolerance per position coordinate (m), and for heading (rad)


def schedule(controls, dt=DT):
    """One (v, delta) per tick of dt for each control's duration, then STOP once."""
    out = [(float(v), float(delta)) for v, delta, t in controls for _ in range(round(t / dt))]
    return out + [STOP]


class Planner(Node):
    def __init__(self):
        super().__init__("planner")
        self.pub = self.create_publisher(Control, "/cmd", 10)
        self.timer = self.create_timer(DT, self.tick)
        self.plan = schedule(PARKING_WORLD.park_maneuver(), DT)
        self.sub = self.create_subscription(PoseStamped, "/pose", self.on_pose, 10)
        self.started = False
        self.sending = False
        self.warned_start = False

    def on_pose(self, msg):
        """Do not replay a fixed plan from a different pose or an incompatible frame."""
        if self.started:
            return
        try:
            x, y, theta = planar_pose(msg)
        except ValueError as exc:
            if not self.warned_start:
                self.get_logger().warning(f"waiting for a valid start pose: {exc}")
                self.warned_start = True
            return
        sx, sy, sth = PARKING_WORLD.start
        heading_error = math.atan2(math.sin(theta - sth), math.cos(theta - sth))
        if max(abs(x - sx), abs(y - sy), abs(heading_error)) > START_TOL:
            if not self.warned_start:
                self.get_logger().warning(f"car at ({x:.2f}, {y:.2f}, {theta:.2f}), "
                                         "waiting for the expected start pose")
                self.warned_start = True
            return
        self.started = True
        self.get_logger().info(f"car at ({x:.2f}, {y:.2f}, {theta:.2f}); "
                               f"sending {len(self.plan) - 1} controls, then a stop")

    def tick(self):
        if not self.started or not self.plan:
            return
        if not self.sending and self.pub.get_subscription_count() == 0:
            return  # wait until the sim has subscribed to /cmd
        self.sending = True
        v, delta = self.plan.pop(0)
        self.pub.publish(make_command(v, delta))
        if not self.plan:
            self.timer.cancel()
            self.get_logger().info("done")


def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
