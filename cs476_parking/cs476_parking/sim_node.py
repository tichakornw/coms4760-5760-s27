"""Integrate one bicycle step per valid custom Control command, publish PoseStamped."""
from cs476_interfaces.msg import Control
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from cs476_parking.interfaces import command_values, make_pose
from cs476_parking.parking_world import PARKING_WORLD

DT = 0.05  # seconds per command and per published pose (20 Hz)


class Sim(Node):
    def __init__(self):
        super().__init__("sim")
        self.world = PARKING_WORLD
        self.pose = self.world.start
        self.collided = False
        self.pub = self.create_publisher(PoseStamped, "/pose", 10)
        self.sub = self.create_subscription(Control, "/cmd", self.on_cmd, 10)
        self.timer = self.create_timer(DT, self.tick)

    def on_cmd(self, msg):
        try:
            v, delta = command_values(msg, self.world.car.max_steer)
        except ValueError as exc:
            self.get_logger().warning(f"ignoring invalid command: {exc}")
            return
        self.pose = self.world.car.step(self.pose, v, delta, DT)
        if not self.collided and self.world.collides(self.pose):
            self.collided = True
            x, y, _ = self.pose
            self.get_logger().warning(f"the car collided at ({x:.2f}, {y:.2f})")

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        self.pub.publish(make_pose(self.pose, stamp))


def main(args=None):
    rclpy.init(args=args)
    node = Sim()
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
