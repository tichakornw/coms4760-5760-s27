"""viewer: draws the street and the car with matplotlib from /pose (PoseStamped),
redrawing REDRAW_HZ times a second. The wheels are drawn straight: /pose carries no steering
angle."""
import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import PoseStamped
from matplotlib.patches import Polygon
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from cs476_parking.interfaces import planar_pose
from cs476_parking.parking_world import PARKING_WORLD

REDRAW_HZ = 10
INK, BLUE, GREEN, GREY = "#1d1f24", "#0072B2", "#009E73", "#BBBBBB"


def draw_street(ax, world=PARKING_WORLD):
    """The street frame, the curb, the two parked cars and the dashed goal outline."""
    xmin, ymin, xmax, ymax = world.bounds
    ax.set_xlim(xmin - 0.1, xmax + 0.1)
    ax.set_ylim(ymin - 0.1, ymax + 0.1)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.add_patch(Polygon([(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)], closed=True,
                         fill=False, edgecolor=INK, lw=2))
    ax.plot([xmin, xmax], [world.curb_y, world.curb_y], "-", color=INK, lw=5)
    for p in world.parked:
        ax.add_patch(Polygon(list(p.exterior.coords)[:-1], closed=True, facecolor=GREY,
                             edgecolor=INK, lw=1.5))
    ax.add_patch(Polygon(list(world.car.footprint(world.goal).exterior.coords)[:-1],
                         closed=True, fill=False, edgecolor=GREEN, ls="--", lw=2))


def draw_car(ax, pose, world=PARKING_WORLD):
    """The car body, its four wheels and the rear-axle dot; returns the artists."""
    car = world.car
    body = ax.add_patch(Polygon(list(car.footprint(pose).exterior.coords)[:-1], closed=True,
                                facecolor="white", edgecolor=BLUE, lw=2.5, zorder=6))
    wheels = [ax.add_patch(Polygon(w, closed=True, facecolor=INK, edgecolor=INK, zorder=7))
              for w in car.wheels(pose)]
    (dot,) = ax.plot(pose[0], pose[1], "o", color=INK, ms=5, zorder=8)
    return [body, *wheels, dot]


class Viewer(Node):
    def __init__(self):
        super().__init__("viewer")
        self.pose = None
        self.car = []
        self.fig, self.ax = plt.subplots(figsize=(9, 3.4))
        self.fig.canvas.manager.set_window_title("cs476_parking viewer")
        draw_street(self.ax)
        self.sub = self.create_subscription(PoseStamped, "/pose", self.on_pose, 10)
        self.timer = self.create_timer(1 / REDRAW_HZ, self.redraw)

    def on_pose(self, msg):
        try:
            self.pose = planar_pose(msg)
        except ValueError as exc:
            self.get_logger().warning(f"ignoring invalid pose: {exc}")

    def redraw(self):
        if self.pose is not None:
            for artist in self.car:
                artist.remove()
            self.car = draw_car(self.ax, self.pose)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = Viewer()
    plt.ion()
    plt.show(block=False)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        plt.close(node.fig)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
