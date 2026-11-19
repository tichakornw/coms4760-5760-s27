"""planner: grows an RRT with your skeleton.py on a world, publishes the path on /path
(nav_msgs/Path, frame "map") and writes path.json, tree.json and tree.png.

    ros2 run cs476_rrt planner --ros-args -p world:=practice -p seed:=0

Parameters: world (a name, "practice" or "challenge", or the path of a world file), seed
(int), step (float, the step epsilon), goal_bias (float), max_iter (int, the budget of
iterations), out_dir (where the three files go). The first timer tick plans; the path is
then published once a second, so a viewer started later (rviz2) still gets it.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from cs476_rrt import skeleton
from cs476_rrt.world import Tree, draw, load_world, path_length, write_files

DEFAULTS = {"world": "practice", "seed": 0, "step": 0.5, "goal_bias": 0.05,
            "max_iter": 5000, "out_dir": "."}
PERIOD = 1.0     # seconds between ticks: plan on the first, republish the path after


def plan(world, seed=0, step=0.5, goal_bias=0.05, max_iter=5000):
    """RRT (L7): start the tree at the start, then repeat. 1. Sample a random point q_rand
    (the goal itself with probability goal_bias); 2 to 4, skeleton.extend. Stop when a new
    node lands in the goal circle. Returns (tree, index of that node or None, iterations)."""
    rng = np.random.default_rng(seed)
    tree = Tree(world.start)
    for k in range(1, max_iter + 1):
        q_rand = world.goal if rng.random() < goal_bias else world.sample(rng)
        i = skeleton.extend(tree, q_rand, world, step)
        if i is not None and math.dist(tree.nodes[i], world.goal) <= world.goal_radius:
            return tree, i, k
    return tree, None, max_iter


def path_message(points, frame="map"):
    msg = Path()
    msg.header.frame_id = frame
    poses = []
    for x, y in points:
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        poses.append(ps)
    msg.poses = poses
    return msg


class Planner(Node):
    def __init__(self):
        super().__init__("planner")
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.pub = self.create_publisher(Path, "/path", 10)
        self.timer = self.create_timer(PERIOD, self.tick)
        self.tree = None
        self.msg = None

    def tick(self):
        if self.msg is not None:
            self.pub.publish(self.msg)
            return
        p = self.p
        world = load_world(p["world"])
        try:
            tree, goal, iters = plan(world, p["seed"], p["step"], p["goal_bias"], p["max_iter"])
        except NotImplementedError as e:
            self.get_logger().error(f"{e} first, then run the planner again")
            self.timer.cancel()
            return
        self.tree = tree
        path = tree.path_to(goal) if goal is not None else []
        files = write_files(p["out_dir"], tree, path)
        png = files[0].parent / "tree.png"
        draw(world, tree, path, png)
        where = ", ".join(str(f) for f in (*files, png))
        if goal is None:
            self.get_logger().warning(
                f"{world.name}, seed {p['seed']}: no path after {iters} iterations "
                f"({len(tree.nodes)} vertices); try another seed or a larger max_iter. "
                f"Wrote {where}")
            self.timer.cancel()
            return
        self.get_logger().info(
            f"{world.name}, seed {p['seed']}: {len(tree.nodes)} vertices after {iters} "
            f"iterations, a path of {len(path)} points, length {path_length(path):.2f}. "
            f"Wrote {where}")
        self.msg = path_message(path)
        self.pub.publish(self.msg)


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
