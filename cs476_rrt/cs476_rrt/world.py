"""Provided code: the world, the tree, the output files and the plot. You do not need to
change anything here; skeleton.py is where you write.

A world is a JSON file (worlds/practice.json, worlds/challenge.json): the bounds
[xmin, ymin, xmax, ymax], the obstacles as polygons (lists of [x, y] vertices), the start,
the goal and the goal radius. Points are (x, y) tuples of floats.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import shapely
from shapely import LineString, Point as ShapelyPoint, Polygon

PACKAGE = "cs476_rrt"


class World:
    def __init__(self, data: dict, name: str = ""):
        self.name = data.get("name", name)
        self.bounds = tuple(float(b) for b in data["bounds"])
        self.obstacles_coords = [[(float(x), float(y)) for x, y in o] for o in data["obstacles"]]
        self.obstacles = [Polygon(o) for o in self.obstacles_coords]
        self.start = tuple(float(c) for c in data["start"])
        self.goal = tuple(float(c) for c in data["goal"])
        self.goal_radius = float(data["goal_radius"])

    def is_free(self, p) -> bool:
        """True if the point p is inside the bounds and touches no obstacle."""
        xmin, ymin, xmax, ymax = self.bounds
        if not (xmin <= p[0] <= xmax and ymin <= p[1] <= ymax):
            return False
        pt = ShapelyPoint(p)
        return not any(o.intersects(pt) for o in self.obstacles)

    def segment_free(self, a, b) -> bool:
        """True if the straight segment from a to b is free: the collision check."""
        if not (self.is_free(a) and self.is_free(b)):
            return False
        seg = LineString([a, b])
        return all(shapely.disjoint(seg, o) for o in self.obstacles)

    def sample(self, rng):
        """A uniformly random point in the bounds (x first, then y)."""
        xmin, ymin, xmax, ymax = self.bounds
        return (float(rng.uniform(xmin, xmax)), float(rng.uniform(ymin, ymax)))

    def in_goal(self, p) -> bool:
        return math.dist(p, self.goal) <= self.goal_radius


SOURCE_WORLDS = Path(__file__).resolve().parent.parent / "worlds"


def worlds_dirs() -> list[Path]:
    """Where named worlds are looked for, in order: the installed share directory, the
    package's source tree (a world file added after `colcon build`, such as the challenge
    handed out in class, is there but not in share/), the current directory and its
    worlds/ folder."""
    dirs = []
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(Path(get_package_share_directory(PACKAGE)) / "worlds")
    except Exception:  # noqa: BLE001 - not installed, or no ROS: skip the share directory
        pass
    dirs += [SOURCE_WORLDS, Path.cwd(), Path.cwd() / "worlds"]
    return dirs


def load_world(name_or_path: str) -> World:
    """A world by the path of its JSON file, or by name ("practice", "challenge"), looked
    for as <name>.json in worlds_dirs()."""
    p = Path(name_or_path).expanduser()
    if p.suffix and p.is_file():
        return World(json.loads(p.read_text()), name=p.stem)
    stem = p.stem if p.suffix == ".json" else name_or_path
    tried = [d / f"{stem}.json" for d in worlds_dirs()]
    for f in tried:
        if f.is_file():
            return World(json.loads(f.read_text()), name=f.stem)
    raise FileNotFoundError(f"no world {name_or_path!r}; looked for "
                            + ", ".join(str(f) for f in tried)
                            + ". Pass the file's path: -p world:=/path/to/world.json")


class Tree:
    """The tree: nodes[i] is a point, parents[i] the index of its parent (-1 for the start)."""

    def __init__(self, start):
        self.nodes = [tuple(start)]
        self.parents = [-1]

    def add(self, q, parent: int) -> int:
        """Add the node q with the edge from nodes[parent]; return the new node's index."""
        self.nodes.append(tuple(q))
        self.parents.append(int(parent))
        return len(self.nodes) - 1

    def path_to(self, i: int) -> list:
        """The points from the start to node i, following parents back."""
        out = []
        while i != -1:
            out.append(self.nodes[i])
            i = self.parents[i]
        return out[::-1]


def path_length(points) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def write_files(out_dir, tree: Tree, path: list) -> list[Path]:
    """path.json (a list of [x, y]) and tree.json ({"nodes", "parents"}) in out_dir: what
    you submit."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "path.json").write_text(json.dumps([list(p) for p in path]))
    (out / "tree.json").write_text(json.dumps({"nodes": [list(p) for p in tree.nodes],
                                               "parents": tree.parents}))
    return [out / "path.json", out / "tree.json"]


def draw(world: World, tree: Tree, path: list, file) -> None:
    """The world, the tree (thin blue) and the path (thick orange) into a PNG file."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon as MplPolygon

    fig, ax = plt.subplots(figsize=(6, 6))
    xmin, ymin, xmax, ymax = world.bounds
    ax.set_xlim(xmin - 0.1, xmax + 0.1)
    ax.set_ylim(ymin - 0.1, ymax + 0.1)
    ax.set_aspect("equal")
    ax.add_patch(MplPolygon([(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)],
                            closed=True, fill=False, edgecolor="#1d1f24", lw=2))
    for o in world.obstacles_coords:
        ax.add_patch(MplPolygon(o, closed=True, facecolor="#BBBBBB", edgecolor="#1d1f24"))
    ax.add_patch(Circle(world.goal, world.goal_radius, fill=False, edgecolor="#009E73", lw=3))
    for i, p in enumerate(tree.parents):
        if p >= 0:
            (x0, y0), (x1, y1) = tree.nodes[p], tree.nodes[i]
            ax.plot([x0, x1], [y0, y1], color="#0072B2", lw=0.8)
    if path:
        ax.plot(*zip(*path), color="#E69F00", lw=3)
    ax.plot(*world.start, "o", color="#1d1f24", ms=8)
    ax.set_title(f"{world.name}: {len(tree.nodes)} vertices"
                 + (f", path length {path_length(path):.2f}" if path else ", no path"))
    fig.savefig(file, dpi=100)
    plt.close(fig)
