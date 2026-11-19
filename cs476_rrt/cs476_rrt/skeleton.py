"""Your RRT: write the three functions below (L7, "Rapidly Exploring Random Trees").

The loop is written for you in planner_node.plan():

    Start the tree at the start
    Repeat until a node lands in the goal circle (or the budget runs out)
        1. Sample a random point q_rand                       (provided)
        2-4. extend(tree, q_rand, world, step)                (yours)

extend does lines 2 to 4 of L7's loop, using nearest and steer:

    2. Find the nearest tree node q_near
    3. Steer one step toward q_rand to get q_new
    4. Check the segment and, if free, add q_new

What you can use:
    tree.nodes            the points of the tree, a list of (x, y)
    tree.add(q, i)        adds the point q with an edge from tree.nodes[i]; returns its index
    world.segment_free(a, b)
                          True if the straight segment from a to b is free (the collision check)
    math.dist(a, b)       the straight-line distance between two points

Check yourself: with seed 0 on the practice world your tree must have the same number of
vertices as the one the lecture shows, and path.json must pass the scorer.
"""
import math  # noqa: F401  (you will need it)


def nearest(nodes, q_rand):
    """Find the nearest tree node q_near.

    nodes: the tree's points, a list of (x, y). q_rand: a point (x, y).
    Return the index i of the point of nodes closest to q_rand in straight-line distance
    (the first such index if several are equally close).
    """
    raise NotImplementedError("skeleton.py: write nearest(nodes, q_rand)")


def steer(q_near, q_rand, step):
    """Steer one step toward q_rand to get q_new.

    Return the point at distance `step` from q_near on the segment toward q_rand, as a
    tuple (x, y); if q_rand is within `step` of q_near, return q_rand itself.
    """
    raise NotImplementedError("skeleton.py: write steer(q_near, q_rand, step)")


def extend(tree, q_rand, world, step):
    """One iteration after the sample: find the nearest tree node q_near (nearest), Steer
    one step toward q_rand to get q_new (steer), then Check the segment and, if free, add
    q_new: if world.segment_free(q_near, q_new), add it with tree.add(q_new, i_near).

    Return the new node's index, or None if the segment collides (nothing is added).
    """
    raise NotImplementedError("skeleton.py: write extend(tree, q_rand, world, step)")
