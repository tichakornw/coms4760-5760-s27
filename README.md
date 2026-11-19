# Course repository for COM S 4760/5760 - Motion Planning for Robotics and Autonomous Systems, Spring 2027

This repository holds the code we use in lecture. It has three ROS 2 packages, all for
ROS 2 Jazzy on Ubuntu 24.04:

- **The parking demo** (`cs476_interfaces` and `cs476_parking`). A bicycle-model car
  reverses into a parking gap. Three nodes, each its own process, share the work: `sim`
  moves the car, `planner` sends the driving commands and `viewer` draws the street. They
  communicate through two topics, `/cmd` and `/pose`; `/cmd` uses our own message,
  `Control`, defined in `cs476_interfaces`.
- **The RRT planner** (`cs476_rrt`). A node that grows a rapidly exploring random tree
  (RRT) on a 2D world, publishes the path it finds and writes it to files. You write three
  functions of the RRT in `cs476_rrt/skeleton.py`; everything else is provided.

Set up ROS 2 once (the first section below), then follow the section for each package.

```text
cs476_interfaces/   the parking demo's custom message
cs476_parking/      the parking demo's three nodes and launch file
cs476_rrt/          the RRT planner node, the skeleton you complete and a practice world
```

## Set up ROS 2 Jazzy on Ubuntu 24.04

Use an Ubuntu 24.04 desktop session with a graphical display. On Windows or macOS, first set
up an Ubuntu environment with working graphics (for example, an Ubuntu virtual machine); the
commands below do not configure display forwarding. Use a fresh terminal with Ubuntu's
system Python, not Conda or another virtual environment.

These steps follow the official
[Jazzy Ubuntu deb installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
If Jazzy is already installed, skip to step 3 and install any missing packages.

1. Check the OS and enable a UTF-8 locale:

   ```bash
   cat /etc/os-release
   locale
   sudo apt update
   sudo apt install -y locales software-properties-common curl python3
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   sudo add-apt-repository -y universe
   ```

2. Install the official ROS repository configuration for Ubuntu Noble (24.04):

   ```bash
   ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])')
   curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
   sudo dpkg -i /tmp/ros2-apt-source.deb
   sudo apt update
   ```

3. Install ROS, the build tools and the Python libraries the packages use:

   ```bash
   sudo apt install -y ros-jazzy-desktop ros-dev-tools \
       build-essential cmake python3-dev \
       python3-numpy python3-shapely python3-matplotlib python3-tk \
       python3-setuptools python3-wheel
   source /opt/ros/jazzy/setup.bash
   ros2 pkg prefix rclpy
   ros2 interface show geometry_msgs/msg/PoseStamped
   ```

   If apt reports conflicts involving `ros-dev-tools`, see the official guide's note about
   enabling `noble-updates` and `noble-backports`.

## The parking demo

### What is in the two packages

```text
cs476_interfaces/               ament_cmake package: our custom message
├── CMakeLists.txt              generates code for msg/Control.msg
├── package.xml
└── msg/Control.msg             the command message: speed and steering_angle
cs476_parking/                  ament_python package: the three nodes
├── package.xml                 dependencies, including cs476_interfaces
├── setup.py                    installs the package and the launch file, defines the executables
├── setup.cfg
├── resource/cs476_parking      marker file for the ament index
├── launch/parking.launch.py    starts sim, viewer and planner together
└── cs476_parking/
    ├── sim_node.py             the sim node
    ├── planner_node.py         the planner node
    ├── viewer_node.py          the viewer node
    ├── interfaces.py           conversions between messages and plain Python values
    └── parking_world.py        the street, the car model and the stored maneuver
```

### The computation graph

| Node | Subscribes | Publishes | What it does |
|---|---|---|---|
| `sim` | `/cmd` (`cs476_interfaces/msg/Control`) | `/pose` (`geometry_msgs/msg/PoseStamped`), nominally 20 Hz | Advances the car one 0.05 s step per valid command; publishes the pose on its own timer |
| `planner` | `/pose` | `/cmd`, nominally 20 Hz while sending | Waits for the car at the start pose and for a matched subscriber on `/cmd`; sends 131 controls and one stop |
| `viewer` | `/pose` | | Draws the street and the latest pose with matplotlib, nominally 10 redraws per second |

All publishers and subscriptions use keep-last history with depth 10 and the other default
QoS settings. Timer rates are nominal; callback scheduling and message delivery affect
observed rates.

### The messages

`/cmd` uses our own message, `cs476_interfaces/msg/Control`:

```text
# Signed speed in m/s
float32 speed
# Virtual front-wheel angle in rad
float32 steering_angle
```

A negative speed means reverse. A positive steering angle turns the virtual front wheel to
the left. The message has no header, so commands carry no timestamp.

`cs476_interfaces/CMakeLists.txt` finds `rosidl_default_generators` and calls
`rosidl_generate_interfaces` on `msg/Control.msg`. Its `package.xml` declares the generator,
the runtime (`rosidl_default_runtime`) and membership in `rosidl_interface_packages`. When
you build, ROS generates the Python and C++ code for the message; you never write the
`Control` class yourself. In Python:

```python
from cs476_interfaces.msg import Control

msg = Control(speed=-1.0, steering_angle=-0.4)
```

`/pose` uses the standard
[PoseStamped](https://github.com/ros2/common_interfaces/blob/jazzy/geometry_msgs/msg/PoseStamped.msg)
message. It holds the **rear-axle pose in the fixed `map` frame**, stamped with the time it
is published. Position is in metres, with z = 0. The heading theta is encoded as a
quaternion with x = y = 0, z = sin(theta/2) and w = cos(theta/2).

`interfaces.py` holds the conversions, so the nodes deal in plain numbers:

- `make_command(v, delta)` builds a `Control`.
- `command_values(msg, max_steer)` reads one back. It raises `ValueError` if a value is not
  finite or the steering angle is beyond the car's limit.
- `make_pose((x, y, theta), stamp)` builds a `PoseStamped` in the `map` frame.
- `planar_pose(msg)` reads one back as `(x, y, theta)`. It raises `ValueError` for another
  frame, a pose that is not planar, a value that is not finite or a quaternion that is not
  of unit length.

### The nodes

Each node file defines a subclass of `rclpy.node.Node` and a `main()` that calls
`rclpy.init()`, creates the node, calls `rclpy.spin()` until Ctrl-C, then destroys the node
and shuts rclpy down. `setup.py` turns each `main()` into an executable (`sim`, `planner`,
`viewer`) under `lib/cs476_parking`, where `ros2 run` and `ros2 launch` look for it.

**`sim_node.py`.** The `Sim` node starts the car at `PARKING_WORLD.start`. Its `/cmd`
callback, `on_cmd`, checks the command with `command_values` (it logs a warning and ignores
an invalid one) and advances the pose by one step of the kinematic bicycle model,
`Car.step(pose, v, delta, 0.05)`. The model advances by 0.05 s **per received command**,
not by elapsed wall-clock time, so the car moves only when commands arrive; a real car
would keep moving between messages. If the car hits a parked car, the curb or the edge of
the street, the sim logs it once and keeps going; it does not stop the car. A separate
20 Hz timer, `tick`, publishes the current pose whether or not the car is moving.

**`planner_node.py`.** The `Planner` node turns the stored maneuver into a list of commands
with `schedule()`: each segment `(v, delta, duration)` becomes `round(duration / 0.05)`
copies of `(v, delta)`, and a stop `(0, 0)` goes at the end. For the default street that
is 76 commands reversing with the wheels turned right (steering -0.4 rad), 55 with the wheels
turned left (0.5 rad), then the stop. Its `/pose` callback waits until the car is within
0.05 m in each position coordinate and 0.05 rad in heading of the start pose, logging once
if it is not. Its nominal 20 Hz timer then publishes one command per tick, but not before
`/cmd` has a matched subscriber: the default volatile durability does not replay earlier
messages to late subscribers. This check does not identify the subscriber as the sim or
acknowledge command execution. After the stop, the timer is
cancelled and the node logs `done`. The planner never looks at the pose after it starts,
so the maneuver is open loop: if the car were pushed off course, the planner would not
notice.

**`viewer_node.py`.** The `Viewer` node draws the street once (`draw_street`): the street's
outline, the curb, the two parked cars and the goal as a dashed outline. Its `/pose`
callback stores the latest pose (invalid poses are logged and ignored). A 10 Hz timer,
`redraw`, replaces the car's drawing (`draw_car`) and lets matplotlib process window
events, so one thread serves both rclpy and the window. `/pose` carries no steering angle,
so the viewer draws the wheels straight.

**`launch/parking.launch.py`.** `ros2 launch` calls its `generate_launch_description()`,
which returns a `LaunchDescription` with three `Node` actions: sim, viewer and planner,
each started as its own process with its output on the screen.

**`parking_world.py`.** Plain Python, no ROS: the street (15 m by 5 m, curb at y = 0, a
parked car at each end leaving a 6.0 m gap), the car (4.0 m by 1.8 m, wheelbase 2.5 m,
steering limit 0.6 rad), the start (11.5, 3.2, 0), the goal (5.5, 1.0, 0) and the stored
maneuver `PARK_CONTROLS`. `Car.step` is one Euler step of the kinematic bicycle model:

```text
x     += v cos(theta) dt
y     += v sin(theta) dt
theta += (v / wheelbase) tan(delta) dt
```

`ParkingWorld.collides(pose)` checks the car's footprint against the parked cars, the curb
and the street's edges with Shapely. `ParkingWorld.park_maneuver()` returns `PARK_CONTROLS`
for the default street, or searches for a maneuver if you change the street.

### Build and run the demo

From the folder that contains this README (your clone of this repository):

```bash
mkdir -p ~/ros2_ws/src
cp -r cs476_interfaces cs476_parking ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to cs476_parking --symlink-install
source install/setup.bash
ros2 interface show cs476_interfaces/msg/Control
ros2 pkg executables cs476_parking
MPLBACKEND=TkAgg ros2 launch cs476_parking parking.launch.py
```

- Build from the workspace root, `~/ros2_ws`, not from inside `src/`.
- `--packages-up-to cs476_parking` builds the parking package and the packages it depends
  on, so `cs476_interfaces` is built first. `--packages-select cs476_parking` alone would
  skip the interface package on a fresh workspace.
- With `--symlink-install`, edits to existing Python files and the launch file in
  `~/ros2_ws/src` do not need a rebuild; restart the affected nodes or launch command to
  load the changes.
  Edits to `Control.msg`, `package.xml` or `setup.py` still need a rebuild.
- `ros2 interface show` should print `float32 speed` and `float32 steering_angle` with their
  comments.
- `ros2 pkg executables` should list `sim`, `planner` and `viewer`.
- `MPLBACKEND=TkAgg` selects matplotlib's interactive Tk backend for the viewer's window.

A window titled `cs476_parking viewer` opens and the car reverses into the gap. The
maneuver is 6.55 s of commands at 20 Hz, plus startup time. The planner logs that it is
sending 131 controls, then logs `done` after the stop; the other nodes keep running.
Press Ctrl-C in the launch terminal to stop all three processes. Launch again to reset the
simulator and replay the maneuver.

The workspace has its own copies of the packages. If you change the files in this
repository, copy them into `~/ros2_ws/src` again, rebuild and restart the nodes; or edit the
copies in `~/ros2_ws/src` directly.

### Inspect it from another terminal

Source both environments in **each new terminal**:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Then run these one at a time (Ctrl-C stops an echo or a rate monitor):

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /pose --verbose
ros2 topic echo /pose --once
ros2 topic hz /pose
ros2 topic echo /cmd
rqt_graph
```

After parking, `/pose` should be a `PoseStamped` with `frame_id: map`, position near
(5.4, 1.1, 0), and quaternion z near 0.021 and w near 1. Its rate should be around 20 Hz.
`/cmd` uses the generated `Control` type. To observe the entire command sequence, use the
separate-terminal procedure below: start the sim and viewer, start `ros2 topic echo /cmd`
in another sourced terminal, then start the planner. No commands are sent after the stop,
and late subscribers do not receive old messages.

To see the car standing still before the commands begin, use three terminals (source both
environments in each) and start the nodes in this order:

```bash
# Terminal 1
ros2 run cs476_parking sim

# Terminal 2
MPLBACKEND=TkAgg ros2 run cs476_parking viewer

# Terminal 3, when you are ready for the car to park
ros2 run cs476_parking planner
```

Stop and restart both the sim and the planner to replay. Restarting only the planner after
parking leaves it waiting for the car to be back at the start.

### Parking demo troubleshooting

- **`ros2: command not found`:** source `/opt/ros/jazzy/setup.bash` in this terminal.
- **Package not found:** build from `~/ros2_ws`, then source its `install/setup.bash`.
- **Missing `cs476_interfaces`:** copy both packages into `~/ros2_ws/src`, run
  `colcon build --packages-up-to cs476_parking --symlink-install` from `~/ros2_ws`, and
  source `install/setup.bash` again. Do not try to install these packages with pip.
- **Python import errors:** use a fresh shell with Ubuntu's system Python, without Conda or
  another virtual environment, and install the apt packages above.
- **No window, or a Tk error:** use a graphical session with `python3-tk` installed and a
  working `DISPLAY`. SSH, WSL, containers and virtual machines need their own display
  setup. The sim and planner can run without the viewer for topic inspection.
- **The car does not move:** check `ros2 node list`, `ros2 topic info /cmd --verbose` and
  the planner's log. The planner waits for a valid `map` pose at the start and for a
  subscriber on `/cmd`. Restart the whole demo if the car has already parked. Use the same
  `ROS_DOMAIN_ID` in every terminal, and stop old demo runs that publish on the same topics.
- **No `/cmd` output after parking:** expected; use the separate-terminal procedure to
  start the echo after the sim and before the planner.
- **You edited `Control.msg`:** rebuild with `--packages-up-to cs476_parking`, source
  `install/setup.bash` again and restart every node; a running node keeps the message type
  it already imported.

To write a custom message package of your own from scratch, follow the ROS 2
[custom-interface tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).

## The RRT planner

### What is in the package

```text
cs476_rrt/                      ament_python package
├── package.xml
├── setup.py                    installs the package, the launch file and the worlds; defines `planner`
├── setup.cfg
├── resource/cs476_rrt          marker file for the ament index
├── launch/rrt.launch.py        starts the planner with `world` and `seed` arguments
├── worlds/practice.json        the practice world
└── cs476_rrt/
    ├── skeleton.py             the three functions you write
    ├── planner_node.py         the planner node and the RRT loop
    └── world.py                worlds, the collision check, the tree, the output files and the plot
```

### What you write

`cs476_rrt/skeleton.py` has three functions, the steps of one RRT iteration after the
random sample:

| Function | RRT step | What it returns |
|---|---|---|
| `nearest(nodes, q_rand)` | Find the nearest tree node q_near | the index of q_near in `nodes` (the first one if several are equally close) |
| `steer(q_near, q_rand, step)` | Steer one step toward q_rand to get q_new | q_new, a point `(x, y)`: the point at distance `step` from q_near toward q_rand, or q_rand itself if it is closer than `step` |
| `extend(tree, q_rand, world, step)` | All three: nearest, steer, then check the segment from q_near to q_new and, if it is free, add q_new | the new node's index, or `None` if the segment collides (nothing is added) |

You can use `tree.nodes` (the tree's points, a list of `(x, y)`), `tree.add(q, i)` (adds
the point `q` with an edge from `tree.nodes[i]` and returns its index),
`world.segment_free(a, b)` (the collision check: `True` if the straight segment from `a` to
`b` is free) and `math.dist(a, b)`. Until a function is written it raises
`NotImplementedError`, and the planner logs which one.

### The rest of the code

**`planner_node.py`.** `plan()` is the RRT loop: start the tree at the start, then repeat
up to `max_iter` times: sample a random point `q_rand` (the goal itself with probability
`goal_bias`, otherwise a uniformly random point in the world's bounds, from a NumPy
generator seeded with `seed`) and call your `extend`. It stops as soon as a new node lands
in the goal circle. The `Planner` node declares the parameters below. On its first timer
tick it loads the world, runs `plan()`, writes the output files and logs the number of
vertices, the iterations and the path's length. It then publishes the path on `/path` once
a second, so a viewer started later still receives it.

| Parameter | Default | Meaning |
|---|---|---|
| `world` | `practice` | a world's name or the path of a world file |
| `seed` | `0` | the random seed (an integer) |
| `step` | `0.5` | the step length used by `steer` (a float) |
| `goal_bias` | `0.05` | the probability of sampling the goal (a float) |
| `max_iter` | `5000` | the most iterations to run |
| `out_dir` | `.` | the folder for the output files |

| Topic | Type | Contents |
|---|---|---|
| `/path` | `nav_msgs/msg/Path` | the path from the start to the goal, in frame `map`; published only if a path was found |

**`world.py`.** A world is a JSON file: `bounds` `[xmin, ymin, xmax, ymax]`, `obstacles`
(each a polygon, a list of `[x, y]` vertices), `start`, `goal` and `goal_radius`. `World`
checks points and segments against the obstacles and the bounds with Shapely. `Tree`
stores the nodes and each node's parent; `path_to(i)` follows the parents from node `i`
back to the start. `load_world(name)` looks for `<name>.json` in the installed package's
`share/cs476_rrt/worlds`, then in the package's source `worlds/` folder, then in the
current folder and its `worlds/` folder. `write_files` and `draw` write the outputs:

- `path.json`: the path, a list of `[x, y]` points from the start to the goal
- `tree.json`: the tree, `{"nodes": [[x, y], ...], "parents": [...]}`, with parent `-1` for the start
- `tree.png`: a plot of the world, the tree (thin blue) and the path (thick orange)

**`launch/rrt.launch.py`.** Declares two launch arguments, `world` and `seed`, and starts
the planner with them as parameters.

### Build and run the planner

From the folder that contains this README:

```bash
mkdir -p ~/ros2_ws/src
cp -r cs476_rrt ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cs476_rrt --symlink-install
source install/setup.bash
ros2 run cs476_rrt planner --ros-args -p world:=practice -p seed:=0
```

- Write your code in `~/ros2_ws/src/cs476_rrt/cs476_rrt/skeleton.py`, the workspace copy.
  With `--symlink-install`, an edit there needs no rebuild: run the planner again.
- The planner writes `path.json`, `tree.json` and `tree.png` to the folder you run it
  from. Open `tree.png` to see your tree.
- Check yourself: on the practice world with seed 0 and the default parameters, a correct
  `skeleton.py` grows the same tree as the one shown in lecture; compare the number of
  vertices in the log line.
- A float parameter needs a decimal point: `-p step:=1.0`, not `-p step:=1`.

Other ways to run it:

```bash
ros2 run cs476_rrt planner --ros-args -p world:=practice -p seed:=3 -p step:=0.4
ros2 launch cs476_rrt rrt.launch.py world:=practice seed:=3
```

To use another world file, such as one handed out in class, pass its path:
`-p world:=/path/to/world.json`. You can also put it in the folder you run from, or in
`~/ros2_ws/src/cs476_rrt/worlds/`, and pass its name (`-p world:=challenge` for
`challenge.json`).

To see the path, run in another terminal (after sourcing both environments, as for the
parking demo) `ros2 topic echo /path --once`, or start `rviz2`, set the fixed frame to
`map` and add a Path display on `/path`.

### What makes a path valid

The path starts at the start and ends in the goal circle, stays inside the world, and no
segment touches an obstacle. Every point of the path is a vertex of your tree (so a
shortcut between vertices is fine, but a path typed by hand is not). The tree starts at the
start, every node's parents lead back to it, and every edge of the tree is collision-free.

### RRT troubleshooting

- **`skeleton.py: write ... first, then run the planner again`:** the planner reached a
  function you have not written yet; the message names it.
- **`no world ...`:** the log lists every place it looked; pass the file's path with
  `-p world:=/path/to/world.json`.
- **`no path after 5000 iterations`:** try another seed or a larger `max_iter`, and check
  that `extend` adds a node only when the segment is free.
- **A parameter type error:** write floats with a decimal point (`step:=0.4`) and integers
  without one (`seed:=3`).
