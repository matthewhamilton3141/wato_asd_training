# WATonomous ASD Admissions Assignment

## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.

> Why docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. This assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and MacOS. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. You're all set! You can begin the assignment by visiting the WATonomous Wiki.

Link to Onboarding Assignment: https://wiki.watonomous.ca/

---

## Solution

Four ROS 2 (Humble, C++17) nodes under `src/robot/` turn lidar scans into
point-to-point navigation. Each package keeps the ROS plumbing in `*_node.cpp`
and the algorithm in `*_core.cpp` so the core can be unit-tested without ROS.

```
/lidar ──► costmap ──► /costmap ──► map_memory ──► /map ──► planner ──► /path ──► control ──► /cmd_vel
                                        ▲                     ▲                    ▲
                                  /odom/filtered        /odom/filtered       /odom/filtered
                                                        /goal_point
```

| Node | In | Out | What it does |
|---|---|---|---|
| `costmap` | `/lidar` (LaserScan) | `/costmap` (OccupancyGrid, lidar frame) | 40×40 m grid at 0.1 m centred on the robot. Bresenham ray-traces free space (0), marks hits (100), leaves untouched cells unknown (−1), then inflates obstacles with `100·(1 − d/r)` out to 2 m. |
| `map_memory` | `/costmap`, `/odom/filtered` | `/map` (OccupancyGrid, `sim_world`, latched) | Fixed 40×40 m global grid. Every 1.5 m of travel it fuses the latest costmap by walking each global cell and sampling the rotated costmap (no gaps), merging with `max()` since the world is static. Costmaps are paired with the odom sample nearest their timestamp so turning doesn't smear walls. |
| `planner` | `/map`, `/goal_point`, `/odom/filtered` | `/path` (Path) | 8-connected A* with a cost penalty for inflated cells (paths prefer open space) and no corner cutting. Start/goal inside an obstacle are snapped to the nearest free cell. Replans on every map update and every 5 s; clears the path when within 0.5 m of the goal. |
| `control` | `/path`, `/odom/filtered` | `/cmd_vel` (Twist, 10 Hz) | Pure pursuit with a 1.5 m lookahead at 0.5 m/s. Spins in place when the lookahead point is more than ~70° off heading, slows into the goal, stops within 0.2 m. |

All tunables live in each package's `config/params.yaml`.

### Running it

```bash
# once: point watod at the modules you want (already in watod-config.local.sh)
#   ACTIVE_MODULES="vis_tools gazebo robot"
./watod build
./watod up
```

Open Foxglove, connect to `ws://localhost:<FOXGLOVE_BRIDGE_PORT>` (printed by
`./watod up`; also in `modules/.env`), import
`config/wato_asd_training_foxglove_config .json`, and click a point in the 3D
panel (publish → point on `/goal_point`). The robot plans and drives there.

After editing a node: `./watod build robot && ./watod up robot`.

> The base Docker image ships an expired ROS apt signing key; the Dockerfiles
> refresh it from rosdistro before installing anything.
