"""Every tunable number for the fleet simulator, in one place.

These mirror the parameter tables in docs/DESIGN.md - change both together.
"""

from dataclasses import dataclass

# Belief map cell values. -1 = unknown, like nav_msgs/OccupancyGrid.
UNKNOWN = -1
FREE = 0
OCCUPIED = 1

# Size limits that fix the shape of the policy's input.
MAX_ROBOTS = 6
MAX_FRONTIERS = 8


@dataclass(frozen=True)
class SimConfig:
    """One episode's settings. Training randomizes some of these per episode."""

    # World and robot
    cell: float = 0.2              # m per grid cell
    robot_radius: float = 0.45     # m; planning clearance (the rover is 0.80 x 0.76 m)
    speed: float = 0.4             # m/s along a path
    speed_jitter: float = 0.0      # each robot's speed is multiplied by U(1 - jitter, 1)
    lidar_range: float = 10.0      # m (same as the Gazebo lidar)
    lidar_rays: int = 180
    min_frontier_m: float = 0.6    # shorter frontiers are lidar speckle
    min_unknown_m2: float = 1.0    # ignore frontiers with less unknown area than this nearby

    # Time
    tick: float = 0.5              # s per simulation step
    ticks_per_decision: int = 6    # so a decision every 3 s
    episode_seconds: float = 240.0

    # Communication
    comm_range: float = 5.0        # m; math.inf means perfect comms
    wall_penalty: float = 3.0      # m of range lost per obstacle crossed

    # Fleet
    n_robots: int = 3
    beacons_per_robot: int = 0
    dropout_prob: float = 0.0      # chance that one robot fails at a random time

    # Reward (shared by the whole team)
    team_bonus: float = 0.25       # weight of "discovered by any robot" next to "known at base"
    reward_per_m2: float = 0.1

    @property
    def decision_seconds(self) -> float:
        return self.tick * self.ticks_per_decision

    @property
    def radius_cells(self) -> float:
        """Robot clearance in cells, measured from cell centers.

        The +0.5 accounts for the obstacle cell's own half-width, so the robot
        keeps robot_radius from the obstacle's edge, not its center.
        """
        return self.robot_radius / self.cell + 0.5


@dataclass(frozen=True)
class HouseConfig:
    """Procedural house generator settings (see houses.py)."""

    width_m: tuple = (18.0, 28.0)
    height_m: tuple = (14.0, 22.0)
    min_room_m: float = 3.0        # never split a room below this side length
    max_room_m: float = 6.0        # rooms longer than this are always split (if they can be)
    door_m: float = 1.6
    loop_door_prob: float = 0.3    # extra door between adjacent rooms, so the layout has loops
    max_furniture_per_room: int = 2
    door_clearance_m: float = 1.4  # furniture keeps this far from doors
    spawn_radius_m: float = 1.5    # robots start this close to the base


def sample_training_config(rng) -> SimConfig:
    """The randomized settings each training episode draws (see docs/DESIGN.md)."""
    return SimConfig(n_robots=int(rng.choice([2, 3, 4])),
                     comm_range=float(rng.uniform(3.0, 12.0)),
                     beacons_per_robot=int(rng.integers(0, 3)),
                     speed_jitter=0.25,
                     dropout_prob=0.2)
