# Before we start, let's define the layout for this file
#
# We are taking on this structure of a monolithic self-contained python file to make everyone's life
# a little easier, since one can simply share this singular file
#
# DOCUMENTATION - Comments explaining the high-level "Design" of this file, further comments will be found later on in CODE
#
# IMPORTS - our library imports
#
# UNMODIFIABLE CONSTANTS - for things like sentinel values, prefixed with CONST_
#
# CONFIGURATION - global CONFIG_ variables
#
# CODE - Implementation
#
#        Throughout CODE, init code will be sprinkled around to initialize and declare global variables.
#        HOWEVER, no actual code will run until the very end of the section.
#

#
#
#
#
#
#
# ---------------------------------- DOCUMENTATION ----------------------------------
#
#
#
#
#
#

# IMPORTANT NOTES
# Global variables with `_` prefixed in their name should NOT
# be modified from other systems. Only modify global variables prefixed with `CONFIG_`.
#
#
#
# ERROR HANDLING
# We will be using a custom `robo_assert` as opposed to the error `raise` paradigm of Python.
# This is because with the `raise` paradigm, we are either relying on the caller to always
# remember to `try...catch`, or are relying on one big `try...catch` in the main outer loop.
#
# This kind of code structure can easily break from a forgotten statement, and such moments
# can lead to undefined behavior within the robot. Thus, we choose to use `robo_assert` as it
# gives us strict determinism, and isolates our code's errors from the robot's errors, as we can
# know that any `raise` would not arise from errors that we created (since we go through `robo_assert`)
#
# `robo_assert` takes in an asserted statement, panic reason, and message, in that order
#
# panic reasons are defined in `class PanicReason`
#
# Printing to the console is done through `print_message`. All `log_` functionality is purely for the logging system
#
#
#
#
#
# CODE STYLE
# Because python is duck typed by default (https://en.wikipedia.org/wiki/Duck_typing),
# please give GLOBAL variables and function parameters explicit types when declared.
#
# Return types only have to be annotated if they are NOT `None`
#
#
#
#
#
# REST OF DOCS
#
# Our primary goal of this is to find a way to construct arbitrary paths between points
# on a weighted grid. The grid size is defined by CONFIG_MAP_WIDTH and CONFIG_MAP_HEIGHT.
#
# Grid coordinates are structured as follows:
#
#
# +y
#
# |
# |
# |
# |
# |
# |
# 0 ---------------- +x
#
# The bottom left is the origin, and it increases in the X direction going right, and the Y direction going up.
#
# NOTE: Gridlines are formed along the lines of tiles, NOT their centers. This means that the walls adjacent to
# the origin are the X and Y axes.
#
# We will use the A* pathfinding algorithm, as this is weighted (if it wasn't weighted, we could probably do Jump Point Search)
#
# There are two main classes: `class Robot` and `class Path`. The `Robot` class is used to abstract over our Brain and Drivetrain
# There are more internal classes, but don't worry about those as much, as those are not as core to the functionality
#

#
#
#
#
#
#
# ---------------------------------- IMPORTS ----------------------------------
#
#
#
#
#
#

import sys
import heapq
from typing import Dict, Optional, List, Tuple, Callable  # we have type safety at home
from enum import IntEnum

#
#
#
#
#
#
# ---------------------------------- CONSTANTS ----------------------------------
#
#
#
#
#
#


CONST_WEIGHT_UNTRAVERESABLE = -1

#
#
#
#
#
#
# ---------------------------------- CONFIG ----------------------------------
#
#
#
#
#
#

CONFIG_DEBUG: bool = (
    True  # Defines if we are in a debug build. More logging functionality will be enabled if we are, but this
)

# May come at the cost of slowing down the program due to the instrumentation.

CONFIG_MAP_WIDTH: int = 7  # Map width in units.
CONFIG_MAP_HEIGHT: int = 7  # Map height in units.

CONFIG_MAP_TILE_WIDTH_INCHES: int = 12  # TODO: measure these
CONFIG_MAP_TILE_HEIGHT_INCHES: int = 12

# TODO: Figure out what the robot's memory usage limitations are

# These two variables exist to limit the size of the map.
# The limits exist because internally we construct a large 2D array of `Tile`s to represent the map.
# Computers have limited memory, so eventually it will OOM,
# and these just exist to make sure we can safely exit before that happens
CONFIG_CONST_MAX_WIDTH: int = 128
CONFIG_CONST_MAX_HEIGHT: int = 128

# This is an array of arrays of (weight, x, y). We parse this one time at init to label the obstacles on our map.
CONFIG_MAP_OBSTACLES: List[List] = [
    [CONST_WEIGHT_UNTRAVERESABLE, 3, 1],
    [CONST_WEIGHT_UNTRAVERESABLE, 2, 6],
    [CONST_WEIGHT_UNTRAVERESABLE, 3, 3],
    [CONST_WEIGHT_UNTRAVERESABLE, 5, 2],
]  # Houses A, B, C, and D respectively

# TODO: more configs for robot/arm speed and whatnot as they come about


#
#
#
#
#
#
# ---------------------------------- CODE ----------------------------------
#
#
#
#
#
#

#
#
# --------- ERROR HANDLING AND INIT CODE ---------
#
#


# Class definitions for error handling and init code
class InitStage(IntEnum):
    INIT_CODE = 0  # Default init stage, only code is being set up
    INIT_ROBOT = 1  # Initializing the robot to a known state
    INIT_RUNNING = 2  # All systems go! (hopefully)


class PanicPhase(IntEnum):
    PANIC_STOP_MOTION = 10  # Stop anything that might be running constantly
    PANIC_SNAPSHOT_STATE = 20  # In case we need to know the current positions of things
    PANIC_USER_DEBUG = 30  # Start the unwind and potentially enter a debug shell
    PANIC_HALT = 40  # bye bye, basically the program halt


class PanicReason(IntEnum):
    PANIC_UNKNOWN = 0

    PANIC_MAP_OOB = 10
    PANIC_MAP_CORRUPT = 11

    PANIC_PATH_INVALID = 20
    PANIC_PATH_NO_NEIGHBOR = 21

    PANIC_ROBOT_ILLEGAL_MOVE = 30
    PANIC_ROBOT_STATE_INVALID = 31

    PANIC_COORD_INVALID = 40

    PANIC_INTERNAL_INVARIANT = 100


class PanicCallback:
    def __init__(
        self,
        name: str,
        phase: PanicPhase,
        fn: Callable[[], None],
        priority: int = 0,
    ):
        self.name = name
        self.phase = phase
        self.priority = priority
        self.fn = fn


_panic_callbacks: list[PanicCallback] = []

# Init stage
_init_stage = InitStage.INIT_CODE


def init_advance(stage: InitStage):
    global _init_stage
    _init_stage = stage


def get_time() -> int:
    return 0  # TODO


def print_message(msg: str = ""):
    print(msg)


# This will terminate the program, it is UNSAFE to call from arbitrary points, and should only be invoked
# from within `panic` unless something truly awful has happened and we cannot recover anything (unlikely)
def halt_and_catch_fire():
    pass  # TODO: shutdown, lol


def panic(reason: str):
    print_message(f'panic "{reason}"')

    callbacks = sorted(
        _panic_callbacks,
        key=lambda c: (c.phase, -c.priority),
    )

    for cb in callbacks:
        try:
            if CONFIG_DEBUG:
                print_message(f"panic: {cb.phase.name}:{cb.name}")
            cb.fn()
        except Exception as e:
            print_message(f"panic callback {cb.name} failed: {e}")

    halt_and_catch_fire()


def panic_callback_register(cb: PanicCallback):
    _panic_callbacks.append(cb)


# USE THIS to assert conditions
def robo_assert(condition: bool, reason: PanicReason, msg: str = ""):
    if condition:
        return

    panic(f"{reason.name}: {msg}")


# this only exists in case something goes VERY VERY WRONG TODO implement
def global_exception_handler(exc_type, exc, tb):
    pass


sys.excepthook = global_exception_handler

#
#
# --------- MAIN LOGIC ---------
#
#


# Types
Coord = Tuple[int, int]  # x, y
TileWeight = int
Direction = Tuple[int, int]
NORTH: Direction = (0, 1)
SOUTH: Direction = (0, -1)
EAST: Direction = (1, 0)
WEST: Direction = (-1, 0)


class LogType(IntEnum):
    LOG_TRACE = 0  # Trace events
    LOG_DEBUG = 1  # Debugging messages
    LOG_WARN = 2  # Warnings
    LOG_ERR = 3  # Fatal errors (these do NOT panic, however)


class Log:
    def __init__(self, log_type: LogType, message: str = ""):
        self.log_type = log_type
        self.message = message
        self.time = get_time()


_logs: List[Log] = []


def log_event(log_type: LogType, message: str = ""):
    _logs.append(Log(log_type, message))


# A "Tile" structure, this has a weight, which we define with __slots__
# because there will be a LOT of these, and this is the only thing that a tile has.
class Tile:
    __slots__ = ("weight",)

    def __init__(self, weight: TileWeight):
        self.weight: TileWeight = weight

    # Negative weights are completely untraversable
    @property
    def traversable(self) -> bool:
        return self.weight >= 0

    @property
    def cost(self) -> int:
        return self.weight


# A "GridMap" structure to represent all the Tiles of the map
class GridMap:
    def __init__(self, width: int, height: int):
        robo_assert(
            width <= CONFIG_CONST_MAX_WIDTH and height <= CONFIG_CONST_MAX_HEIGHT,
            PanicReason.PANIC_MAP_OOB,
            "map size exceeds hard limits",
        )
        self.width: int = width
        self.height: int = height

        # indexed as [y][x], or [row][col]
        self.tiles: List[List[Tile]] = [
            [Tile(1) for _ in range(width + 1)] for _ in range(height + 1)
        ]

    def in_bounds(self, coord: Coord) -> bool:
        x, y = coord
        return 0 <= x <= self.width and 0 <= y <= self.height

    def tile(self, coord: Coord) -> Tile:
        x, y = coord
        return self.tiles[y][x]

    def neighbors(self, coord: Coord) -> List[Coord]:
        x, y = coord
        candidates = [
            (x + 1, y),  # eastern one
            (x - 1, y),  # western one
            (x, y + 1),  # northern one
            (x, y - 1),  # southern one
        ]

        result: List[Coord] = []
        for c in candidates:
            if self.in_bounds(c) and self.tile(c).traversable:
                result.append(c)

        return result


# No need to use __slots__ here, since these structures aren't as frequently
# instantiated, and we might end up adding extra properties later on
class Path:
    def __init__(self, points: List[Coord]):
        robo_assert(
            len(points) != 0, PanicReason.PANIC_PATH_INVALID, "path must have points"
        )
        self.points: List[Coord] = points

    @property
    def start(self) -> Coord:
        return self.points[0]

    @property
    def end(self) -> Coord:
        return self.points[-1]

    def __len__(self) -> int:
        return len(self.points)

    def __iter__(self):
        return iter(self.points)


class Robot:
    def __init__(self, start: Coord):
        self.position: Coord = start
        self.heading_deg: float = 0.0

    def can_step_to(self, target: Coord) -> bool:
        x1, y1 = self.position
        x2, y2 = target
        return abs(x1 - x2) + abs(y1 - y2) == 1

    def follow_path(self, path: Path):
        robo_assert(
            path.start == self.position,
            PanicReason.PANIC_PATH_INVALID,
            f"path start {path.start} does not match robot position {self.position}",
        )

        for next_coord in path.points[1:]:
            robo_assert(
                self.can_step_to(next_coord),
                PanicReason.PANIC_PATH_INVALID,
                f"illegal move from {self.position} to {next_coord}",
            )

            self._move_to(next_coord)

    def _move_to(self, coord: Coord):
        if CONFIG_DEBUG:
            print(f"moving from {self.position} to {coord}")

        # TODO: Drivetrain
        self.position = coord


def reconstruct_path(came_from: dict[Coord, Coord], current: Coord) -> List[Coord]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# manhattan distance as we restrict ourselves to horizontal and vertical
def heuristic(a: Coord, b: Coord) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) # imagine having an algorithm named after your bad urban planning


def build_map_from_config() -> GridMap:
    grid = GridMap(CONFIG_MAP_WIDTH, CONFIG_MAP_HEIGHT)

    for entry in CONFIG_MAP_OBSTACLES:
        weight, x, y = entry
        coord: Coord = (x, y)

        robo_assert(
            grid.in_bounds(coord),
            PanicReason.PANIC_MAP_CORRUPT,
            f"obstacle out of bounds: {coord}",
        )

        grid.tile(coord).weight = weight

    return grid


# We're using an A* algorithm here to pathfind, however we have very strong bias
# towards avoiding turning and prefer direct straight lines for error minimization

# As an effect of this, this algorithm often behaves identically to a much simpler, more
# naive "drive forward to the right row, turn right, drive to right column", especially
# on smaller (e.g. 6x6) maps that have smaller turn costs.

# However, we will preserve the use of A* in case we want to experiment with treating
# smaller units as "tiles" (e.g. one virtual tile is not a physical tile on the ground)


# This algorithm can also be unbiased to test different degrees of turn costs to time efficiency
def astar_with_directions(
    grid: GridMap,
    start: Coord,
    goal: Coord,
    turn_cost: int = 5,
    start_dir: Optional[Direction] = None,
    goal_dir: Optional[Direction] = None,
) -> Optional[Path]:
    robo_assert(
        grid.in_bounds(start) and grid.in_bounds(goal),
        PanicReason.PANIC_COORD_INVALID,
        "start or goal out of bounds",
    )

    if not grid.tile(goal).traversable:
        grid.tile(goal).weight = (
            0  # NOTE: We will allow going to untraversable locations because houses are
            # untraversable along the way but can be a destination
        )

    if not grid.tile(start).traversable or not grid.tile(goal).traversable:
        return None

    # (f_score, Coord)
    open_heap: List[Tuple[int, Coord]] = []
    heapq.heappush(open_heap, (0, start))

    # path reconstruction
    came_from: Dict[Coord, Coord] = {} # What tile did we come from?
    came_from_dir: Dict[Coord, Direction] = (
        {}
    )  # directions we came from to reach this node
    
    if start_dir is not None:
        came_from_dir[start] = start_dir # set our starting direction if one exists

    g_score: Dict[Coord, int] = {start: 0} # Cost to get to the current node

    def get_direction(a: Coord, b: Coord) -> Direction:
        return (b[0] - a[0], b[1] - a[1])

    while open_heap:
        _, current = heapq.heappop(open_heap) # `current` is tile with lowest estimated cost

        if current == goal:
            path = Path(reconstruct_path(came_from, current))

            if goal_dir is not None and len(path.points) >= 2:
                final_dir = get_direction(path.points[-2], path.points[-1])
                robo_assert(
                    final_dir == goal_dir,
                    PanicReason.PANIC_PATH_INVALID,
                    "goal orientation unsatisfiable", # face the required direction
                )

            return path

        # check N, S, E, W neighbors from us
        for neighbor in grid.neighbors(current):
            tentative_g = g_score[current] + grid.tile(neighbor).cost
            move_dir = get_direction(current, neighbor)
        
            prev_dir = came_from_dir.get(current)

            # this would require a turn, so we bias it away
            if prev_dir is not None and move_dir != prev_dir:
                tentative_g += turn_cost
        
            # if this neighbor is unvisited OR if we have found a cheaper path than before,
            # update things and continue from the big outer loop
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                came_from_dir[neighbor] = move_dir
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal) # heuristic is estimated cost from here to goal
                heapq.heappush(open_heap, (f_score, neighbor))
        

    robo_assert(False, PanicReason.PANIC_PATH_INVALID, "no path found")
    return None
