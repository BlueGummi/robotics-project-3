#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math

# Brain should be defined by default
brain = Brain()

# Robot configuration code
brain_inertial = Inertial()
controller = Controller()
left_drive_smart = Motor(Ports.PORT3, False)
right_drive_smart = Motor(Ports.PORT8, True)
drivetrain = SmartDrive(left_drive_smart, right_drive_smart, brain_inertial, 319.19, 320, 40, MM, 1)


# Wait for sensor(s) to fully initialize
wait(100, MSEC)

# generating and setting random seed
def initializeRandomSeed():
    wait(100, MSEC)
    xaxis = brain_inertial.acceleration(XAXIS) * 1000
    yaxis = brain_inertial.acceleration(YAXIS) * 1000
    zaxis = brain_inertial.acceleration(ZAXIS) * 1000
    systemTime = brain.timer.system() * 100
    urandom.seed(int(xaxis + yaxis + zaxis + systemTime)) 

# Initialize random seed 
initializeRandomSeed()

vexcode_initial_drivetrain_calibration_completed = False
def calibrate_drivetrain():
    # Calibrate the Drivetrain Inertial
    global vexcode_initial_drivetrain_calibration_completed
    sleep(200, MSEC)
    brain.screen.print("Calibrating")
    brain.screen.next_row()
    brain.screen.print("Inertial")
    brain_inertial.calibrate()
    while brain_inertial.is_calibrating():
        sleep(25, MSEC)
    vexcode_initial_drivetrain_calibration_completed = True
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)


# Calibrate the Drivetrain
calibrate_drivetrain()

#endregion VEXcode Generated Robot Configuration

#
# Copyright (c) 2026 Team VMPSADBW
# All rights reserved.
#
# This code is licensed under the BSD 3-Clause License.
#

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
# THIS PLATFORM DOES NOT SUPPORT TYPE SAFETY OR THE 'typing' MODULE.
# 
# This version of MicroPython that Vex uses does NOT SUPPORT F STRINGS!!! <-- found this out after much pain
#
# ERROR HANDLING
# We will be using a custom `robo_assert` as opposed to the error `raise` paradigm of Python.
# This is because with the `raise` paradigm, we are either relying on the caller to always
# remember to `try...catch`, or are relying on one big `try...catch` in the main outer loop.
#
# This kind of code structure can easily break from a forgotten statement, and such moments
# can lead to undefined behavior within the robot. See https://en.wikipedia.org/wiki/Ariane_flight_V88 for
# an unfortunate but real case of failure with this kind of error handling paradigm.
# Thus, we choose to use `robo_assert` as it
# gives us strict determinism, and isolates our code's errors from the robot's errors, as we can
# know that any `raise` would not arise from errors that we created, besides language errors
# (since we go through `robo_assert`)
#
# `robo_assert` takes in an asserted statement, panic reason, and message, in that order
#
# panic reasons are defined in `class PanicReason`
#
# Printing to the console is done through `print_message`. All `log_` functionality is purely for the logging system
#
# CODE STYLE
# TODO: write this once you're closer to complete with the codebase
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
# CONTROL PARADIGM:
# 
# The robot starts powered off. After powering on, it needs to be calibrated. As the voice says, press "B" to calibrate
# it. This will reset sensors and motors to their initial states, and the robot's internal heading to North and position
# to "0,0". Once this is done, the robot will enter a menu. Navigating through this menu will allow you to find your
# designated destination.
#
# CONTROL NOTES:
# 
# Vex is weird and doesn't allow us to unregister a callback (???), thus, we model a state machine of the robot, and 
# we make a callback for the functionality of all relevant buttons at each state, and inside the callback we check if 
# the expected state matches the current state and only operate if they do match. 
#

# no library imports for now
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

CONFIG_DEBUG = (
    True  # Defines if we are in a debug build. More logging functionality will be enabled if we are, but this
)         # may come at the cost of slowing down the program due to the instrumentation

CONFIG_SPIN_LATENCY_MS = 5 # Spinning/waiting in a loop - how long should we wait each iteration?

CONFIG_AUDIO_DIR = "" # only if the audio on the microSD is under some directory
CONFIG_MAP_WIDTH = 7  # Map width in units.
CONFIG_MAP_HEIGHT = 7  # Map height in units.

CONFIG_MAP_TILE_WIDTH_INCHES = 24
CONFIG_MAP_TILE_HEIGHT_INCHES = 24

# definition:
#    LOG_TRACE = 0  # Trace events
#    LOG_DEBUG = 1  # Debugging messages
#    LOG_WARN = 2  # Warnings
#    LOG_ERR = 3  # Fatal errors (these do NOT panic, however)

CONFIG_PRINT_LOG_DEPTH = 0 # The deepest log we should still print

# TODO: Figure out what the robot's memory usage limitations are

# These two variables exist to limit the size of the map.
# The limits exist because internally we construct a large 2D array of `Tile`s to represent the map.
# Computers have limited memory, so eventually it will OOM,
# and these just exist to make sure we can safely exit before that happens
CONFIG_CONST_MAX_WIDTH = 128
CONFIG_CONST_MAX_HEIGHT = 128
# https://github.com/cetio/VEXAPI 
# Reverse engineering of Vex internals. In here it mentions the amount of memory for V5 controllers.
# We are EXP, but perhaps we could try and test with V5's limits

CONFIG_FONT = FontType.MONO12 # Please change CONFIG_FONT_ROWS and CONFIG_FONT_COLS according to the
                              # table listed in brain.screen.next_row()'s documentation file!
CONFIG_FONT_ROWS = 9
CONFIG_FONT_COLS = 26
CONFIG_TURN_COST = 5 # Used in A*

# This is an array of arrays of (weight, x, y). We parse this one time at init to label the obstacles on our map.
CONFIG_MAP_OBSTACLES = [
    [CONST_WEIGHT_UNTRAVERESABLE, 3, 1],
    [CONST_WEIGHT_UNTRAVERESABLE, 2, 6],
    [CONST_WEIGHT_UNTRAVERESABLE, 3, 3],
    [CONST_WEIGHT_UNTRAVERESABLE, 5, 2],
]  # Houses A, B, C, and D respectively

# This is an array of arrays of (name: str, audio_file: str, x, y)
CONFIG_MAP_LOCATIONS = [
    ["House A", "house_a.wav", 3, 1],
    ["House B", "house_b.wav", 2, 6],
    ["House C", "house_c.wav", 3, 3],
    ["House D", "house_d.wav", 5, 2]
]

CONFIG_SOUND_VOL = 50 # Range 0-100

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
# Good luck!
# Remember: "Science isn't about WHY. It's about WHY NOT."

#
#
# --------- Handrolling things that this environment doesn't have ---------
#
#

class FakeIntEnum:
    @classmethod
    def name(cls, value):
        for attr, val in cls.__dict__.items():
            if not attr.startswith("_") and not callable(val) and val == value:
                return attr
        return None

class heapq:
    def __init__(self):
        self.data = []

    def heappush(self, item):
        self.data.append(item)
        self.data.sort(key=lambda x: x[0])

    def heappop(self):
        return self.data.pop(0)

    def __bool__(self):
        return len(self.data) > 0

#
#
# --------- ERROR HANDLING, AUDIO, AND INIT CODE ---------
#
#


# Class definitions for error handling and init code
class InitStage(FakeIntEnum):
    INIT_CODE = 0  # Default init stage, only code is being set up
    INIT_ROBOT = 1  # Initializing the robot to a known state
    INIT_RUNNING = 2  # All systems go! (hopefully)

class PanicPhase(FakeIntEnum):
    PANIC_STOP_MOTION = 10  # Stop anything that might be running constantly
    PANIC_SNAPSHOT_STATE = 20  # In case we need to know the current positions of things
    PANIC_USER_DEBUG = 30  # Start the unwind and potentially enter a debug shell
    PANIC_HALT = 40  # bye bye, basically the program halt

class PanicReason(FakeIntEnum):
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
        name,
        phase,
        fn,
        priority = 0,
    ):
        self.name = name
        self.phase = phase
        self.priority = priority
        self.fn = fn

# Init stage
_INIT_STAGE = InitStage.INIT_CODE
def init_advance(stage):
    global _INIT_STAGE
    _INIT_STAGE = stage

def spin_wait():
    wait(CONFIG_SPIN_LATENCY_MS, MSEC)

def play_audio(fname, blocking=True):
    if fname is None:
        return

    if CONFIG_AUDIO_DIR != "":
        raw_path = CONFIG_AUDIO_DIR + "/" + fname
    else:
        raw_path = fname

    if brain.sdcard.exists(raw_path):
        brain.play_file(raw_path, CONFIG_SOUND_VOL)
        while brain.sound_is_active():
            spin_wait()

def speak_number(num):
    def speak_word(word):
        play_audio("num_" + word + ".wav")

    ones = [ # I know this is silly but this used to convert
             # into english words rather than numbers, (e.g. 0 -> "zero")
             # and now I am too lazy to change it again.

             # The reason I changed it to this alongside the file names
             # is because the FAT filesystem was having trouble looking up
             # long file names (as expected), and using numbers instead of
             # words removes this problem
        "",
        "1",
        "2",
        "3",
        "4",
        "5",
        "6",
        "7",
        "8",
        "9",
        "10",
        "11",
        "12",
        "13",
        "14",
        "15",
        "16",
        "17",
        "18",
        "19",
    ]

    tens = [
        "",
        "",
        "20",
        "30",
        "40",
        "50",
        "60",
        "70",
        "80",
        "90",
    ]

    print_message(str(num))
    if num < 0 or num >= 1000:
        return

    if num == 0:
        return speak_word("0")

    if num >= 100:
        one_part = ones[num / 100]
        speak_word(one_part)
        speak_word("100")
        num %= 100
        if num > 0:
            speak_word("and")

    if num >= 20:
        speak_word(tens[num / 10])
        num %= 10
        if num > 0:
            speak_word(ones[num])
    elif num > 0:
        speak_word(ones[num])


def get_time_ms():
    return brain.timer.time(MSEC)

def print_message(msg = ""):
    print(msg)
    robot_render_pos()
    brain.screen.print(msg)
    brain.screen.next_row()

def clear_console():
    brain.screen.clear_screen()
    robot_render_pos()


# This will terminate the program, it is UNSAFE to call from arbitrary points, and should only be invoked
# from within `panic` unless something truly awful has happened and we cannot recover anything (unlikely)
def halt_and_catch_fire():
    brain.program_stop()

_PANIC_CALLBACKS = []
def panic(reason):
    print_message('PANIC "%s"' % (reason))
    if _INIT_STAGE < InitStage.INIT_RUNNING:
        play_audio("initialization_failed.wav")

    callbacks = sorted(
        _PANIC_CALLBACKS,
        key=lambda c: (c.phase, -c.priority),
    )

    for cb in callbacks:
        if CONFIG_DEBUG:
            print_message("panic: %s:%s" % (PanicPhase.name(cb.phase), cb.name))
            
        cb.fn()

    halt_and_catch_fire()


def panic_callback_register(cb):
    _PANIC_CALLBACKS.append(cb)


# USE THIS to assert conditions
def robo_assert(condition, reason, msg = ""):
    if condition:
        return

    play_audio("halt_assertion_failure.wav") # TODO: maybe someone can find a smarter way 
                                             # of doing audio files and not making
                                             # a million CONFIG_*s.... 
    panic("%s: %s" % (PanicReason.name(reason), msg))

#
#
# --------- MAIN LOGIC ---------
#
#


# Types (unused, typing doesn't work)
# Coord = (int, int)  # x, y
# TileWeight = int
# Direction = (int, int)

class LogType(FakeIntEnum):
    LOG_TRACE = 0  # Trace events
    LOG_DEBUG = 1  # Debugging messages
    LOG_WARN = 2  # Warnings
    LOG_ERR = 3  # Fatal errors (these do NOT panic, however)


_LOGS = []
class Log:
    def __init__(self, log_type, message = ""):
        self.log_type = log_type
        self.message = message
        self.timestamp = get_time_ms()

    

def log_event(log_type, message = ""):
    if log_type >= CONFIG_PRINT_LOG_DEPTH:
        print(LogType.name(log_type) + ": " + message)
    _LOGS.append(Log(log_type, message))


# A "Tile" structure, this has a weight, which we define with __slots__
# because there will be a LOT of these, and this is the only thing that a tile has.
class Tile:
    __slots__ = ("weight")

    def __init__(self, weight):
        self.weight = weight

    # Negative weights are completely untraversable
    @property
    def traversable(self):
        return self.weight >= 0

    @property
    def cost(self):
        return self.weight


# A "GridMap" structure to represent all the Tiles of the map
class GridMap:
    def __init__(self, width, height):
        robo_assert(
            width <= CONFIG_CONST_MAX_WIDTH and height <= CONFIG_CONST_MAX_HEIGHT,
            PanicReason.PANIC_MAP_OOB,
            "map size exceeds hard limits",
        )
        self.width = width
        self.height = height

        # indexed as [y][x], or [row][col]
        self.tiles = [
            [Tile(1) for _ in range(width + 1)] for _ in range(height + 1)
        ]

    def in_bounds(self, coord):
        x, y = coord
        return 0 <= x <= self.width and 0 <= y <= self.height

    def tile(self, coord):
        x, y = coord
        return self.tiles[y][x]

    def neighbors(self, coord):
        x, y = coord
        candidates = [
            (x + 1, y),  # eastern one
            (x - 1, y),  # western one
            (x, y + 1),  # northern one
            (x, y - 1),  # southern one
        ]

        result = []
        for c in candidates:
            if self.in_bounds(c) and self.tile(c).traversable:
                result.append(c)

        return result

def print_grid(grid, path=None):
    path_set = set(path) if path else set()

    for y in range(grid.height + 1):
        row = ""
        for x in range(grid.width + 1):
            coord = (x, y)
            tile = grid.tile(coord)
            if coord in path_set:
                row += "P"  # path tile
            elif not tile.traversable:
                row += "B"  # blocked
            elif tile.cost > 1:
                row += "E"  # expensive tile
            else:
                row += "."  # normal tile
        print(row)



_LOCATIONS = [] # List of Locations to add in the menu
class Location:
    def __init__(self, name, audio_file, coordinate):
        self.name = name
        self.audio_file = audio_file
        self.coords = coordinate

    def speak(self):
        play_audio(self.audio_file)

def parse_locations():
    for location in CONFIG_MAP_LOCATIONS:
        name = location[0]
        audio_file = location[1]
        x = location[2]
        y = location[3]
        lclass = Location(name, audio_file, (x, y))    
        _LOCATIONS.append(lclass)

def menu_from_locations():
    return Menu(_LOCATIONS)

# Menu:
#                          
#      ┌───────────────┐   
#      │Display Current│   
#   ┌─▶│  Destination  │◀─┐
#   │  └───────────────┘  │
#   │          │          │
#   │          │          │
#   └───Prev───┼───Next───┘
#              │           
#            Select        
#              │           
#              │           
#              ▼           
#    ┌───────────────────┐ 
#    │Travel to selection│ 
#    └───────────────────┘ 

class MenuState(FakeIntEnum):
    MENU_DISPLAY = 0
    MENU_SELECTED = 1

class Menu:
    def __init__(self, locations):
        robo_assert(len(locations) > 0, PanicReason.PANIC_INTERNAL_INVARIANT, 
                    "Menu must have at least one location")
        self.locations = locations
        self.current_location = locations[0]
        self.current_location_index = 0
        self.number_locations = len(locations)
        self.state = MenuState.MENU_DISPLAY
        self.data_changed = False 

    def present(self):
        if self.data_changed:
            clear_console()
        
            brain.screen.set_cursor(3, 1)
            brain.screen.print("Menu:")
            brain.screen.next_row()
            brain.screen.print("  < " + self.current_location.name + " >")
            brain.screen.next_row()
        
            self.data_changed = False


    # Internal. use scroll_right/left from the outside world
    def _scroll(self, direction):
        if direction:
            advance = 1
        else:
            advance = -1
        
        self.current_location_index = ((self.current_location_index + advance)
                                        % self.number_locations)
        self.current_location = self.locations[self.current_location_index]
        self.data_changed = True

    def scroll_right(self):
        self._scroll(True)
    
    def scroll_left(self):
        self._scroll(False)
    
    def select(self):
        self.state = MenuState.MENU_SELECTED

    def enter(self, should_play_sound=True):
        ROBOT.change_state(RobotState.ROBOT_IN_MENU)
        log_event(LogType.LOG_TRACE, "Entering menu")
        self.data_changed = True 
        controller.buttonDown.pressed(self.scroll_left)
        controller.buttonUp.pressed(self.scroll_right)
        controller.buttonA.pressed(self.select)
        controller.buttonB.pressed(self.select)
        if should_play_sound:
            play_audio("menu_enter.wav")

        while self.state != MenuState.MENU_SELECTED:
            self.present()
            spin_wait()

        self.state = MenuState.MENU_DISPLAY
        play_audio("destination_set.wav")
        play_audio(self.current_location.audio_file)
        log_event(LogType.LOG_TRACE, "Destination set to " + self.current_location.name)
        clear_console()
        return self.current_location


# No need to use __slots__ here, since these structures aren't as frequently
# instantiated, and we might end up adding extra properties later on
class Path:
    def __init__(self, points):
        robo_assert(
            len(points) != 0, PanicReason.PANIC_PATH_INVALID, "path must have points"
        )
        self.points = points

    @property
    def start(self):
        return self.points[0]

    @property
    def end(self):
        return self.points[-1]

    def __len__(self):
        return len(self.points)

    def __iter__(self):
        return iter(self.points)


# ---------------------- ROBOT STATE MACHINE ----------------------
#                         ┌───────────────┐                           
#                         │               │                           
#             ┌─INIT FAIL─│    STOPPED    │──INIT OK──┐               
#             │           │               │           │               
#             │           └───────────────┘           │               
#             │                   ▲                   │               
#             ▼                   │                   ▼               
#   ┌───────────────────┐         │         ┌───────────────────┐     
#   │                   │         │         │                   │     
#   │    FAILED INIT    │         │         │    INITIALIZED    │     
#   │                   │         │         │                   │     
#   └───────────────────┘         │         └───────────────────┘     
#             │                   │                   │               
#             │                   │               ENTER MENU          
#             └──RETURN BACK TO───┘                   ▼               
#                                           ┌──────────────────┐      
#                                           │                  │      
#             ┌──RE-ENTER MENU──┬──────────▶│  SELECTION MENU  │◀─┐   
#             │                 │           │                  │  │   
#   ┌───────────────────┐       │           └──────────────────┘  │   
#   │                   │       │                     │           │   
#   │  DELIVER PACKAGE  │   PATH FAIL                 │           │   
#   │                   │       │                     ├────STAY───┘   
#   └───────────────────┘       │                     │               
#             ▲                 │                     │               
#             │          ┌────────────┐               │               
#             │          │            │               │               
#             └─PATH OK──│   TRAVEL   │◀───SELECTED───┘               
#                        │            │                               
#                        └────────────┘                               
#                                                                     
#                                                                     
#                                ◎                                    
#                                ╳                                    
#                           STOP SIGNAL                               
#                         (can arrive at                              
#                            any time)                                
#                                ╳                                    
#                                ▼                                    
#                        ┌───────────────┐                            
#                        │               │                            
#                        │    STOPPED    │                            
#                        │               │                            
#                        └───────────────┘                            

class RobotState(FakeIntEnum):
    ROBOT_FAILED_INIT = -1
    ROBOT_STOPPED = 0
    ROBOT_INITIALIZED = 1
    ROBOT_IN_MENU = 2
    ROBOT_TRAVELLING = 3
    ROBOT_DELIVERING = 4

class Robot:
    def __init__(self, start):
        self.position = start
        self.state = RobotState.ROBOT_STOPPED
    
    def change_state(self, new_state):
        log_event(LogType.LOG_TRACE, "Robot state change %s -> %s" 
                    % (RobotState.name(self.state), RobotState.name(new_state)))
        self.state = new_state

    def can_step_to(self, target):
        x1, y1 = self.position
        x2, y2 = target
        return abs(x1 - x2) + abs(y1 - y2) == 1

    def follow_path(self, path):
        robo_assert(
            path.start == self.position,
            PanicReason.PANIC_PATH_INVALID,
            "path start %s does not match robot position %s" % (path.start, self.position),
        )

        self.change_state(RobotState.ROBOT_TRAVELLING)

        for next_coord in path.points[1:]:
            robo_assert(
                self.can_step_to(next_coord),
                PanicReason.PANIC_PATH_INVALID,
                "impossible move from %s to %s" % (self.position, next_coord),
            )

            self._move_to(next_coord)

    def _move_to(self, coord):
        if CONFIG_DEBUG:
            print_message("moving from %s to %s" % (self.position, coord))

        # TODO: Drivetrain
        self.position = coord
    
    def deliver_package(self):
        self.change_state(RobotState.ROBOT_DELIVERING)
        pass # TODO


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# manhattan distance as we restrict ourselves to horizontal and vertical
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(
        a[1] - b[1]
    )  # imagine having an algorithm named after your bad urban planning


def build_map_from_config():
    grid = GridMap(CONFIG_MAP_WIDTH, CONFIG_MAP_HEIGHT)

    for entry in CONFIG_MAP_OBSTACLES:
        weight, x, y = entry
        coord = (x, y)

        robo_assert(
            grid.in_bounds(coord),
            PanicReason.PANIC_MAP_CORRUPT,
            "obstacle out of bounds: %d, %d" % (coord[0], coord[1]),
        )

        grid.tile(coord).weight = weight

    return grid

# We're using an A* algorithm here to pathfind, however we implemented very strong bias
# towards avoiding turning and preferring direct straight lines for error minimization

# As an effect of this, this algorithm often behaves identically to a much simpler, more
# naive "drive forward to the right row, turn right, drive to right column", especially
# on smaller (e.g. 6x6) maps that have smaller turn costs.

# However, we will preserve the use of A* in case we want to experiment with treating
# smaller units as "tiles" (e.g. one virtual tile is not a physical tile on the ground)


# This algorithm can also be unbiased to test different degrees of turn costs to time efficiency
def astar_internal(
    grid,
    start,
    goal,
    turn_cost = CONFIG_TURN_COST,
    start_dir = None,
):
    log_event(LogType.LOG_DEBUG, "astar_with_directions called: start=%s, goal=%s, start_dir=%s, turn_cost=%s" %
              (start, goal, start_dir, turn_cost))

    robo_assert(
        grid.in_bounds(start) and grid.in_bounds(goal),
        PanicReason.PANIC_COORD_INVALID,
        "start or goal out of bounds",
    )

    end_tile_changed = False
    if not grid.tile(goal).traversable:
        grid.tile(goal).weight = 0
        end_tile_changed = True

    if not grid.tile(start).traversable:
        return None

    # (f_score, Coord)
    open_heap = heapq()
    heapq.heappush(open_heap, (0, start))

    # path reconstruction
    came_from = {}
    came_from_dir = {}

    if start_dir is not None:
        came_from_dir[start] = start_dir

    g_score = {start: 0}

    def get_direction(a, b):
        return (b[0] - a[0], b[1] - a[1])

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current == goal:
            path = Path(reconstruct_path(came_from, current))
            log_event(LogType.LOG_TRACE, "found path")
            
            if end_tile_changed:
                grid.tile(goal).weight = -1
            
            return path

        for neighbor in grid.neighbors(current):
            tentative_g = g_score[current] + grid.tile(neighbor).cost
            move_dir = get_direction(current, neighbor)
            prev_dir = came_from_dir.get(current)

            if prev_dir is not None and move_dir != prev_dir:
                tentative_g += turn_cost

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                came_from_dir[neighbor] = move_dir
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f_score, neighbor))

    if end_tile_changed:
        grid.tile(goal).weight = -1

    return None

def astar(grid, start, goal, turn_cost=CONFIG_TURN_COST, start_dir=None, tried_fallback=False):
    path = astar_internal(grid, start, goal, turn_cost, start_dir)
    
    if path is not None:
        return path

    if not tried_fallback and turn_cost != 0:
        log_event(LogType.LOG_WARN, "No path found with turn_cost=%s, retrying with turn_cost=0" % turn_cost)
        return astar(grid, start, goal, turn_cost=0, start_dir=start_dir, tried_fallback=True)

    return None


GLOBAL_MAP = None
ROBOT = None
MENU = None

def robot_render_pos(): # TODO: if someone can figure out how to use something "like" Rust async channels
                        # within this environment, it would be really *really* nice to have a separate
                        # render thread alongside the main one, and send render requests to the renderer. 

                        # From my understanding, VexOS uses a cooperative scheduler (i.e. no preemption),
                        # so this might not need mutexes/locks (and python wouldn't let me do that anyways
                        # because we are in a barebones environment and thus cannot use atomics or handroll
                        # atomics with inline assembly - maybe we could do this in C++)

                        # But yeah, I think having an SPSC queue of render requests to better abstract
                        # over the display rendering mechanism would be *super* nice!!!

    # We always want to make the robot's internal position visible
    # on the screen. We'll render this at the top left of the display.

    # If the cursor was previously occupying the location of the coordinates,
    # then we move the cursor down to no longer overwrite our coordinates.

    # Otherwise, we'll just move the cursor down one row since we won't be
    # overwriting the coordinates we just wrote to the screen
    row = brain.screen.row()
    col = brain.screen.column()
    brain.screen.set_cursor(1, 1)
    x, y = ROBOT.position
    brain.screen.print("(" + str(x) + ", " + str(y) + ")")

    if row == 1:
        brain.screen.next_row()
    else:
        brain.screen.set_cursor(row, col) # go back to where we were


def menu_right_callback():
    if ROBOT.state == RobotState.ROBOT_IN_MENU:
        MENU.scroll_right()

def menu_left_callback():
    if ROBOT.state == RobotState.ROBOT_IN_MENU:
        MENU.scroll_left()

def travel_to(target):
    global GLOBAL_MAP
    path = astar(GLOBAL_MAP, ROBOT.position, target.coords,
                CONFIG_TURN_COST, 0)
        
    if not path:
        play_audio("alert_no_valid_path.wav")
        return
    
    log_event(LogType.LOG_TRACE, "Following path")
    ROBOT.follow_path(path)
    ROBOT.deliver_package()
    play_audio("complete.wav")

    pass # TODO

# THE BIG INIT - initialize in-software things BEFORE calibration 
# (i.e. initialize things that don't need user input)
def init():
    parse_locations()
    global GLOBAL_MAP, ROBOT, MENU
    GLOBAL_MAP = build_map_from_config()
    if CONFIG_DEBUG:
        print_grid(GLOBAL_MAP)

    ROBOT = Robot((0, 0))
    MENU = menu_from_locations()
    # Code init finished

    brain.screen.set_font(CONFIG_FONT)
    init_advance(InitStage.INIT_ROBOT)

    if CONFIG_DEBUG:
        play_audio("debug_mode.wav")
    else:
        play_audio("release_mode.wav")

def calibrate():
    if ROBOT.state != RobotState.ROBOT_STOPPED:
        return
    
    brain_inertial.set_heading(0, DEGREES)
    brain_inertial.set_rotation(0, DEGREES)
    brain_inertial.calibrate()
    ROBOT.change_state(RobotState.ROBOT_INITIALIZED)

def main():
    init()
    play_audio("on.wav")

    controller.buttonB.pressed(calibrate)

    while ROBOT.state != RobotState.ROBOT_INITIALIZED:
        spin_wait()

    play_audio("initialized.wav")
    init_advance(InitStage.INIT_RUNNING)
    first_run = True

    while True:
        target = MENU.enter(first_run)
        first_run = False
        # This is the main loop. In here we want to select a target, go there,
        # select a target again, and continue on and on
        travel_to(target)

if __name__ == "__main__":
    main()
