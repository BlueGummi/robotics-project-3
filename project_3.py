#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math

# Brain should be defined by default
brain = Brain()

# Robot configuration code
brain_inertial = Inertial()
left_drive_smart = Motor(Ports.PORT6, False)
right_drive_smart = Motor(Ports.PORT1, True)
drivetrain = SmartDrive(left_drive_smart, right_drive_smart, brain_inertial, 319.19, 320, 40, MM, 1)
controller = Controller()


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
# to "0,0". Then, it will follow a predefined path and turn accordingly
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

# TODO: Figure out what the robot's memory usage limitations are

# These two variables exist to limit the size of the map.
# The limits exist because internally we construct a large 2D array of `Tile`s to represent the map.
# Computers have limited memory, so eventually it will OOM (out of memory),
# and these just exist to make sure we can safely exit before that happens
CONST_MAX_WIDTH = 128
CONST_MAX_HEIGHT = 128
# ESP32 based, 520KB SRAM, Brain Main Memory is 320KB Heap

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

# --------------- DEBUG/RELEASE ---------------

CONFIG_DEBUG = (
    True  # Defines if we are in a debug build. More logging functionality will be enabled if we are, but this
)         # may come at the cost of slowing down the program due to the instrumentation

# --------------- CUSTOM VARS ---------------
CONFIG_DIST_TO_HOUSE_FROM_CORNER = 16 # inches

# --------------- MAP ---------------

CONFIG_MAP_WIDTH = 24 # Map width in "tiles".
CONFIG_MAP_HEIGHT = 24  # Map height in "tiles".
CONFIG_NOP_MOVES = False # Whether or not moves should be signaled to the robot.
                        # Useful if testing robot without motors. NOTE: please
                        # remember to refactor drivetrain commands into 
                        # a wrapper function to avoid scattering checks of this everywhere!

CONFIG_MAP_TILE_SIDE_INCHES = 6
CONFIG_WHEEL_DIAMETER_IN = 4
# This is an array of arrays of (weight, x, y). We parse this one time at init to label the obstacles on our map.
CONFIG_MAP_OBSTACLES = [
    [CONST_WEIGHT_UNTRAVERESABLE, 11, 5],
    [CONST_WEIGHT_UNTRAVERESABLE, 7, 23],
    [CONST_WEIGHT_UNTRAVERESABLE, 13, 13],
    [CONST_WEIGHT_UNTRAVERESABLE, 23, 9],
]  # Houses A, B, C, and D respectively

# NOTE: if a LOCATION is in the "obstacles" list as UNTRAVERSABLE, the
# robot will still allow traversal to the location if AND ONLY IF it is
# set as a destination in the CONFIG_ROUTE variable.

# The way we represent LOCATIONs is we first state its initial target, and
# then its final move to orient at the house. This is because houses are 
# faced angled in certain directions, and must be reached by first
# going to a location, then turning, and driving forward a bit.

# This array is thus a series of
# [location_name: str, location_sound: str, initial_x: int
# initial_y: int, final_orientation: float, final_traversal: float]

# When we traverse locations, we first traverse with A* to the initial_x/y,
# and then we orient the robot to final_orientation, travel for final_traversal,
# before turning back around to return to initial_x/y, and continuing

# All final_orientations are ABSOLUTE, where 0.0 is NORTH
CONFIG_MAP_LOCATIONS = [
    ["House A", "house_a.wav", 8, 8, 135, CONFIG_DIST_TO_HOUSE_FROM_CORNER],
    ["House B", "house_b.wav", 4, 20, 45, CONFIG_DIST_TO_HOUSE_FROM_CORNER],
    ["House C", "house_c.wav", 16, 16, 225, CONFIG_DIST_TO_HOUSE_FROM_CORNER],
    ["House D", "house_d.wav", 20, 12, 135, CONFIG_DIST_TO_HOUSE_FROM_CORNER]
]

# --------------- ROUTE ---------------

CONFIG_ROUTE = [ # A route from start to finish, all route locations MUST be in CONFIG_MAP_LOCATIONS
    "House A",
    "House B",
    "House C",
    "House D",
]

# --------------- ROBOT INIT STATE ---------------

CONFIG_ROBOT_START_POS = (6, 2) # Tuple to indicate software-perceived start pos
CONFIG_ROBOT_START_ORIENTATION = 0.0 # direction robot is facing at start,
                                     # "0.0" is "north"

# definition:
#    LOG_TRACE = 0  # Trace events
#    LOG_DEBUG = 1  # Debugging messages
#    LOG_WARN = 2  # Warnings
#    LOG_ERR = 3  # Fatal errors (these do NOT panic, however)

CONFIG_PRINT_LOG_DEPTH = 0 # The deepest log we should still print
                           # e.g. setting this to 2 will print 
                           # ERR and WARN but not DEBUG or TRACE

CONFIG_FONT = FontType.PROP60 # Please change CONFIG_FONT_ROWS and CONFIG_FONT_COLS according to the
                              # table listed in brain.screen.next_row()'s documentation file!
CONFIG_FONT_ROWS = 1
CONFIG_FONT_COLS = 9

# --------------- LOCOMOTION ---------------
CONFIG_BATCH_MOVES = True # This will make it so that instead of stopping 
                          # after each "move", the robot will traverse in a
                          # straight line and then turn afterwards  
CONFIG_TURN_COST = 5 # Used in A*, tunable
CONFIG_TURN_ITERS = 2 # How many extra times after an initial turn do we check again?
CONFIG_TURN_ERROR_MARGIN = 5 # How much difference until we turn?

CONFIG_AUDIO_DIR = "" # only if the audio on the microSD is under some directory
CONFIG_SOUND_VOL = 50 # Range 0-100

CONFIG_SPIN_LATENCY_MS = 5 # Spinning/waiting in a loop - how long should we wait each iteration?

# TODO: more configs for robot/arm speed and whatnot as they come about

CONFIG_ADJUSTMENT_INCHES = 6 # How many inches do we move when adjusting?
CONFIG_ROBOT_DRIVE_VEL_PCT = 60
CONFIG_ROBOT_TURN_VEL_PCT = 50
CONFIG_ROBOT_FINAL_VEL_PCT = 20 # Final turn towards a location

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
        self._siftup(len(self.data) - 1)

    def heappop(self):
        if len(self.data) == 0:
            panic("pop from empty heap")
        if len(self.data) == 1:
            return self.data.pop()
        root = self.data[0]
        self.data[0] = self.data.pop()
        self._siftdown(0)
        return root

    def _siftup(self, idx):
        parent = (idx - 1) // 2
        while idx > 0 and self.data[idx][0] < self.data[parent][0]:
            self.data[idx], self.data[parent] = self.data[parent], self.data[idx]
            idx = parent
            parent = (idx - 1) // 2

    def _siftdown(self, idx):
        n = len(self.data)
        while True:
            left = 2 * idx + 1
            right = 2 * idx + 2
            smallest = idx

            if left < n and self.data[left][0] < self.data[smallest][0]:
                smallest = left
            if right < n and self.data[right][0] < self.data[smallest][0]:
                smallest = right
            if smallest == idx:
                break
            self.data[idx], self.data[smallest] = self.data[smallest], self.data[idx]
            idx = smallest

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
        if brain.sound_is_active():
            brain.sound_off()

        brain.play_file(raw_path, CONFIG_SOUND_VOL)
        if blocking:
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
             # words removes this problem because filenames are shorter
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
    brain.screen.print(msg)
    brain.screen.next_row()

def clear_console():
    brain.screen.clear_screen()

# Returns the temperature of the battery if the robot is currently on fire.
# Smoldering doesn't count. If the robot isn't on fire, the function returns some other value.
def is_robot_on_fire():
    temp = brain.battery.temperature(TemperatureUnits.CELSIUS)
    if temp > 100:
        return temp
    
    return 0.63739

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

    brain.program_stop()

def panic_manual():
    play_audio("halt_manual_override.wav")
    panic("Manual override")

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

    if CONFIG_DEBUG:
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

def find_orient(cur, next_pos):
    if cur[0] == next_pos[0]:
        if next_pos[1] > cur[1]:
            return 0
        return 180
    if cur[1] == next_pos[1]:
        if next_pos[0] > cur[0]:
            return 90
        return 270

    robo_assert(False, PanicReason.PANIC_PATH_INVALID, "path points are further than one apart")
    return 0

# A "GridMap" structure to represent all the Tiles of the map
class GridMap:
    def __init__(self, width, height):
        robo_assert(
            width <= CONST_MAX_WIDTH and height <= CONST_MAX_HEIGHT,
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

    def neighbors(self, coord): # capture neighbors for this coordinate
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


LOCATIONS = [] # List of Locations
class Location:
    def __init__(self, name, audio_file, coordinate, final_orient, final_len):
        self.name = name
        self.audio_file = audio_file
        self.coords = coordinate
        self.final_orient = final_orient
        self.final_len = final_len

def parse_locations():
    for location in CONFIG_MAP_LOCATIONS:
        name = location[0]
        audio_file = location[1]
        x = location[2]
        y = location[3]
        final_orientation = location[4]
        final_len = location[5]
        lclass = Location(name, audio_file, (x, y), final_orientation, final_len)    
        LOCATIONS.append(lclass)

# No need to use __slots__ here, since these structures aren't as frequently
# instantiated, and we might end up adding extra properties later on
class Path:
    def __init__(self, points, final_orient = 0, final_len = 0): 
        robo_assert(
            len(points) != 0, PanicReason.PANIC_PATH_INVALID, "path must have points"
        )
        self.points = points
        self.final_orient = final_orient
        self.final_len = final_len

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
#             │                   │               ENTER LOOP     
#             └──RETURN BACK TO───┘                   ▼          
#                                           ┌──────────────────┐ 
#                                           │                  │ 
#             ┌────PICK NEXT────┬──────────▶│   FIND TARGET    │ 
#             │                 │           │                  │ 
#   ┌───────────────────┐       │           └──────────────────┘ 
#   │                   │       │                     │          
#   │  DELIVER PACKAGE  │   PATH FAIL                 │          
#   │                   │       │                     │          
#   └───────────────────┘       │                     │          
#             ▲                 │                     │          
#             │          ┌────────────┐               │          
#             │          │            │               │          
#             └─PATH OK──│   TRAVEL   │◀───SELECTED───┤          
#                        │            │               │          
#                        └────────────┘               │          
#                                                     │          
#                                                     │          
#                                ◎                    │          
#                                ╳                    │          
#                           STOP SIGNAL               │          
#                         (can arrive at              │          
#                            any time)                │          
#                                ╳                    │          
#                                ▼                    │          
#                        ┌───────────────┐            │          
#                        │               │            │          
#                        │    STOPPED    │◀─WENT TO ALL          
#                        │               │                       
#                        └───────────────┘                       

class RobotState(FakeIntEnum):
    ROBOT_FAILED_INIT = -1
    ROBOT_STOPPED = 0
    ROBOT_INITIALIZED = 1
    ROBOT_SELECTING = 2
    ROBOT_TRAVELLING = 3
    ROBOT_DELIVERING = 4
    ROBOT_ADJUSTING = 5
    ROBOT_DELIVERED = 6

class RobotAdjustment(FakeIntEnum):
    ROBOT_LEFT = 1
    ROBOT_RIGHT = 2
    ROBOT_FORWARD = 3
    ROBOT_BACKWARD = 4

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
    
    def turn(self, new_orient):
        log_event(LogType.LOG_TRACE, "Turn to %f" % new_orient)

        def angle_delta(target, current):
            return ((target - current + 180) % 360) - 180

        for i in range(0, CONFIG_TURN_ITERS):
            orient = brain_inertial.heading(DEGREES)
            delta = angle_delta(new_orient, orient)

            log_event(LogType.LOG_DEBUG, "orient = %f, new_orient = %f, delta = %f" % (orient, new_orient, delta))
            if abs(delta) < CONFIG_TURN_ERROR_MARGIN:
                break
            
            if not CONFIG_NOP_MOVES:
                drivetrain.turn_for(RIGHT, delta)
        
        log_event(LogType.LOG_TRACE, "Turn completed")
        wait(50, MSEC)
    
    def move_by_tiles(self, tiles):
        if CONFIG_NOP_MOVES:
            return

        heading_to_maintain = brain_inertial.heading(DEGREES)
        wheel_circumference = CONFIG_WHEEL_DIAMETER_IN * math.pi
        travelled_distance = tiles * CONFIG_MAP_TILE_SIDE_INCHES

        expected_rotations = travelled_distance / wheel_circumference
        expected_degree_change = 360 * expected_rotations
                
        starting_mpos = left_drive_smart.position(DEGREES)
        expected_mpos = starting_mpos + expected_degree_change

        drivetrain.drive(FORWARD)
        # We can poll an arbitrary motor since they should be in sync
        while left_drive_smart.position(DEGREES) < expected_mpos:
            log_event(LogType.LOG_TRACE, "ldsmartpos = %d, expected = %d" % (left_drive_smart.position(DEGREES), expected_mpos) )
            spin_wait()
        
        drivetrain.stop()
        
        # done.
        
        

    def _follow_path_batched(self, path):
        # In here, we'll want to follow a path until we are "on final approach".
        # However, rather than scooting one by one, we'll first identify our
        # starting orientation, and then traverse a whole line until we need 
        # to turn, and continue to do the same until we reach our destination

        def get_num_traversals_until_turn(coord_list, start_orient):
            # In here, we find how long it takes until two elements' orientation
            # in `path` doesn't match `orient`
            index = 0
            for coordinate in coord_list[:-1]:
                cur = coordinate
                next_pos = coord_list[index + 1]
                next_orient = find_orient(cur, next_pos)
                if next_orient != start_orient:
                    return index, next_orient, False
                
                index += 1

            return len(coord_list), start_orient, True
        
        log_event(LogType.LOG_DEBUG, "First point is %s, second is %s" % (path.points[0], path.points[1]))
        initial_orient = find_orient(path.points[0], path.points[1]) # what is our initial orientation?
        self.turn(initial_orient)

        traversed_steps = 0
        log_event(LogType.LOG_TRACE, "Path: %s" % (path.points))
        next_orient = initial_orient
        while True:
            steps, next_orient, done = get_num_traversals_until_turn(path.points[traversed_steps:], 
                                                          next_orient)

            log_event(LogType.LOG_TRACE, "Moving %d tiles forward" % steps)
            self.move_by_tiles(steps)
            if not done:
                self.turn(next_orient)
            log_event(LogType.LOG_TRACE, "Traversed %d steps in %f" % (steps, next_orient))
            traversed_steps += steps
            if done:
                break
        
        self.position = path.points[-1]

    def _follow_path_unbatched(self, path):
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
        orient = find_orient(self.position, coord)
        if abs(brain_inertial.heading(DEGREES) - orient) > CONFIG_TURN_ERROR_MARGIN:
            self.turn(orient) # turn only if over margin of error
        
        self.move_by_tiles(1)
        self.position = coord

    def follow_path(self, path):
        log_event(LogType.LOG_TRACE, "starting path traversal")
        if CONFIG_BATCH_MOVES:
            self._follow_path_batched(path)
        else:
            self._follow_path_unbatched(path)

        # Go "on final"
        log_event(LogType.LOG_TRACE, "completed path traversal")
        self.turn(path.final_orient)

        if not CONFIG_NOP_MOVES:
            drivetrain.set_drive_velocity(CONFIG_ROBOT_FINAL_VEL_PCT, PERCENT)
            drivetrain.drive_for(FORWARD, path.final_len)
            drivetrain.set_drive_velocity(CONFIG_ROBOT_DRIVE_VEL_PCT, PERCENT)

        # OK

    # "overloaded" internal function to make an adjustment
    def _adjust_internal(self, turn = None):
        if CONFIG_NOP_MOVES:
            return

        if turn and turn != REVERSE:
            drivetrain.turn_for(turn, 90)

        if turn and turn == REVERSE:
            drivetrain.drive_for(REVERSE, CONFIG_ADJUSTMENT_INCHES)
        else:
            drivetrain.drive_for(FORWARD, CONFIG_ADJUSTMENT_INCHES)

        if turn and turn != REVERSE:
            drivetrain.turn_for(turn, -90)

    def _adjust_left(self):
        self._adjust_internal(LEFT)
        
    def _adjust_right(self):
        self._adjust_internal(RIGHT)

    def _adjust_forward(self):
        self._adjust_internal(None)

    def _adjust_backward(self):
        self._adjust_internal(REVERSE)

    def adjust(self, adjustment):
        if self.state != RobotState.ROBOT_ADJUSTING: # not adjusting
            return
        
        adjustments = [self._adjust_left, self._adjust_right, 
                       self._adjust_forward, self._adjust_backward]
        adjustments[adjustment]()

    def deliver_package(self, path):
        self.change_state(RobotState.ROBOT_DELIVERING)

        # TODO: delivery

        log_event(LogType.LOG_TRACE, "spinning back around")
        self.turn(path.final_orient + 180)
        if not CONFIG_NOP_MOVES:
            drivetrain.drive_for(FORWARD, path.final_len)
    
    def shutdown(self): # trigger this on program halt. 
                        # this should shutdown the robot,
                        # and make sure that nothing
                        # is left in a weird undefined state.

                        # for example, an arm should be DOWN

                        # TODO
        pass


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
    start_dir = None,
):
    log_event(LogType.LOG_DEBUG, "astar_with_directions called: start=%s, goal=%s, start_dir=%s, turn_cost=%s" %
              (start, goal, start_dir, CONFIG_TURN_COST))

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

    # manhattan distance as we restrict ourselves to horizontal and vertical
    def manhattan_heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def reconstruct_path(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

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
                tentative_g += CONFIG_TURN_COST

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                came_from_dir[neighbor] = move_dir
                g_score[neighbor] = tentative_g
                f_score = tentative_g + manhattan_heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f_score, neighbor))

    if end_tile_changed:
        grid.tile(goal).weight = -1

    return None

# This will return a path WITHOUT a final_orient/len, we must add it on on top
def astar(grid, start, goal):
    path = astar_internal(grid, start, goal)
    
    if path is not None:
        return path

    log_event(LogType.LOG_WARN, "No path found with turn_cost=%d, retrying with turn_cost=0" % CONFIG_TURN_COST)
    return astar_internal(grid, start, goal)

def generate_path_for_destination(target):
    path = astar(GLOBAL_MAP, ROBOT.position, target.coords)

    path.final_orient = target.final_orient
    path.final_len = target.final_len
    return path

GLOBAL_MAP = None
ROBOT = None

def deliver_complete():
    if ROBOT.state == RobotState.ROBOT_DELIVERING:
        ROBOT.change_state(RobotState.ROBOT_DELIVERED)    

def deliver_a():
    if ROBOT.state == RobotState.ROBOT_DELIVERING:
        play_audio("adjusting.wav", blocking=False)
        ROBOT.change_state(RobotState.ROBOT_ADJUSTING)

def calibrate_b():
    if ROBOT.state != RobotState.ROBOT_STOPPED:
        return
    
    brain_inertial.set_heading(0, DEGREES)
    brain_inertial.set_rotation(0, DEGREES)
    brain_inertial.calibrate()
    ROBOT.change_state(RobotState.ROBOT_INITIALIZED)

# .adjust will check this
def adjust_up():
    ROBOT.adjust(RobotAdjustment.ROBOT_FORWARD)

def adjust_down():
    ROBOT.adjust(RobotAdjustment.ROBOT_BACKWARD)

def adjust_a():
    ROBOT.adjust(RobotAdjustment.ROBOT_RIGHT)

def adjust_b():
    ROBOT.adjust(RobotAdjustment.ROBOT_LEFT)

def robot_render_pos(): 
    # We always want to make the robot's internal position visible
    # on the screen. We'll render this at the top left of the display.

    # If the cursor was previously occupying the location of the coordinates,
    # then we move the cursor down to no longer overwrite our coordinates.

    # Otherwise, we'll just move the cursor down one row since we won't be
    # overwriting the coordinates we just wrote to the screen
    while True:
        row = brain.screen.row()
        col = brain.screen.column()
        brain.screen.set_cursor(1, 1)
        x, y = ROBOT.position
        deg = brain_inertial.heading(DEGREES)
        dgstr = "%.2f" % deg
        brain.screen.print("(" + str(x) + ", " + str(y) + ") " + dgstr)
        spin_wait()

def travel_to(target):
    global GLOBAL_MAP
    path = generate_path_for_destination(target)
        
    if not path:
        play_audio("alert_no_valid_path.wav")
        return
    
    log_event(LogType.LOG_TRACE, "Following path")
    play_audio("travelling_to.wav")
    play_audio(target.audio_file)
    ROBOT.follow_path(path)
    ROBOT.deliver_package(path)
    play_audio("complete.wav", blocking=False)

    while ROBOT.state != RobotState.ROBOT_DELIVERED:
        spin_wait()

    play_audio("continuing.wav", blocking=False)

    
def get_next_location():
    ROBOT.change_state(RobotState.ROBOT_SELECTING)
    if len(LOCATIONS):
        return LOCATIONS.pop(0)

    return None

def traverse_all():
    next_place = get_next_location()
    while next_place is not None:
        travel_to(next_place)
        next_place = get_next_location()

# THE BIG INIT - initialize in-software things BEFORE calibration 
# (i.e. initialize things that don't need user input)
def init():
    parse_locations()
    global GLOBAL_MAP, ROBOT
    GLOBAL_MAP = build_map_from_config()
    if CONFIG_DEBUG:
        print_grid(GLOBAL_MAP)

    ROBOT = Robot(CONFIG_ROBOT_START_POS)
    # Code init finished

    controller.buttonDown.pressed(adjust_down)
    controller.buttonUp.pressed(adjust_up)
    controller.buttonA.pressed(adjust_a)
    controller.buttonB.pressed(adjust_b)
    controller.buttonA.pressed(deliver_a)
    controller.buttonB.pressed(deliver_complete)
    controller.buttonL1.pressed(deliver_complete)
    controller.buttonL2.pressed(deliver_complete)
    controller.buttonR1.pressed(deliver_complete)
    controller.buttonR2.pressed(deliver_complete)
    
    brain.screen.set_font(CONFIG_FONT)

    init_advance(InitStage.INIT_ROBOT)

    #if CONFIG_DEBUG:
    #    log_event(LogType.LOG_DEBUG, "In debug mode")
    #    play_audio("debug_mode.wav")
    #else:
    #    play_audio("release_mode.wav")

def main():
    init()    
    controller.buttonB.pressed(calibrate_b)

    play_audio("on.wav", blocking=False)

    while ROBOT.state != RobotState.ROBOT_INITIALIZED:
        spin_wait()

    drivetrain.set_turn_velocity(CONFIG_ROBOT_TURN_VEL_PCT, PERCENT)
    drivetrain.set_drive_velocity(CONFIG_ROBOT_DRIVE_VEL_PCT, PERCENT)
    play_audio("initialized.wav", blocking=False)
    init_advance(InitStage.INIT_RUNNING)

    Thread(robot_render_pos)
    traverse_all()

    brain.program_stop() # safe to invoke here

if __name__ == "__main__":
    main()
