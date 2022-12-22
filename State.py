import enum
import random


class CellStates(enum.Enum):
    pass


class CellStatesAvailability(CellStates):
    available = 1
    unavailable = 1000
    blocked = 3000
    visited = 6
    unvisited = 7
    tovisit = 8


class CellStatesRobot(CellStates):
    occupied = 100
    path = 15


class CellStatesTasks(CellStates):
    table = 12
    ready = 13
    making = 14


class CellStatesDesks(CellStates):
    machine = 90
    neutral = 19
    ordered = 20
    received = 21
    finished = 22


class CellColors():
    color = {CellStatesAvailability.unavailable: (0, 0, 0, 1),
             CellStatesAvailability.blocked: (random.random(), random.random(), random.random(), 0),
             CellStatesAvailability.available: (0.2, 0.2, 0.2, 0.1),
             CellStatesAvailability.visited: (random.random(), random.random(), random.random(), 0.2),
             CellStatesAvailability.unvisited: (random.random(), random.random(), random.random(), 0),
             CellStatesAvailability.tovisit: (random.random(), random.random(), random.random(), 0.2),
             CellStatesRobot.occupied: (random.random(), random.random(), random.random(), 1),
             CellStatesRobot.path: (random.random(), random.random(), random.random(), 0.5),
             CellStatesDesks.neutral: (0, 0, 1, 1),
             CellStatesDesks.ordered: (0, 1, 0, 1),
             CellStatesDesks.received: (1, 1, 0, 1),
             CellStatesDesks.finished: (1, 0, 0, 1),
             CellStatesDesks.machine: (random.random(), random.random(), random.random(), 1),
             CellStatesTasks.table: (random.random(), random.random(), random.random(), 1),
             CellStatesTasks.ready: (random.random(), random.random(), random.random(), 1),
             CellStatesTasks.making: (random.random(), random.random(), random.random(), 1),
             }


STOPPED = False
UPDATING = False
EDITING_STATE = CellStatesAvailability.unavailable
