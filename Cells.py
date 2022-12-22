from kivy.uix.boxlayout import BoxLayout
from kivy.properties import NumericProperty, ColorProperty, ObjectProperty
import random
import enum
import copy
import State
from State import CellStatesTasks, CellStatesAvailability, CellStatesDesks, CellStatesRobot, CellColors


class SquareCell(BoxLayout):
    value = NumericProperty(0)
    color = ColorProperty()
    state = ObjectProperty()
    i = NumericProperty()
    j = NumericProperty()
    id = NumericProperty()
    dist = NumericProperty()
    gridSize = NumericProperty()

    def __init__(self, i, j, gsz, sts=[], **kwargs):
        super().__init__(**kwargs)
        self.i = i
        self.j = j
        self.gridSize = gsz
        self.id = self.i*gsz+j
        self.idx = -1
        # self.color = random.random(), random.random(), random.random(), 1
        self.color = 0, 0, 0, 0.2
        self.value = 0
        self.bind(state=self.update_color)
        self.states = sts.copy()
        if len(self.states) == 0:
            self.add_state(CellStatesAvailability.available)
        else:
            self.update_state()
        self.original_state = self.state
        self.par = None
        self.dist = -1
        self.g = -1
        self.to_order_acc_prob = 0.0
        self.to_finish_acc_prob = 0.0

    def __lt__(self, other):
        self_priority = (self.dist, self.id)
        other_priority = (other.dist, other.id)
        return self_priority < other_priority

    def reset(self):
        self.states = []
        self.add_state(copy.copy(self.original_state))
        self.par = None
        self.dist = -1
        self.g = -1

    def clear(self):
        self.states = []
        self.add_state(CellStatesAvailability.available)
        self.par = None
        self.dist = -1
        self.g = -1

    def update_color(self, a, b):
        self.color = CellColors.color[self.state]

    def add_state(self, new_state):
        to_remove = []
        for state in self.states:
            if state.__class__ == new_state.__class__:
                to_remove.append(state)
        for state in to_remove:
            self.states.remove(state)
        self.states.append(new_state)
        self.update_state()

    def rem_state(self, old_state):
        to_remove = []
        for state in self.states:
            if state.__class__ == old_state.__class__:
                to_remove.append(state)
        for state in to_remove:
            self.states.remove(state)
        self.update_state()

    def update_state(self):
        self.state = max(self.states, key=lambda e: int(e.value))

    def on_touch_down(self, touch):
        if State.UPDATING == True:
            return True
        if self.collide_point(touch.x, touch.y):
            if touch.button == 'left':
                self.add_state(State.EDITING_STATE)
            elif touch.button == 'right':
                self.clear()

            return True
        else:
            return super().on_touch_move(touch)

    def on_touch_move(self, touch):
        if State.UPDATING == True:
            return True
        if self.collide_point(touch.x, touch.y):
            if touch.button == 'left':
                self.add_state(State.EDITING_STATE)
                if State.EDITING_STATE == CellStatesDesks.machine or State.EDITING_STATE == CellStatesRobot.occupied:
                    State.EDITING_STATE = CellStatesAvailability.unavailable
            elif touch.button == 'right':
                self.clear()

            return True
        else:
            return super().on_touch_move(touch)

    def is_available(self):
        return self.states.__contains__(CellStatesAvailability.available)

    def is_visited(self):
        return self.states.__contains__(CellStatesAvailability.visited)

    def is_tovisit(self):
        return self.states.__contains__(CellStatesAvailability.tovisit)

    def is_blocked(self):
        return self.states.__contains__(CellStatesAvailability.blocked)

    def is_unavailable(self):
        return self.states.__contains__(CellStatesAvailability.unavailable)

    def is_unvisited(self):
        return self.states.__contains__(CellStatesAvailability.unvisited)

    def is_occupied(self):
        return self.states.__contains__(CellStatesAvailability.occupied)

    def is_neutral(self):
        return self.states.__contains__(CellStatesDesks.neutral)

    def is_ordered(self):
        return self.states.__contains__(CellStatesDesks.ordered)

    def is_received(self):
        return self.states.__contains__(CellStatesDesks.received)

    def is_finished(self):
        return self.states.__contains__(CellStatesDesks.finished)

    def is_desk(self):
        return self.is_neutral() or self.is_ordered() or self.is_received() or self.is_finished()

    def is_machine(self):
        return self.states.__contains__(CellStatesDesks.machine)

    def is_robot(self):
        return self.states.__contains__(CellStatesRobot.occupied)

    def __str__(self) -> str:
        return 'cell(%s,%s) id= %s' % (str(self.i), str(self.j), str(self.id))
