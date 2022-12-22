from Cells import *
import numpy as np
import enum
from State import CellStatesDesks, CellStatesTasks


class TaskTypes(enum.Enum):
    order = 1
    pickup = 2


class Task():
    def __init__(self, type, desk, frame) -> None:
        self.type = type
        self.desk = desk
        self.frame = frame

    def set_cell(self, cell, making_frame):
        self.cell = cell
        self.making_frame = making_frame
        self.i = cell.i
        self.j = cell.j
        self.cell.add_state(CellStatesTasks.making)

    def __str__(self) -> str:
        return 'Task: %s@%s' % (self.type, self.desk)


class Machine():
    def __init__(self) -> None:
        pass


class CoffeeMachine(Machine):
    def __init__(self, i, j, grid, fps) -> None:
        super().__init__()
        self.i = i
        self.j = j
        self.grid = grid
        self.fps = fps
        self.cell = self.grid.cells[self.i][self.j]
        self.cell.add_state(CellStatesDesks.machine)
        self.capacity = 10
        self.init_table()
        self.not_started_tasks_queue = []
        self.started_tasks_queue = []
        self.ready_tasks_queue = []
        self.frames_to_make = 1
        self.desks = self.grid.desks
        self.max_to_order_time = 3000.0
        self.max_to_finish_time = 3000.0
        self.to_order_prob = 1/(self.fps*self.max_to_order_time)
        self.to_finish_prob = 1/(self.fps*self.max_to_finish_time)
        self.frames = 0
        self.read_grid()
        """ print('%.6f' % self.to_order_prob)
        print('%.6f' % self.to_finish_prob) """

    def read_grid(self):
        for _ in self.grid.cells:
            for cell in _:
                if cell.is_desk():
                    if cell.is_ordered():
                        print('ordered cell:%i prob:%.3f frame_no:%i' %
                              (cell.id, cell.to_order_acc_prob, self.frames))
                        self.not_started_tasks_queue.append(
                            Task(TaskTypes.order, cell, self.frames))
                    elif cell.is_finished():
                        print('finish cell:%i prob:%.6f frame_no:%i' %
                              (cell.id, cell.to_finish_acc_prob, self.frames))
                        self.not_started_tasks_queue.append(
                            Task(TaskTypes.pickup, cell, self.frames))

    def update(self):
        self.frames += 1
        self.update_desks()
        self.update_tasks()

    def update_desks(self):
        for desk in self.desks:
            if desk.is_neutral():
                if self.chance(desk.to_order_acc_prob):
                    print('ordered cell:%i prob:%.3f frame_no:%i' %
                          (desk.id, desk.to_order_acc_prob, self.frames))
                    desk.add_state(CellStatesDesks.ordered)
                    desk.to_order_acc_prob = 0.0
                    self.not_started_tasks_queue.append(
                        Task(TaskTypes.order, desk, self.frames))
                else:
                    desk.to_order_acc_prob += self.to_order_prob

            elif desk.is_received():
                if self.chance(desk.to_finish_acc_prob):
                    print('finish cell:%i prob:%.6f frame_no:%i' %
                          (desk.id, desk.to_finish_acc_prob, self.frames))
                    desk.add_state(CellStatesDesks.finished)
                    desk.to_finish_acc_prob = 0.0
                    self.not_started_tasks_queue.append(
                        Task(TaskTypes.pickup, desk, self.frames))
                else:
                    desk.to_finish_acc_prob += self.to_finish_prob

    def update_tasks(self):
        while len(self.not_started_tasks_queue) != 0 and len(self.started_tasks_queue)+len(self.ready_tasks_queue) < self.capacity:
            task = self.not_started_tasks_queue.pop(0)
            table_cell = self.table_cells.pop(0)
            task.set_cell(table_cell, self.frames)
            self.started_tasks_queue.append(task)
        while len(self.started_tasks_queue) != 0 and self.frames-self.started_tasks_queue[0].making_frame >= self.frames_to_make:
            task = self.started_tasks_queue.pop(0)
            task.cell.add_state(CellStatesTasks.ready)
            self.ready_tasks_queue.append(task)

    def init_table(self):
        l = 0
        h = 5
        ts = 5
        while l <= h:
            md = (l+h) >> 1
            if (md*(md+1)/2)*8 >= self.capacity:
                ts = md
                h = md-1
            else:
                l = md+1
        self.table_size = ts
        self.table_cells = []
        for i in range(-self.table_size, self.table_size+1):
            for j in range(-self.table_size, self.table_size+1):
                if i == 0 and j == 0:
                    continue
                idx = self.i+i
                idy = self.j+j
                if idx >= 0 and idx < self.grid.grid_size and idy >= 0 and idy < self.grid.grid_size and self.grid.cells[idx][idy].is_available():
                    self.table_cells.append(self.grid.cells[idx][idy])
                    self.grid.cells[idx][idy].add_state(CellStatesTasks.table)

    def finish_tasks(self, chosen_tasks):
        for task in chosen_tasks:
            task.cell.add_state(CellStatesTasks.table)
            self.ready_tasks_queue.remove(task)
            self.table_cells.append(task.cell)

    def chance(self, prop):
        return 1-prop <= np.random.random()
