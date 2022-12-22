from Cells import *
import enum
import queue
import math
from Machines import *
import numpy as np
from State import CellStatesAvailability, CellStatesDesks, CellStatesRobot, CellStatesTasks


class RobotStates(enum.Enum):
    spawned = 0
    preprocessing = 1
    idle = 2
    searching = 3
    moving = 4


class Robot():
    def __init__(self) -> None:
        pass


class CoffeeRobot(Robot):

    dx = [1, 1, 1, 0, 0, -1, -1, -1]
    dy = [1, -1, 0, 1, -1, 1, -1, 0]

    def __init__(self, i, j, grid, coffee_machine, fps) -> None:
        super().__init__()
        self.i = i
        self.j = j
        self.grid = grid
        self.coffee_machine = coffee_machine
        self.fps = fps
        self.cell = self.grid.cells[self.i][self.j]
        self.rows = self.grid.grid_size
        self.cols = self.grid.grid_size
        self.state = RobotStates.spawned
        self.desks = self.grid.desks
        self.targets = self.desks.copy()
        self.goals = []
        self.targets_dist_pairs = {}
        self.grid.cells[self.i][self.j].add_state(
            CellStatesRobot.occupied)
        self.path = []
        self.frames_per_move = 10
        self.frame_cnt = 0
        self.capacity = 5
        self.current_capacity = 0
        self.max_consider_task = 9

    def set_grid(self, grid):
        self.grid = grid

    def set_state(self, new_state):
        self.state = new_state

    def update(self):
        if self.state == RobotStates.spawned or self.state == RobotStates.preprocessing:
            self.pre_compute_distances()
        elif self.state == RobotStates.idle or self.state == RobotStates.searching:
            self.search()
        elif self.state == RobotStates.moving:
            self.move()

    def pre_compute_distances(self):
        if self.state == RobotStates.spawned:
            for _ in self.grid.cells:
                for cell in _:
                    if cell.is_available() or cell.is_visited() or cell.is_tovisit():
                        cell.add_state(CellStatesAvailability.unvisited)
                        cell.dist = -1
                        cell.par = None
            self.start = self.targets.pop(0)
            self.start.dist = 0
            self.Q = [self.start]
            self.frame_cnt = 0
            self.bfs_steps_per_frame = 50
            self.set_state(RobotStates.preprocessing)
        if self.state == RobotStates.preprocessing:
            self.bfs()
        else:
            return
        if len(self.Q) == 0:
            self.targets_dist_pairs[self.start.idx] = {}
            for desk in self.desks:
                self.targets_dist_pairs[self.start.idx][desk.idx] = desk.dist
            if len(self.targets) == 0:
                self.set_state(RobotStates.idle)
            else:
                self.set_state(RobotStates.spawned)

    def search(self):
        if self.state == RobotStates.idle:
            if len(self.goals) == 0:
                if self.cell != self.coffee_machine.cell:
                    self.goals.append(self.coffee_machine.cell)
                elif len(self.coffee_machine.ready_tasks_queue) != 0:
                    self.choose_goals()
                    return
                else:
                    return
            for _ in self.grid.cells:
                for cell in _:
                    if cell.is_available() or cell.is_visited() or cell.is_tovisit():
                        cell.add_state(CellStatesAvailability.unvisited)
                        cell.dist = -1
                        cell.g = -1
                        cell.par = None
                        cell.rem_state(CellStatesRobot.path)
            self.start = self.cell
            self.goal = self.goals[0]
            self.start.g = 0
            self.start.dist = self.heuristic_score(
                self.start, self.start, self.goal)
            self.a_star_steps_per_frame = 1
            self.frame_cnt = 0
            self.Q = queue.PriorityQueue()
            self.Q.put(self.start)
            self.set_state(RobotStates.searching)
        if self.state == RobotStates.searching:
            self.A_star()

    def choose_goals(self):
        self.ready_tasks = self.coffee_machine.ready_tasks_queue
        self.order_mask = 0
        self.pickup_mask = 0
        num_order=0
        for bit, task in enumerate(self.ready_tasks):
            if task.type == TaskTypes.order:
                self.order_mask = self.bit_set(self.order_mask, bit)
                num_order+=1
            elif task.type == TaskTypes.pickup:
                self.pickup_mask = self.bit_set(self.pickup_mask, bit)
        self.num_tasks = len(self.ready_tasks)
        tasks_mask = self.order_mask | self.pickup_mask
        d1 = self.max_consider_task+1
        d2 = self.capacity+1
        d3 = self.capacity+1
        d4 = (1 << d1)
        d5 = 2
        self.dp = np.array([-1]*(d1*d2*d3*d4*d5)).reshape(d1, d2, d3, d4, d5)
        ans = self.check(
            len(self.desks)-1, min(num_order, self.capacity), 0, tasks_mask)
        self.chosen_tasks = []
        self.backtrack(len(self.desks)-1, min(num_order, self.capacity),
                       0, tasks_mask, ans)
        self.coffee_machine.finish_tasks(self.chosen_tasks)
        for task in self.chosen_tasks:
            print(task)
            if task.type == TaskTypes.order:
                self.current_capacity += 1
            self.goals.append(task.desk)
    # [task1,task2,task3]

    # return pair (max_num_of_tasks,minimum_time)
    def check(self, desk_no, full_cups, empty_cups, tasks_mask):
        if full_cups == 0 and (empty_cups == self.capacity or (tasks_mask & self.pickup_mask) == 0):
            return list([1, self.targets_dist_pairs[desk_no][len(self.desks)-1]])
        ret = self.dp[desk_no][full_cups][empty_cups][tasks_mask]
        if not self.equal_pair(ret, [-1, -1]):
            return ret
        ans = list([-1, 1e9])
        for bit in range(self.num_tasks):
            if self.bit_on(tasks_mask, bit):
                n_desk_no = self.ready_tasks[bit].desk.idx
                n_tasks_mask = self.bit_reset(tasks_mask, bit)
                dist = self.targets_dist_pairs[desk_no][n_desk_no]
                if self.bit_on(self.order_mask, bit) and full_cups > 0:
                    take = self.add_pair([1, dist], self.check(
                        n_desk_no, full_cups-1, empty_cups, n_tasks_mask))
                    ans = self.best_pair(ans, take)
                elif self.bit_on(self.pickup_mask, bit) and full_cups+empty_cups < self.capacity:
                    take = self.add_pair([1, dist], self.check(
                        n_desk_no, full_cups, empty_cups+1, n_tasks_mask))
                    ans = self.best_pair(ans, take)
        self.dp[desk_no][full_cups][empty_cups][tasks_mask] = ans
        return self.dp[desk_no][full_cups][empty_cups][tasks_mask]

    def backtrack(self, desk_no, full_cups, empty_cups, tasks_mask, ans):
        if full_cups == 0 and (empty_cups == self.capacity or (tasks_mask & self.pickup_mask) == 0):
            return
        for bit in range(self.num_tasks):
            if self.bit_on(tasks_mask, bit):
                n_desk_no = self.ready_tasks[bit].desk.idx
                n_tasks_mask = self.bit_reset(tasks_mask, bit)
                dist = self.targets_dist_pairs[desk_no][n_desk_no]
                prev = [1, dist]
                if self.bit_on(self.order_mask, bit) and full_cups > 0:
                    take = self.add_pair([1, dist], self.check(
                        n_desk_no, full_cups-1, empty_cups, n_tasks_mask))
                    if self.equal_pair(take, ans):
                        n_ans = self.sub_pair(ans, prev)
                        self.chosen_tasks.append(self.ready_tasks[bit])
                        self.backtrack(n_desk_no, full_cups-1,
                                       empty_cups, n_tasks_mask, n_ans)
                        return
                elif self.bit_on(self.pickup_mask, bit) and full_cups+empty_cups < self.capacity:
                    take = self.add_pair([1, dist], self.check(
                        n_desk_no, full_cups, empty_cups+1, n_tasks_mask))
                    if self.equal_pair(take, ans):
                        n_ans = self.sub_pair(ans, prev)
                        self.chosen_tasks.append(self.ready_tasks[bit])
                        self.backtrack(n_desk_no, full_cups,
                                       empty_cups+1, n_tasks_mask, n_ans)
                        return

    def bit_on(self, mask, bit):
        return (mask >> bit & 1 > 0)

    def bit_set(self, mask, bit):
        return mask | (1 << bit)

    def bit_reset(self, mask, bit):
        return (mask & ~(1 << bit))

    def add_pair(self, t1, t2):
        return [t1[0]+t2[0], t1[1]+t2[1]]

    def sub_pair(self, t1, t2):
        return [t1[0]-t2[0], t1[1]-t2[1]]

    def best_pair(self, t1, t2):
        if t1[0] != t2[0]:
            return t1.copy() if t1[0] > t2[0] else t2.copy()
        return t1.copy() if t1[1] < t2[1] else t2.copy()

    def equal_pair(self, t1, t2):
        return t1[0] == t2[0] and t1[1] == t2[1]

    def bfs(self):
        while len(self.Q) != 0 and self.frame_cnt < self.bfs_steps_per_frame:
            self.frame_cnt += 1
            cur = self.Q.pop(0)
            if cur.is_visited():
                continue
            cur.add_state(CellStatesAvailability.visited)
            for k in range(len(self.dx)):
                idx = cur.i+self.dx[k]
                idy = cur.j+self.dy[k]
                if self.valid_cell(idx, idy) and self.grid.cells[idx][idy].is_unvisited():
                    nxt = self.grid.cells[idx][idy]
                    nxt.add_state(CellStatesAvailability.tovisit)
                    nxt.par = cur
                    nxt.dist = cur.dist+1
                    self.Q.append(nxt)
        self.frame_cnt = 0

    def A_star(self):
        while not self.Q.empty() and self.frame_cnt < self.a_star_steps_per_frame:
            self.frame_cnt += 1
            cur = self.Q.get()
            if cur == self.goal:
                self.path = []
                while cur != None:
                    self.path.append(cur)
                    cur.add_state(CellStatesRobot.path)
                    cur = cur.par
                self.path = self.path[::-1]
                self.goals.pop(0)
                self.set_state(RobotStates.moving)
                return
            if cur.is_visited():
                continue
            cur.add_state(CellStatesAvailability.visited)
            for k in range(len(self.dx)):
                idx = cur.i+self.dx[k]
                idy = cur.j+self.dy[k]
                if self.valid_cell(idx, idy) and self.grid.cells[idx][idy].is_unvisited():
                    nxt = self.grid.cells[idx][idy]
                    nxt.add_state(CellStatesAvailability.tovisit)
                    nxt.par = cur
                    nxt.g = cur.g+1
                    nxt.dist = self.heuristic_score(
                        nxt, self.start, self.goal, 'diagonal')
                    self.Q.put(nxt)
        self.frame_cnt = 0

    def move(self):
        if self.frame_cnt >= self.frames_per_move:
            self.frame_cnt = 0
            if len(self.path) != 0:
                cur = self.path.pop(0)
                self.cell.add_state(CellStatesRobot.path)
                self.i = cur.i
                self.j = cur.j
                self.cell = cur
                cur.add_state(CellStatesRobot.occupied)
                if len(self.path) == 0:
                    if cur.is_ordered():
                        cur.add_state(CellStatesDesks.received)
                    elif cur.is_finished():
                        cur.add_state(CellStatesDesks.neutral)
            else:
                self.set_state(RobotStates.idle)
        else:
            self.frame_cnt += 1

    def euclidean_distance(self, cur, to):
        return round(math.sqrt((cur.i-to.i)*(cur.i-to.i)+(cur.j-to.j)*(cur.j-to.j)))

    def manhatten_distance(self, cur, to):
        return abs(cur.i-to.i)+abs(cur.j-to.j)

    def diagonal_distance(self, cur, to):
        dx = abs(cur.i-to.i)
        dy = abs(cur.j-to.j)
        return round(dx+dy+(math.sqrt(2)-2)*min(dx, dy))

    def heuristic_score(self, cur, start, end, metric='euclidean'):
        if metric == 'euclidean':
            return cur.g+self.euclidean_distance(cur, end)
        elif metric == 'manhatten':
            return cur.g+self.manhatten_distance(cur, end)
        elif metric == 'diagonal':
            return cur.g+self.diagonal_distance(cur, end)

    def valid_cell(self, i, j):
        return i >= 0 and i < self.rows and j >= 0 and j < self.cols
