from kivy.uix.gridlayout import GridLayout
from kivy.properties import NumericProperty
from Cells import *
from State import CellStatesDesks


class SquareGrid(GridLayout):
    grid_size = NumericProperty(0)

    def __init__(self, sz=40, grid=None, **kwargs):
        super().__init__(**kwargs)
        if grid == None:
            self.grid_size = sz
            self.rows = self.grid_size
            self.cols = self.grid_size
            self.create_cells()
            self.desks_coords = [(0, 0), (0, 39), (39, 0),
                                 (29, 15), (19, 19), (12, 30)]
            self.desks = [self.cells[dc[0]][dc[1]] for dc in self.desks_coords]
            for i, desk in enumerate(self.desks):
                desk.add_state(CellStatesDesks.neutral)
                desk.idx = i
        else:
            self.grid_size = len(grid)
            self.rows = self.grid_size
            self.cols = self.grid_size
            self.create_loaded_cells(grid)

    def create_cells(self):
        # creating the cells matrix
        self.cells = [[SquareCell(i, j, self.grid_size) for j in range(
            self.grid_size)] for i in range(self.grid_size)]

        # adding the cells to the grid layout
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                self.add_widget(self.cells[i][j])

    def create_loaded_cells(self, cell_states):
        self.cells = [[SquareCell(i, j, self.grid_size, cell_states[i][j]) for j in range(
            self.grid_size)] for i in range(self.grid_size)]
        machine = None
        self.desks = []
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                self.add_widget(self.cells[i][j])
                if self.cells[i][j].is_desk():
                    self.cells[i][j].idx = len(self.desks)
                    self.desks.append(self.cells[i][j])
                if self.cells[i][j].is_machine():
                    machine = self.cells[i][j]
        if machine:
            self.desks.append(machine)
            machine.idx = len(self.desks)-1

    def reset_cells(self):
        for _ in self.cells:
            for cell in _:
                cell.reset()
