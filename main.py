from kivy.app import App
from kivy.uix.screenmanager import ScreenManager
from kivy.uix.screenmanager import Screen
from kivy.clock import Clock
from kivy.uix.boxlayout import BoxLayout
from Grids import SquareGrid
from kivy.graphics import Rectangle
from kivy.graphics import Color
from Machines import CoffeeMachine
from Robots import CoffeeRobot
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.config import Config
import State
from State import CellStatesAvailability, CellStatesTasks, CellStatesDesks, CellStatesRobot, CellColors
from functools import partial
import pickle
from Labels import ColorLabel

screen_manager = ScreenManager()


class Game(BoxLayout):
    def __init__(self, fps, ** kwargs):
        super().__init__(**kwargs)
        self.fps = fps
        self.grid = SquareGrid(40)
        self.add_widget(self.grid)
        self.coffeemachine = CoffeeMachine(12, 30, self.grid, self.fps)
        self.robot = CoffeeRobot(
            15, 15, self.grid, self.coffeemachine, self.fps)
        self.create_user_interface()

    def update(self, dt):
        self.play()

    def play(self):
        self.robot.update()
        self.coffeemachine.update()

    def create_user_interface(self):
        UILayout = BoxLayout(size_hint=(0.2, 1), orientation='vertical')
        edit_btn = Button(
            text='Edit', size_hint=(1, 0.2), on_press=self.goto_edit)
        load_btn = Button(
            text='Load', size_hint=(1, 0.2), on_press=self.load_grid)
        stop_btn = Button(
            text='Stop/Play', size_hint=(1, 0.2), on_press=self.stop_play)
        states = [CellStatesDesks.neutral, CellStatesDesks.ordered,
                  CellStatesDesks.received, CellStatesDesks.finished, CellStatesDesks.machine, CellStatesRobot.occupied]
        for state in states:
            UILayout.add_widget(ColorLabel(text=state.name, color=(
                1, 1, 1, 1), background_color=CellColors.color[state], size_hint=(1, 0.2)))
        UILayout.add_widget(edit_btn)
        UILayout.add_widget(load_btn)
        UILayout.add_widget(stop_btn)
        self.UILayout = UILayout
        self.add_widget(self.UILayout)

    def goto_edit(self, a):
        screen_manager.current = 'edit'

    def stop_play(self, a):
        if State.STOPPED:
            State.STOPPED = False
            State.UPDATING = True
            Clock.schedule_interval(self.update, 1.0/self.fps)
        else:
            State.STOPPED = True
            State.UPDATING = False
            Clock.unschedule(self.update)

    def load_grid(self, a=None):
        Clock.unschedule(self.update)
        State.UPDATING = False

        grid_file_name = 'grid.pkl'
        file = open(grid_file_name, 'rb')
        grid = pickle.load(file)
        self.remove_widget(self.grid)
        self.remove_widget(self.UILayout)
        self.grid = SquareGrid(grid=grid)
        self.add_widget(self.grid)
        machine = None
        robot = None
        for _ in self.grid.cells:
            for cell in _:
                if cell.is_machine():
                    machine = cell
                if cell.is_robot():
                    robot = cell
        self.coffeemachine = CoffeeMachine(
            machine.i, machine.j, self.grid, self.fps)
        self.robot = CoffeeRobot(
            robot.i, robot.j, self.grid, self.coffeemachine, self.fps)
        self.create_user_interface()
        file.close()

        Clock.schedule_interval(self.update, 1.0/self.fps)
        State.UPDATING = True


class GameScreen(Screen):
    def __init__(self, fps, **kw):
        super().__init__(**kw)
        self.fps = fps
        self.game = Game(self.fps)
        self.add_widget(self.game)


class Edit(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # self.add_widget(BoxLayout())
        self.fps = 120
        self.grid = SquareGrid(40)
        self.add_widget(self.grid)
        self.create_user_interface()

    def create_user_interface(self):
        UILayout = BoxLayout(size_hint=(0.2, 1), orientation='vertical')
        game_btn = Button(
            text='Game', size_hint=(1, 0.2), on_press=self.goto_game)
        save_btn = Button(
            text='Save', size_hint=(1, 0.2), on_press=self.save_grid)
        load_btn = Button(
            text='Load', size_hint=(1, 0.2), on_press=self.load_grid)

        states = [CellStatesAvailability.available, CellStatesAvailability.unavailable,
                  CellStatesAvailability.blocked, CellStatesDesks.neutral, CellStatesDesks.ordered]
        state_btns = [Button(text=state.name, size_hint=(1, 0.2), on_press=partial(self.change_editing_state, state), background_color=CellColors.color[state], color=(1, 0, 0, 1))
                      for state in states]
        machine_btn = Button(
            text='Machine', size_hint=(1, 0.2), on_press=self.add_machine, background_color=CellColors.color[CellStatesDesks.machine])
        robot_btn = Button(
            text='Robot', size_hint=(1, 0.2), on_press=self.add_robot, background_color=CellColors.color[CellStatesRobot.occupied])
        for btn in state_btns:
            UILayout.add_widget(btn)
        UILayout.add_widget(robot_btn)
        UILayout.add_widget(machine_btn)
        UILayout.add_widget(game_btn)
        UILayout.add_widget(save_btn)
        UILayout.add_widget(load_btn)
        self.UILayout = UILayout
        self.add_widget(self.UILayout)

    def goto_game(self, a):
        screen_manager.current = 'game'
        # Clock.unschedule(game.game.update)

    def add_machine(self, a):
        for _ in self.grid.cells:
            for cell in _:
                if cell.is_machine():
                    cell.clear()
                    break
        self.change_editing_state(CellStatesDesks.machine)

    def add_robot(self, a):
        for _ in self.grid.cells:
            for cell in _:
                if cell.is_robot():
                    cell.clear()
                    break
        self.change_editing_state(CellStatesRobot.occupied)

    def save_grid(self, a):
        file = open('grid.pkl', 'wb')
        grid_cell_states = []
        for _ in self.grid.cells:
            grid_cell_states.append([cell.states for cell in _])
        pickle.dump(grid_cell_states, file)
        file.close()

    def change_editing_state(self, state, a=None):
        State.EDITING_STATE = state

    def load_grid(self, a=None):

        grid_file_name = 'grid.pkl'
        file = open(grid_file_name, 'rb')
        grid = pickle.load(file)
        self.remove_widget(self.grid)
        self.remove_widget(self.UILayout)
        self.grid = SquareGrid(grid=grid)
        self.add_widget(self.grid)
        machine = None
        robot = None
        for _ in self.grid.cells:
            for cell in _:
                if cell.is_machine():
                    machine = cell
                if cell.is_robot():
                    robot = cell
        self.coffeemachine = CoffeeMachine(
            machine.i, machine.j, self.grid, self.fps)
        self.robot = CoffeeRobot(
            robot.i, robot.j, self.grid, self.coffeemachine, self.fps)
        self.create_user_interface()
        file.close()


class EditScreen(Screen):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.edit = Edit()
        self.add_widget(self.edit)


class CoffeeRobotApp(App):
    def build(self):
        self.fps = 120
        self.game = GameScreen(self.fps, name='game')
        self.edit = EditScreen(name='edit')
        screen_manager.add_widget(self.game)
        screen_manager.add_widget(self.edit)
        screen_manager.current = 'game'
        Clock.schedule_interval(self.check_screen, 1.0/self.fps)
        return screen_manager

    def check_screen(self, dt):
        if screen_manager.current == 'game':
            if not State.UPDATING and not State.STOPPED:
                Clock.schedule_interval(self.game.game.update, 1.0/self.fps)
                State.UPDATING = True
        elif screen_manager.current == 'edit':
            if State.UPDATING:
                Clock.unschedule(self.game.game.update)
                State.UPDATING = False


Config.set('input', 'mouse', 'mouse,multitouch_on_demand')
if __name__ == '__main__':
    CoffeeRobotApp().run()
