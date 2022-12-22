from kivy.uix.boxlayout import BoxLayout
from kivy.properties import StringProperty, ColorProperty


class ColorLabel(BoxLayout):
    text = StringProperty('')
    color = ColorProperty()
    background_color = ColorProperty()

    def __init__(self, text, color, background_color, size_hint, ** kwargs):
        super().__init__(**kwargs)
        self.text = text
        self.color = color
        self.background_color = background_color
        self.size_hint = size_hint
