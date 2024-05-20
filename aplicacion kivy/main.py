from kivymd.app import MDApp
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager

class DemoProyect(ScreenManager):
    pass

class Main(MDApp):
    def build(self):
        Builder.load_file("main.kv") #llama a la aplicación visual
        return DemoProyect()

Main().run()