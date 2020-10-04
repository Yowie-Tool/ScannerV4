import time
import sys, os
from datetime import datetime
import os.path
from os import path

os.environ['KIVY_WINDOW'] = 'sdl2'

from kivy.config import Config
from kivy.clock import Clock
Config.set('kivy', 'keyboard_mode', 'systemanddock')
Config.set('graphics', 'width', '800')
Config.set('graphics', 'height', '480')
Config.set('graphics', 'maxfps', '60')
Config.set('kivy', 'KIVY_CLOCK', 'interrupt')
Config.write()

import kivy
from kivy.app import App
from kivy.uix.screenmanager import ScreenManager, Screen, NoTransition
from kivy.core.window import Window

from screens import screen1, screen2, screen3, screen4
from comms import scanner_machine


class YowieApp(App):


	def build(self):

		sm = ScreenManager(transition=NoTransition())
		m = scanner_machine.ScannerMachine(sm)

		sm.add_widget(screen1.YowieScreen1(name='s1', screen_manager = sm, scanner = m))
		sm.add_widget(screen2.YowieScreen2(name='s2', screen_manager = sm, scanner = m))
		sm.add_widget(screen3.YowieScreen3(name='s3', screen_manager = sm, scanner = m))
		sm.add_widget(screen4.YowieScreen4(name='s4', screen_manager = sm, scanner = m))
		sm.current = 's1'
		return sm

YowieApp().run()