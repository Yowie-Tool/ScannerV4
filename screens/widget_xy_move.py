'''
Created on 1 Feb 2018
@author: Ed
'''

import kivy
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen, NoTransition
from kivy.uix.floatlayout import FloatLayout
from kivy.properties import ObjectProperty, ListProperty, NumericProperty # @UnresolvedImport
from kivy.uix.widget import Widget
from kivy.base import runTouchApp
from kivy.clock import Clock

Builder.load_string("""

#:import hex kivy.utils.get_color_from_hex

<XYMove>

	jogModeButtonImage:jogModeButtonImage

	BoxLayout:

		size: self.parent.size
		pos: self.parent.pos      
		orientation: 'vertical'
		padding: 10
		spacing: 10
		# center: self.parent.center

		GridLayout:
			cols: 3
			orientation: 'horizontal'
			spacing: 0
			size_hint_y: None
			height: self.width
			center: self.parent.center

			BoxLayout:
				padding: 10
				size: self.parent.size
				pos: self.parent.pos                              

			Button:
				background_color: hex('#F4433600')
				always_release: True
				# on_release: 
				#     root.cancelXYJog()
				#     self.background_color = hex('#F4433600')
				on_press: 
					print('press')
					root.buttonJogXY('X+')
					# self.background_color = hex('#F44336FF')
				BoxLayout:
					padding: 0
					size: self.parent.size
					pos: self.parent.pos
					Image:
						source: "./screens/img/xy_arrow_up.png"
						center_x: self.parent.center_x
						y: self.parent.y
						size: self.parent.width, self.parent.height
						allow_stretch: True                                    

			BoxLayout:
				padding: 10
				size: self.parent.size
				pos: self.parent.pos

			Button:
				background_color: hex('#F4433600')
				always_release: True
				disabled: True
				opacity: 0
		  
			Button:
				background_color: hex('#F4433600')
				on_release: 
					self.background_color = hex('#F4433600')
				on_press:
					root.jogModeCycled()
					self.background_color = hex('#F44336FF')
				BoxLayout:
					padding: 0
					size: self.parent.size
					pos: self.parent.pos
					Image:
						id: jogModeButtonImage
						source: "./screens/img/jog_mode_180.png"
						center_x: self.parent.center_x
						y: self.parent.y
						size: self.parent.width, self.parent.height
						allow_stretch: True  
			Button:
				background_color: hex('#F4433600')
				always_release: True
				disabled: True
				opacity: 0

			BoxLayout:
				padding: 10
				size: self.parent.size
				pos: self.parent.pos                 

			Button:
				background_color: hex('#F4433600')
				always_release: True
				# on_release:
				#     print('release')
				#     root.cancelXYJog()
				#     self.background_color = hex('#F4433600')
				on_press: 
					print('press')
					root.buttonJogXY('X-')
					# self.background_color = hex('#F44336FF')
				BoxLayout:
					padding: 0
					size: self.parent.size
					pos: self.parent.pos      
					Image:
						source: "./screens/img/xy_arrow_down.png"
						center_x: self.parent.center_x
						y: self.parent.y
						size: self.parent.width, self.parent.height
						allow_stretch: True

			BoxLayout:
				padding: 10
				size: self.parent.size
				pos: self.parent.pos

""")

class XYMove(Widget):

	def __init__(self, **kwargs):

		super(XYMove, self).__init__()
		self.m=kwargs['scanner']
		self.sm=kwargs['screen_manager']

	jogMode = 'plus_180'
	jog_mode_button_press_counter = 0

	# JOG SCANNER BY ANGLES OF 180, 20, 10, 5

	def jogModeCycled(self):

		self.jog_mode_button_press_counter += 1
		if self.jog_mode_button_press_counter % 4 == 0: 
			self.jogMode = 'plus_180'
			self.jogModeButtonImage.source = './screens/img/jog_mode_180.png'
		if self.jog_mode_button_press_counter % 4 == 1: 
			self.jogMode = 'plus_20'
			self.jogModeButtonImage.source = './screens/img/jog_mode_20.png'
		if self.jog_mode_button_press_counter % 4 == 2: 
			self.jogMode = 'plus_10'
			self.jogModeButtonImage.source = './screens/img/jog_mode_10.png'
		if self.jog_mode_button_press_counter % 4 == 3: 
			self.jogMode = 'plus_5'
			self.jogModeButtonImage.source = './screens/img/jog_mode_5.png'

	def buttonJogXY(self, case):

		if self.jogMode == 'plus_5':
			if case == 'X-': self.m.jog_clockwise(5)
			if case == 'X+': self.m.jog_anticlockwise(5)

		elif self.jogMode == 'plus_10':
			if case == 'X-': self.m.jog_clockwise(10)
			if case == 'X+': self.m.jog_anticlockwise(10)

		elif self.jogMode == 'plus_20':
			if case == 'X-': self.m.jog_clockwise(20)
			if case == 'X+': self.m.jog_anticlockwise(20)

		elif self.jogMode == 'plus_180':
			if case == 'X-': self.m.jog_clockwise(180)
			if case == 'X+': self.m.jog_anticlockwise(180)
