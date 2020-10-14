import kivy
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.gridlayout import GridLayout
from kivy.uix.widget import Widget
from kivy.uix.behaviors import ToggleButtonBehavior
from kivy.clock import Clock

from screens import widget_xy_move

Builder.load_string("""

<YowieScreen1>:

	xy_move_container: xy_move_container
	xy_label: xy_label
	laser_toggle: laser_toggle
	laser_toggle_label: laser_toggle_label

    canvas:
        Color: 
            rgba: [1, 1, 1, 1]
        Rectangle: 
            size: self.size
            pos: self.pos

    BoxLayout:
        orientation: 'horizontal'
        padding: 0
        spacing: 0
        size_hint: (None, None)
        height: dp(480)
        width: dp(800)

        # SCREEN BUTTONS
	    BoxLayout:
	        orientation: 'horizontal'
	        size_hint: (None, None)
	        height: dp(480)
	        width: dp(180)

	        GridLayout:
		        size: self.parent.size
		        pos: self.parent.pos
		        cols: 2

		        # Top right
	        	Button:
	        		on_press: root.go_s2()
	        		Label:
	        			center: self.parent.center
	        			color: [1,1,1,1]
		        		height: self.parent.width
				        canvas.before:
				            PushMatrix
				            Rotate:
				                angle: 90
				                origin: self.center
				        canvas.after:
				            PopMatrix
		        		text: 'Scan Settings'

		        # Bottom right
	        	Button:
	        		on_press: root.go_s4()
	        		Label:
	        			center: self.parent.center
	        			color: [1,1,1,1]
		        		height: self.parent.width
				        canvas.before:
				            PushMatrix
				            Rotate:
				                angle: 90
				                origin: self.center
				        canvas.after:
				            PopMatrix
		        		text: 'Scan Output'

		        # Top left
	        	Button:
	        		on_press: root.go_s1()
	        		Label:
	        			center: self.parent.center
	        			color: [1,1,1,1]
		        		height: self.parent.width
				        canvas.before:
				            PushMatrix
				            Rotate:
				                angle: 90
				                origin: self.center
				        canvas.after:
				            PopMatrix
		        		text: 'Levelling'

		        # Bottom left
	        	Button:
	        		on_press: root.go_s3()
	        		Label:
	        			center: self.parent.center
	        			color: [1,1,1,1]
		        		height: self.parent.width
				        canvas.before:
				            PushMatrix
				            Rotate:
				                angle: 90
				                origin: self.center
				        canvas.after:
				            PopMatrix
		        		text: 'Scan'

	    BoxLayout:
	        orientation: 'horizontal'
	        size_hint: (None, None)
	        height: dp(480)
	        width: dp(100)
			Label:
				id: xy_label
    			center: self.parent.center
    			color: [0,0,0,1]
        		height: self.parent.width
        		width: self.parent.height
		        canvas.before:
		            PushMatrix
		            Rotate:
		                angle: 90
		                origin: self.center
		        canvas.after:
		            PopMatrix
		        markup: True
		        font_size: '22sp'
				text: "X-Y label"


		ToggleButton:
			id: laser_toggle
	        size_hint: (None, None)
	        height: dp(480)
	        width: dp(50)
	        on_press: root.switch_laser_light()
			Label:
				id: laser_toggle_label
    			center: self.parent.center
    			color: [1,1,1,1]
        		height: self.parent.width
        		width: self.parent.height
		        canvas.before:
		            PushMatrix
		            Rotate:
		                angle: 90
		                origin: self.center
		        canvas.after:
		            PopMatrix
		        markup: True
		        font_size: '22sp'
		        text: "Visible light"

		# X Y widget

	    BoxLayout:
	        orientation: 'horizontal'
	        padding: 20
	        spacing: 20
	        size_hint: (None, None)
	        height: dp(480)
	        width: dp(470)
	        id: xy_move_container


""")

class YowieScreen1(Screen):

	def __init__(self, **kwargs):
		super(YowieScreen1, self).__init__()

		self.sm = kwargs['screen_manager']
		self.name = kwargs['name']
		self.m = kwargs['scanner']
		self.xy_move_widget = widget_xy_move.XYMove(screen_manager = self.sm, scanner = self.m)
		self.xy_move_container.add_widget(self.xy_move_widget)

# SCREEN BUTTONS
	def go_s1(self):
		self.sm.current = 's1'

	def go_s2(self):
		self.sm.current = 's2'

	def go_s3(self):
		self.sm.current = 's3'

	def go_s4(self):
		self.sm.current = 's4'

# XY Move label update

	def on_enter(self):
		# UPDATES X-Y LABEL EVERY 0.2 SECONDS
		self.xy_label_update_event = Clock.schedule_interval(self.update_xy_label, 0.2)

	def update_xy_label(self, dt):

		# INSERT RELEVANT VARIABLE THAT COMES OUT OF SERIAL:

		self.xy_label.text = 'XXX YYY'

# LASER TOGGLE BUTTON
	def switch_laser_light(self):
		if self.laser_toggle.state == 'normal':
			self.laser_toggle_label.text = 'Visible laser on'
			self.m.visible_laser_on()

		if self.laser_toggle.state == 'down':
			self.laser_toggle_label.text = 'Visible laser off'
			self.m.visible_laser_off()


# ON LEAVE STOP UPDATING SCREEN

	def on_leave(self):
		Clock.unschedule(self.xy_label_update_event)

