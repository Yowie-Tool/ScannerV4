import kivy
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.gridlayout import GridLayout
from kivy.uix.widget import Widget
from kivy.clock import Clock
from kivy.uix.behaviors import ToggleButtonBehavior

Builder.load_string("""

<YowieScreen3>:

	scan_progress_output: scan_progress_output
	scan_toggle: scan_toggle
	scan_toggle_label: scan_toggle_label
	current_angle_output: current_angle_output
	average_distance_output: average_distance_output
	max_distance_output: max_distance_output
	points_rec_output: points_rec_output
	pass_output: pass_output
	scan_time_output: scan_time_output

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


		# CAMERA SCAN

		BoxLayout:
			size_hint: (None, None)
			height: dp(480)
			width: dp(620)
			orientation: 'horizontal'

			BoxLayout:
				size_hint: (None, None)
				height: dp(480)
				width: dp(20)
				Label:
					id: scan_progress_output
					center: self.parent.center
					color: [0,0,0,1]
					height: self.parent.width
					canvas.before:
						PushMatrix
						Rotate:
							angle: 90
							origin: self.center
					canvas.after:
						PopMatrix
					# text_size: self.size
					text: 'Scan progress: 0%'


			BoxLayout:
				size_hint: (None, None)
				height: dp(480)
				width: dp(600)
				orientation: 'vertical'

				BoxLayout:
					size_hint: (None, None)
					height: dp(400)
					width: dp(600)

				BoxLayout:
					size_hint: (None, None)
					height: dp(80)
					width: dp(600)
					orientation: 'horizontal'

					ToggleButton:
						id: scan_toggle
						on_press: root.start_stop_scan()
						Label:
							id: scan_toggle_label
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
							text: 'Start Scan'

					Label:
						center: self.parent.center
						color: [0,0,0,1]
						height: self.parent.width
						canvas.before:
							PushMatrix
							Rotate:
								angle: 90
								origin: self.center
						canvas.after:
							PopMatrix
						font_size: '12sp'
						valign: 'middle'
						text: 'Current Angle'
						text_size: self.size

					Label:
						id: current_angle_output
						center: self.parent.center
						color: [0,0,0,1]
						height: self.parent.width
						canvas.before:
							PushMatrix
							Rotate:
								angle: 90
								origin: self.center
						canvas.after:
							PopMatrix
						text: 'XX'
						font_size: '14sp'
						valign: 'middle'
						text_size: self.size

					Label:
						center: self.parent.center
						color: [0,0,0,1]
						height: self.parent.width
						canvas.before:
							PushMatrix
							Rotate:
								angle: 90
								origin: self.center
						canvas.after:
							PopMatrix
						text_size: self.size
						font_size: '12sp'
						valign: 'middle'
						text: 'Average distance'

					Label:
						id: average_distance_output
						center: self.parent.center
						color: [0,0,0,1]
						height: self.parent.width
						canvas.before:
							PushMatrix
							Rotate:
								angle: 90
								origin: self.center
						canvas.after:
							PopMatrix
						text: 'XX'
						text_size: self.size
						font_size: '14sp'
						valign: 'middle'

					Label:
						center: self.parent.center
						color: [0,0,0,1]
						height: self.parent.width
						canvas.before:
							PushMatrix
							Rotate:
								angle: 90
								origin: self.center
						canvas.after:
							PopMatrix
						text_size: self.size
						text: 'Max distance'
						font_size: '12sp'
						valign: 'middle'

					Label:
						id: max_distance_output
						center: self.parent.center
						color: [0,0,0,1]
						height: self.parent.width
						canvas.before:
							PushMatrix
							Rotate:
								angle: 90
								origin: self.center
						canvas.after:
							PopMatrix
						text: 'XX'
						text_size: self.size
						font_size: '14sp'
						valign: 'middle'

					Label:
						center: self.parent.center
						color: [0,0,0,1]
						height: self.parent.width
						canvas.before:
							PushMatrix
							Rotate:
								angle: 90
								origin: self.center
						canvas.after:
							PopMatrix
						text_size: self.size
						text: 'Points Rec'
						font_size: '16sp'
						valign: 'middle'

					Label:
						id: points_rec_output
						center: self.parent.center
						color: [0,0,0,1]
						height: self.parent.width
						canvas.before:
							PushMatrix
							Rotate:
								angle: 90
								origin: self.center
						canvas.after:
							PopMatrix
						text: 'XX'
						text_size: self.size
						font_size: '14sp'
						valign: 'middle'

					Label:
						center: self.parent.center
						color: [0,0,0,1]
						height: self.parent.width
						canvas.before:
							PushMatrix
							Rotate:
								angle: 90
								origin: self.center
						canvas.after:
							PopMatrix
						text_size: self.size
						text: 'Pass'
						font_size: '16sp'
						valign: 'middle'

					Label:
						id: pass_output
						center: self.parent.center
						color: [0,0,0,1]
						height: self.parent.width
						canvas.before:
							PushMatrix
							Rotate:
								angle: 90
								origin: self.center
						canvas.after:
							PopMatrix
						text: 'XX'
						text_size: self.size
						font_size: '14sp'
						valign: 'middle'

					Label:
						id: scan_time_output
						center: self.parent.center
						color: [0,0,0,1]
						height: self.parent.width
						canvas.before:
							PushMatrix
							Rotate:
								angle: 90
								origin: self.center
						canvas.after:
							PopMatrix
						text_size: self.size
						text: 'Scan time'
						font_size: '16sp'
						valign: 'middle'

					Label:
						center: self.parent.center
						color: [0,0,0,1]
						height: self.parent.width
						canvas.before:
							PushMatrix
							Rotate:
								angle: 90
								origin: self.center
						canvas.after:
							PopMatrix
						text_size: self.size
						text: 'XX:XX'
						font_size: '14sp'
						valign: 'middle'


""")

class YowieScreen3(Screen):

	def __init__(self, **kwargs):

		self.sm = kwargs['screen_manager']
		self.name = kwargs['name']
		self.m = kwargs['scanner']

		super(YowieScreen3, self).__init__()

# SCREEN BUTTONS
	def go_s1(self):
		self.sm.current = 's1'

	def go_s2(self):
		self.sm.current = 's2'

	def go_s3(self):
		self.sm.current = 's3'

	def go_s4(self):
		self.sm.current = 's4'

# ON ENTER START UPDATING SCAN OUTPUT

	def on_enter(self):
		self.update_scan_info_event = Clock.schedule_interval(self.update_all_outputs, 0.2)

# START SCAN
	def start_stop_scan(self):
		if self.scan_toggle.state == 'down':
			self.m.start_scan()
			self.scan_toggle_label.text = 'Stop scan'

		if self.scan_toggle.state == 'normal':
			self.m.stop_scan()
			self.scan_toggle_label.text = 'Start scan'


# SCAN OUTPUT

	# INSERT RELEVANT VARIABLES THAT COME OUT OF SERIAL:

	def update_all_outputs(self, dt):
		# self.update_scan_progress_output()
		self.update_current_angle_output()
		self.update_average_distance_output()
		self.update_max_distance_output()
		self.update_points_rec_output()
		self.update_pass_output()
		self.update_scan_time_output()

	def update_scan_progress_output(self, string_input):
		
		self.scan_progress_output.text = string_input

	def update_current_angle_output(self):

		self.current_angle_output.text = str(self.m.current_angle_readout)

	def update_average_distance_output(self):

		self.average_distance_output.text = str(int(self.m.averagedistance))

	def update_max_distance_output(self):

		self.max_distance_output.text = str(int(self.m.maxdistance))

	def update_points_rec_output(self):

		self.points_rec_output.text = ''

	def update_pass_output(self):

		self.pass_output.text = ''

	def update_scan_time_output(self):

		self.scan_time_output.text = ''

# ON LEAVE STOP UPDATING SCREEN

	def on_leave(self):
		Clock.unschedule(self.update_scan_info_event)