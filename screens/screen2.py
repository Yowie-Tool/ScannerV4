import kivy
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.gridlayout import GridLayout
from kivy.uix.widget import Widget
from kivy.uix.checkbox import CheckBox

Builder.load_string("""

<YowieScreen2>:

	_90_degree_scan: _90_degree_scan
	_180_degree_scan: _180_degree_scan
	_270_degree_scan: _270_degree_scan
	_360_degree_scan: _360_degree_scan

	low_res_fast: low_res_fast
	full_res_short: full_res_short
	full_res_standard: full_res_standard
	full_res_multiple: full_res_multiple

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
	        orientation: 'vertical'
	        size_hint: (None, None)
	        height: dp(480)
	        width: dp(620)
	        padding: [dp(0), dp(20), dp(0), dp(0)]

	        # Angle options
		    BoxLayout:
		        orientation: 'horizontal'
		        size_hint: (None, None)
		        height: dp(230)
		        width: dp(620)

			    BoxLayout:
			        orientation: 'horizontal'
	        		height: self.parent.width/4
	        		width: self.parent.height
			        canvas.before:
			            PushMatrix
			            Rotate:
			                angle: 90
			                origin: self.center
			        canvas.after:
			            PopMatrix
         

			        CheckBox: 
			        	id: _90_degree_scan
			        	group: 'angle_options'
			        	on_press: root.scan_angle_select()

			        Label: 
			        	color: [0,0,0,1]
			        	text: '90 Degree Scan'

		    	BoxLayout:
			        orientation: 'horizontal'
	        		height: self.parent.width/4
	        		width: self.parent.height
			        canvas.before:
			            PushMatrix
			            Rotate:
			                angle: 90
			                origin: self.center
			        canvas.after:
			            PopMatrix
			        CheckBox:
			        	id: _180_degree_scan
			        	group: 'angle_options'
			        	on_press: root.scan_angle_select()

			        Label:
			        	color: [0,0,0,1]
			        	text: '180 Degree Scan'

			    BoxLayout:
			        orientation: 'horizontal'
	        		height: self.parent.width/4
	        		width: self.parent.height
			        canvas.before:
			            PushMatrix
			            Rotate:
			                angle: 90
			                origin: self.center
			        canvas.after:
			            PopMatrix

			        CheckBox: 
			        	id: _270_degree_scan
			        	group: 'angle_options'
			        	on_press: root.scan_angle_select()

			        Label: 
			        	color: [0,0,0,1]
			        	text: '270 Degree Scan'

		    	BoxLayout:
			        orientation: 'horizontal'
	        		height: self.parent.width/4
	        		width: self.parent.height
			        canvas.before:
			            PushMatrix
			            Rotate:
			                angle: 90
			                origin: self.center
			        canvas.after:
			            PopMatrix
			        CheckBox:
			        	id: _360_degree_scan
			        	group: 'angle_options'
			        	on_press: root.scan_angle_select()
			        Label:
			        	color: [0,0,0,1]
			        	text: '360 Degree Scan'

		    # Resolution options
		    BoxLayout:
		        orientation: 'horizontal'
		        size_hint: (None, None)
		        height: dp(230)
		        width: dp(620)

			    BoxLayout:
			        orientation: 'horizontal'
	        		height: self.parent.width/4
	        		width: self.parent.height
			        canvas.before:
			            PushMatrix
			            Rotate:
			                angle: 90
			                origin: self.center
			        canvas.after:
			            PopMatrix
         

			        CheckBox: 
			        	id: low_res_fast
			        	group: 'resolution_options'

				    BoxLayout:
				        orientation: 'vertical'
				        Label: 
				        	color: [0,0,0,1]
				        	text: 'Low Res'
				        	text_size: self.size
				        	valign: 'bottom'
				        	markup: True
				        Label: 
				        	color: [0,0,0,1]
				        	text: 'Fast Scan'
				        	text_size: self.size
				        	valign: 'top'
				        	markup: True

		    	BoxLayout:
			        orientation: 'horizontal'
	        		height: self.parent.width/4
	        		width: self.parent.height
			        canvas.before:
			            PushMatrix
			            Rotate:
			                angle: 90
			                origin: self.center
			        canvas.after:
			            PopMatrix
			        CheckBox:
			        	id: full_res_short
			        	group: 'resolution_options'

				    BoxLayout:
				        orientation: 'vertical'
				        Label: 
				        	color: [0,0,0,1]
				        	text: 'Full Res'
				        	text_size: self.size
				        	valign: 'bottom'
				        	markup: True
				        Label: 
				        	color: [0,0,0,1]
				        	text: 'Short Range'
				        	text_size: self.size
				        	valign: 'top'
				        	markup: True

			    BoxLayout:
			        orientation: 'horizontal'
	        		height: self.parent.width/4
	        		width: self.parent.height
			        canvas.before:
			            PushMatrix
			            Rotate:
			                angle: 90
			                origin: self.center
			        canvas.after:
			            PopMatrix

			        CheckBox: 
			        	id: full_res_standard
			        	group: 'resolution_options'

				    BoxLayout:
				        orientation: 'vertical'
				        Label: 
				        	color: [0,0,0,1]
				        	text: 'Full Res'
				        	text_size: self.size
				        	valign: 'bottom'
				        	markup: True
				        Label: 
				        	color: [0,0,0,1]
				        	text: 'Standard'
				        	text_size: self.size
				        	valign: 'top'
				        	markup: True

		    	BoxLayout:
			        orientation: 'horizontal'
	        		height: self.parent.width/4
	        		width: self.parent.height
			        canvas.before:
			            PushMatrix
			            Rotate:
			                angle: 90
			                origin: self.center
			        canvas.after:
			            PopMatrix
			        CheckBox:
			        	id: full_res_multiple
			        	group: 'resolution_options'
				    BoxLayout:
				        orientation: 'vertical'
				        Label: 
				        	color: [0,0,0,1]
				        	text: 'Full Res'
				        	text_size: self.size
				        	valign: 'bottom'
				        	markup: True
				        Label: 
				        	color: [0,0,0,1]
				        	text: 'Standard'
				        	text_size: self.size
				        	valign: 'top'
				        	markup: True

""")

class YowieScreen2(Screen):

	def __init__(self, **kwargs):

		self.sm = kwargs['screen_manager']
		self.name = kwargs['name']
		self.m = kwargs['scanner']

		super(YowieScreen2, self).__init__()

# SCREEN BUTTONS
	def go_s1(self):
		self.sm.current = 's1'

	def go_s2(self):
		self.sm.current = 's2'

	def go_s3(self):
		self.sm.current = 's3'

	def go_s4(self):
		self.sm.current = 's4'

# SCAN ANGLE OPTIONS

	def scan_angle_select(self):

		if self._90_degree_scan.state == 'down':
			self.m.scanangle=90
		elif self._180_degree_scan.state == 'down':
			self.m.scanangle=180
		elif self._270_degree_scan.state == 'down':
			self.m.scanangle=270
		elif self._360_degree_scan.state == 'down':
			self.m.scanangle=360


# SCAN RESOLUTION OPTIONS

	def resolution_select(self):
		
		if self.low_res_fast.state == 'down':
			self.m.scanresolution=1
			self.m.scancameras=2
		elif self.full_res_short.state == 'down':
			self.m.scanresolution=1
			self.m.scancameras=1
		elif self.full_res_standard.state == 'down':
			self.m.scanresolution=1
			self.m.scancameras=2
		elif self.full_res_multiple.state == 'down':
			self.m.scanresolution=3
			self.m.scancameras=2
