import kivy
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.gridlayout import GridLayout
from kivy.uix.widget import Widget

from comms import usb_storage

Builder.load_string("""

<YowieScreen4>:

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
	        width: dp(200)


		Button:
	        size_hint: (None, None)
	        height: dp(480)
	        width: dp(200)
	        on_press: root.save_output()
			Label:
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
		        text: "Save"




""")

class YowieScreen4(Screen):

	def __init__(self, **kwargs):

		self.sm = kwargs['screen_manager']
		self.name = kwargs['name']
		self.m = kwargs['scanner']

		super(YowieScreen4, self).__init__()

		self.usb_stick = usb_storage.USB_storage(self.sm)
		self.usb_stick.enable()

# SCREEN BUTTONS
	def go_s1(self):
		self.sm.current = 's1'

	def go_s2(self):
		self.sm.current = 's2'

	def go_s3(self):
		self.sm.current = 's3'

	def go_s4(self):
		self.sm.current = 's4'


# SAVE OUTPUT

	def save_output(self):

		if self.usb_stick.is_usb_mounted_flag == True:

			file_path = self.usb_stick.get_path() + "RoomReaderScan.pts"
	        #currently only one file name. 
			try: 
				file_object = open(file_path, "w")

				print ("Saving points file")
				exportint = len(xdist) # Need to get this from machine object I think
				#currently based on length of x array. Will change to a 2D or 3D array at some point to make the array smaller, as won't have to include 0 values.

				for export in range (exportint):
				    #I've deleted Z and RGB from this now. exports just a text file with X,Y coordinates
				    xout = str(xdist[export])
				    yout = str(ydist[export])
				    file_object.write(xout + " " + yout + "\n")

				file_object.close()

				saveendtime=time.time()
				print("File outputted in %f seconds" % (int(saveendtime-calculateendtime)))

				self.usb_stick.disable()

			except: 
				print('Could not save file')





