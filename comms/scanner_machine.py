from kivy.clock import Clock
import sys, os
from datetime import datetime
import os.path
from os import path, system
from cv2 import imread, subtract, cvtColor, GaussianBlur, minMaxLoc, threshold
import numpy as np
from picamera import PiCamera
import time
import array
import math
import RPi.GPIO as GPIO

from comms import serial_connection
from scipy.special._ufuncs import hyp1f1

def log(message):
	timestamp = datetime.now()
	print (timestamp.strftime('%H:%M:%S.%f' )[:12] + ' ' + str(message))


class ScannerMachine(object):

	chan_listc=[12,16,18] #camera switcher pins
	chan_listl=[29,31,33] #laser pins

	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(self.chan_listc, GPIO.OUT)
	GPIO.setup(self.chan_listl, GPIO.OUT)
	GPIO.output(self.chan_listl,0)

	photonum1 = 0
	photonum2 = 0

	photoangle1=[]
	photoangle2=[]

	current_angle = float(0)
	current_angle_readout = float(0)
	most_recent_angle_change = float(0)

	scanangle=90
	scan_passes=1
	scanrange=1

	scannercalibration=[]

	scanstepscamera1=0
	scanstepscamera2=0
	scanstepstotal=0
	scanstepangle1=float(0)
	scanstepangle2=float(0)

	scan_cameras=1

	for_calc_1=[]
	for_calc_2=[]

	output=[]

	maxdistance=0
	allengths=[]
	low_res = True
	averagedistance=0

	doing_scan_camera_1 = False
	doing_scan_camera_2 = False
	scan_progress = 0

	def __init__(self, screen_manager):

		self.sm = screen_manager

		# Establish 's'erial comms and initialise
		self.s = serial_connection.SerialConnection(self, self.sm)
		self.s.establish_connection()

		global camera
		camera=PiCamera()

	def read_in_calibration_values(self):

		try:
			print('Please wait, reading in calibration values...') 

			file_object=open("CalibrationValues.txt","r")

			int = 0

			while int < 24:

				int += 1

				if (int % 2) == 0:
					self.scannercalibration.append(file_object.readline()) #Camera 1 H FOV 0, V FOV 1, Base 2, Z mount distance 3, X Mount Distance 4, calibration 5, camera 2 H FOV 6, v FOV 67, Base 8, Z mount distance 9, X Mount Distance 10, calibration 11
				
				else:
					throwaway = file_object.readline()

			file_object.close()

		except: 
			print('Could not read in calibration values, please check file!')


	# JOG FUNCTION
	def jog_relative(self, angle):
		strrotate = 'c' + str(angle*30)
		self.s.write_command(strrotate)
		self.most_recent_angle_change = angle

	# GET UP TO DATE VARIABLES
	def update_angle_moved(self):
		fullrotint = math.floor((self.most_recent_angle_change*30)/360)
		fullrotang = fullrotint*360
		self.current_angle = ((self.current_angle_readout)+fullrotang)/30

	def get_scan_progress(self):

		try: 
			if self.scan_cameras == 1:
				self.scan_progress = (self.photonum1/self.scanstepscamera1)

			else:
				self.scan_progress =  (self.photonum1/self.scanstepscamera1) + (self.photonum2/self.scanstepscamera2)
		except:
			self.scan_progress = 0

		return self.scan_progress

	# DO SCAN
	def start_scan(self):
		self.scan_setup()
		self.start_scan_camera_1()
		# NB: star_scan_camera_2 is called in the end_scan() function in order to maintain proper timing

	def stop_scan(self):
		if self.doing_scan_camera_1:
			self.doing_scan_camera_1 = False

		if self.doing_scan_camera_2:
			self.doing_scan_camera_2 = False

	def scan_setup(self):
		self.scanstepscamera1 = math.ceil((self.scanangle*self.scan_passes)/(float(self.scannercalibration[1])))
		self.scanstepscamera2 = math.ceil((self.scanangle*self.scan_passes)/(float(self.scannercalibration[6])))
		self.scanstepangle1 = float(self.scanangle/self.scanstepscamera1)
		self.scanstepangle2 = float(self.scanangle/self.scanstepscamera2)
		if self.scan_cameras == 2:
			self.scanstepstotal = self.scanstepscamera1 + self.scanstepscamera2
		else:
			self.scanstepstotal = self.scanstepscamera1

	def end_scan(self):
		self.camera_close()

		if self.doing_scan_camera_1:
			self.doing_scan_camera_1 = False

			if self.scan_cameras == 2:
				self.set_scanner_to_origin(self.current_angle)
				self.start_scan_camera_2()
				return

		elif self.doing_scan_camera_2:
			self.doing_scan_camera_2 = False

		self.process_scan()

	def set_scanner_to_origin(self,current_angle):
		angle_to_return_to_origin=float(360-current_angle)
		self.jog_relative(angle_to_return_to_origin)
		time.sleep(10) # need to replace this with a flag
		return(current_angle)

	# CAMERA FUNCTIONS
	def camera_1_open(self,resolution_bool):
		i2c='i2cset -y 1 0x70 0x00 0x04' #set camera 1 i2c
		os.system(i2c)
		GPIO.output(self.chan_listc,(1,0,0))
		#select camera 1 GPIO
		if self.resolution_bool:
			camera.resolution=(1640,1232)
		else:
			camera.resolution=(3280,2464)
		camera.meter_mode='backlit'
		camera.start_preview(fullscreen=False,window=(200,80,600,400)) # check that this ends up with our screen in the right place!
		time.sleep() #this needs a value
		
	def camera2open(self,resolution_bool):
		i2c='i2cset -y 1 0x70 0x00 0x06' #set camera 1 i2c
		os.system(i2c)
		GPIO.output(self.chan_listc,(0,1,0))
		#select camera 2 GPIO
		if self.resolution_bool:
			camera.resolution = (1640,1232)
		else:
			camera.resolution = (3280,2464)
		camera.meter_mode = 'backlit'
		camera.start_preview(fullscreen=False,window=(200,80,600,400)) # check that this ends up with our screen in the right place!
		time.sleep() # this neesa a value

	def camera_close(self):
		camera.stop_preview()
		
	def camera_1_take(self,photonum):
		loffname = 'c1loff' + str(photonum) + '.jpg'
		lonname = 'c1on' + str(photonum) + '.jpg'
		expt = camera.exposure_speed
		#query exposure speed from camera
		if expt < 4000:
			camera.shutter_speed=4000
		#sets shutter speed to maximum (minimum speed)
		else:
			camera.shutter_speed=0
		#doesn't override
		camera.capture(loffname,'jpeg',use_video_port=True)
		GPIO.output(self.chan_listl, (1,0,0)) # this needs to be the IR laser, double check when able
		camera.capture(lonname,'jpeg',use_video_port=True)
		camera.shutter_speed=0 # allows camera to adjust after taking both photos
		
	def camera_2_take(self,photonum):
		loffname = 'c2loff' + str(photonum) + '.jpg'
		lonname = 'c2on' + str(photonum) +'.jpg'
		expt = camera.exposure_speed
		#query exposure speed from camera
		if expt < 4000:
			camera.shutter_speed=4000
		#sets shutter speed to maximum (minimum speed)
		else:
			camera.shutter_speed=0
		#doesn't override
		camera.capture(loffname,'jpeg',use_video_port=True)
		GPIO.output(self.chan_listl, (1,0,0)) # this needs to be the IR laser, double check when able
		camera.capture(lonname,'jpeg',use_video_port=True)
		camera.shutter_speed=0 # allows camera to adjust after taking both photos	

	# CAMERA STEPS - THESE GET CALLED IN THE SERIAL LOOP EACH TIME A NEW LINE COMES IN
	def start_scan_camera_1(self):
		self.camera_1_open(self.low_res)
		self.doing_scan_camera_1 = True

	def do_scan_step_camera1(self):
		self.photoangle1.append(self.current_angle)
		self.camera1take(self.photonum1)

		if (self.photonum1 <= self.scanstepscamera1):
			self.jog_relative(self.scanstepangle1)
			self.photonum1 = self.photonum1 + 1

		else:
			self.end_scan()

	def start_scan_camera_2(self):
		self.camera_2_open(self.low_res)
		self.doing_scan_camera_2 = True

	def do_scan_step_2(self):
		self.photoangle2.append(self.current_angle)
		self.camera2take(self.photonum2)

		if self.photonum2 <= self.scanstepscamera2: 
			self.jog_relative(self.scanstepangle2)
			self.photonum2 = self.photonum2 + 1

		else:
			self.end_scan()

	# SCAN PROCESSING

	def process_scan(self):
		self.read_images_1(self.scanstepscamera1,self.photoangle1())
		if self.scan_cameras == 2:
			self.read_images_2(self.scanstepscamera2,self.photoangle2())

		self.calculate_cloud_1(self.for_calc_1,self.scannercalibration)
		if self.scan_cameras == 2:
			self.calculate_cloud_2(self.for_calc_2,self.scannercalibration)

	def weighted_average(self,t):
		tmax=max(t) # highest intensity value in row
		maxvalue=np.argmax(t) # position of value
		length=t.size #length of array, should be equal to Y resolution....
		tcurrent=0
		tlast=tmax
		tint=maxvalue
		if maxvalue > 0:
			while tcurrent < tlast: 
				tlast=t[tint]
				tint=tint-1
				tcurrent=t[tint]
			tminv=tint+1 #the last descending value position in the negative direction
		elif maxvalue == 0:
			tminv=0
		tcurrent=0
		tlast=tmax
		tint=maxvalue
		if maxvalue < length:
			while tcurrent < tlast:
				tlast=t[tint]
				tint=tint+1
				tcurrent=t[tint]
			tmaxv=tint-1 #the last descending value position in the positive direction
		elif maxvalue == length:
			tmaxv=length
		tint2 = tmaxv-tminv+1
		tint3 = tminv
		tint4 = tminv
		tout1=0
		for i in range(tint2):
			tout1=tout1+(tint3*(t[tint3]))
			tint3=tint3+1 # sum of position * intensity in limited array
		tout2=0
		for i2 in range(tint2):
			tout2=tout2+(t[tint4])
			tint4 = tint4 + 1 #sum of intensity in limited array
		if tout1 != (maxvalue*tmax):
			weighted=(tout1/tout2) #weighted value out, if laser line found to be more than 1 pixel wide (to eliminate random points)
		else:
			weighted = 0
				
		return(weighted)
		
	def read_images_1(self,scansteps,photoangle1):
		succesful1=0
		for photographs in range(scansteps):
			pnumstr=str(photographs)
			loffname='1loff' + pnumstr + '.jpg'
			lonname='1lon'+pnumstr + '.jpg'
			#There is a way of taking photo's directly into opencv as an array, but previous attempts at this have been unsuccesful, it seems this only works at low resolutions.
			loff=imread(loffname)
			lon=imread(lonname)
			src=subtract(lon,loff)
			#subtract the laser on image from the laser off image. In theory, when we lock down the camera settings between the two photos, we should end up with just the laser line left. In practice, there is extra interference involved. 
			red=cvtColor(src,cv.COLOR_BGR2GRAY)
			#extract just the red array, used with the visible laser. When we switch to IR, we might change to another channel.
			blur=GaussianBlur(red,(5,5),0)
			#create a blurred image to find the maximum value from. This means that any anomalies are removed
			(minVal, maxVal, MinLoc, maxLoc) = minMaxLoc(blur)
			#find the location of minimum and maximum values in the image
			threshamount = maxVal*0.2 # maybe make the 0.2 a variable, but this was good in testing originally.
			#create a value that will remove any values below that, which is a proportion of the maximum value
			retval, threshold = threshold(red, threshamount, 255, cv.THRESH_TOZERO);
			#this then removes those from the image
			#(minVal, maxVal, MinLoc, maxLoc) = minMaxLoc(threshold) - #not sure if this is needed any more, so commented it out.
			#find the maximum value of the non blurred image
			maxvalue = np.argmax(threshold,axis=1) # Just to check value is nonzero
			#Now find the maximum value in each column. Originally, this was then counted as the centre point of the laser, but we are now going to use a weighted average.
			os.remove(loffname)
			os.remove(lonname)
			#Delete the image files, to save space
			row, col = threshold.shape
			for i in range (row):
				if maxvalue[i] != 0:
					newarray=threshold[[i],:]
					laserctr=self.weighted_average(newarray)
					for_calc_1.append([i,laserctr,(photoangle1[photographs]),0])
					succesful1=succesful1+1
				else: succesful1=succesful1
		print ("short range points captured %d" % (succesful1))
	   
	def read_images_2(self,scansteps,photoangle2):
		succesful2=0
		for photographs in range(scansteps):
			pnumstr=str(photographs)
			loffname='2loff' + pnumstr + '.jpg'
			lonname='2lon'+pnumstr + '.jpg'
			#There is a way of taking photo's directly into opencv as an array, but previous attempts at this have been unsuccesful, it seems this only works at low resolutions.
			loff=imread(loffname)
			lon=imread(lonname)
			src=subtract(lon,loff)
			#subtract the laser on image from the laser off image. In theory, when we lock down the camera settings between the two photos, we should end up with just the laser line left. In practice, there is extra interference involved. 
			red=cvtColor(src,cv.COLOR_BGR2GRAY)
			#extract just the red array, used with the visible laser. When we switch to IR, we might change to another channel.
			blur=GaussianBlur(red,(5,5),0)
			#create a blurred image to find the maximum value from. This means that any anomalies are removed
			(minVal, maxVal, MinLoc, maxLoc) = minMaxLoc(blur)
			#find the location of minimum and maximum values in the image
			threshamount = maxVal*0.2 # maybe make the 0.2 a variable, but this was good in testing originally.
			#create a value that will remove any values below that, which is a proportion of the maximum value
			retval, threshold = threshold(red, threshamount, 255, cv.THRESH_TOZERO);
			#this then removes those from the image
			#(minVal, maxVal, MinLoc, maxLoc) = minMaxLoc(threshold) - #not sure if this is needed any more, so commented it out.
			#find the maximum value of the non blurred image
			maxvalue = np.argmax(threshold,axis=1) # Just to check value is nonzero. We could do this in the weighted average part, but it's potentially quicker here.
			#Now find the maximum value in each column. Originally, this was then counted as the centre point of the laser, but we are now going to use a weighted average.
			os.remove(loffname)
			os.remove(lonname)
			#Delete the image files, to save space
			row, col = threshold.shape
			for i in range (row):
				if maxvalue[i] != 0:
					newarray=threshold[[i],:]
					laserctr=self.weighted_average(newarray)
					for_calc_2.append([i,laserctr,(photoangle2[photographs]),0])
					succesful2=succesful2+1
				else: succesful2=succesful2
		print ("long range points captured %d" % (succesful2))    
		
	def calculate_cloud_1(self,for_calc,calibration):
		readlines=len(for_calc)
		camangleh = ((float(calibration[0]))/(180/math.pi))
		camanglev = ((float(calibration[1]))/(180/math.pi))
		camangled=((float(calibration[2]))/(180/math.pi))
		Bconst = (((180-(float(calibration[0])))/2)/(180/math.pi))
		cosB = math.cos(Bconst)
		laser=float(calibration[3])
		camxoffset=float(calibration[4])
		calib_value=float(calibration[5])
		if self.low_res:
			xresolution=1640
			yresolution=1232
		else:
			xresolution=3280
			yresolution=2464
		halfyres=yresolution/2
		aconst = (xresolution*(math.sin(Bconst)))/(math.sin(camangleh))
		aconstsqrd = math.pow(aconst,2)
		a2const=halfyres/math.tan(camanglev/2)
		for i in range(readlines):
			u=for_calc[i][0] # camera vertical vector - In world view this is actually horizontal
			v=for_calc[i][1] # camera horizontal vector - In world view this is actually vertical
			r=for_calc[i][2] # machine rotation around camera Z axis
			cosC=((2*aconstsqrd)-(2*aconst*(v+1)*cosB))/((2*aconst*(math.sqrt((aconstsqrd+((v+1)*(v+1))-(2*aconst*(v+1)*cosB)))))) # My trusty old calculation
			angle=math.acos(cosC) # find the angle in radians
			totalangle=angle+camangled # add it to the base angle
			y1=(laser*(math.tan(totalangle)))+calib_value #Y (world) value before the rotation taken into account
			w=math.sqrt((math.pow(y1,2))+(math.pow(laser,2))) # camera w vector 
			if u < halfyres:
				u2=halfyres-u
				theta=math.atan(u2/a2const)
				x1=(-(w*math.tan(theta)))+camxoffset # x would be a - value where it falls less than half way across CCD, then offset by amount camera is off centre
			if u == halfyres:
				x1=camxoffset # would be 0 where it falls in the centre of the CCD, then offset by the amount the camera is off centre
			if u > halfyres:
				u2=u-halfyres
				theta=math.atan(u2/a2const)
				x1=(w*math.tan(theta))+camxoffset # x would be a * value where it falls over halfway across CCD, then offset by amount camera is off centre
			hyp1=math.sqrt((math.pow(x1,2))+(math.pow(y1,2))) # Find the hypotenuse of the newly created triangle of points (where opposite = x, adjacent =y)
			if hyp1 > maxdistance:
				maxdistance=hyp1
			alllengths.append(hpy1)
			averagedistance=numpy.mean(alllengths)
			tan1=math.atan(x1/y1) #find theta angle for new triangle				
			rrad=r/(180/math.pi) # rotation of unit in radians
			xout=hyp1*math.sin(rrad+tan1) # x output adjusted for rotation around Z axis
			yout=hyp1*math.cos(rrad+tan1) # y output adjusted for rotation around Z axis
			output.append([xout,yout,0]) # X, Y, Z coordinates for output. Z is assumed to be 0 for easy import into CAD
		
	def calculate_cloud_2(self,for_calc,calibration):
		readlines=len(for_calc)
		camangleh = ((float(calibration[6]))/(180/math.pi))
		camanglev = ((float(calibration[7]))/(180/math.pi))
		camangled=((float(calibration[8]))/(180/math.pi))
		Bconst = (((180-(float(calibration[6])))/2)/(180/math.pi))
		cosB = math.cos(Bconst)
		laser = float(calibration[9])
		camxoffset = float(calibration[10])
		calib_value = float(calibration[11])
		if self.low_res:
			xresolution = 1640
			yresolution = 1232
		else:
			xresolution = 3280
			yresolution = 2464
		halfyres = yresolution/2
		aconst = (xresolution*(math.sin(Bconst)))/(math.sin(camangleh))
		aconstsqrd = math.pow(aconst,2)
		a2const = halfyres/math.tan(camanglev/2)
		for i in range(readlines):
			u = for_calc[i][0] # camera vertical vector - In world view this is actually horizontal
			v = for_calc[i][1] # camera horizontal vector - In world view this is actually vertical
			r = for_calc[i][2] # machine rotation around camera Z axis
			cosC = ((2*aconstsqrd)-(2*aconst*(v+1)*cosB))/((2*aconst*(math.sqrt((aconstsqrd+((v+1)*(v+1))-(2*aconst*(v+1)*cosB)))))) # My trusty old calculation
			angle = math.acos(cosC) # find the angle in radians
			totalangle = angle+camangled # add it to the base angle
			y1 = (laser*(math.tan(totalangle)))+calib_value #Y (world) value before the rotation taken into account
			w = math.sqrt((y1*y1)+(laser*laser)) # camera w vector 
			if u < halfyres:
				u2 = halfyres-u
				theta = math.atan(u2/a2const)
				x1 = (-(w*math.tan(theta)))+camxoffset
			if u == halfyres:
				x1 = camxoffset
			if u > halfyres:
				u2=u-halfyres
				theta=math.atan(u2/a2const)
				x1=(w*math.tan(theta))+camxoffset				
			hyp1=math.sqrt((math.pow(x1,2))+(math.pow(y1,2))) # Find the hypotenuse of the newly created triangle of points (where opposite = x, adjacent =y)
			if hyp1 > maxdistance:
				maxdistance=hyp1
			alllengths.append(hpy1)
			averagedistance=numpy.mean(alllengths)
			tan1=math.atan(x1/y1) #find theta angle for new triangle				
			rrad=r/(180/math.pi) # rotation of unit in radians
			xout=hyp1*math.sin(rrad+tan1) # x output adjusted for rotation around Z axis
			yout=hyp1*math.cos(rrad+tan1) # y output adjusted for rotation around Z axis
			output.append([xout,yout,0]) # X, Y, Z coordinates for output. Z is assumed to be 0 for easy import into CAD		
				