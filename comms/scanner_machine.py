
from kivy.clock import Clock
import sys, os
from datetime import datetime
import os.path
from os import path, system
from cv2 import imread, subtract, cvtColor, GaussianBlur, MinMaxLoc, threshold
import numpy as np
from picamera import PiCamera
import time
import array
import math
import RPi.GPIO as GPIO

from comms import serial_connection

def log(message):
	timestamp = datetime.now()
	print (timestamp.strftime('%H:%M:%S.%f' )[:12] + ' ' + str(message))


class ScannerMachine(object):
	GPIO.setmode(GPIO.BOARD)
	chan_listc=[12,16,18] #camera switcher pins
	chan_listl=[29,31,33] #laser pins
	GPIO.setup(chan_listc, GPIO.OUT)
	GPIO.setup(chan_listl, GPIO.OUT)
	GPIO.output(chan_listl,0)
	photo1num=0
	photo2num=0
	photoangle1=[]
	photoangle2=[]
	current_angle = float(0)
	scanangle=90
	scanresolution=1
	scanrange=1
	scannercalibration=[]
	scanstepscamera1=0
	scanstepscamera2=0
	scanstepangle1=0
	scanstepangle2=0
	
	def scansetup(self):
		file_object=open("CalibrationValues.txt","r")
		int=0
		while int <20:
			int=int+2
			scannercalibration.append(file_object.readline(int)) #Camera 1 H FOV 0, V FOV 1, Base 2, mount distance 3, calibration 4, camera 2 H FOV 5, v FOV 6, Base 7, mount distance 8, calibration 9 
		flie_object.close()
		if self._90_degree_scan.state == 'down':
			scanangle=90 
		elif self._180_degree_scan.state == 'down':
			scanangle=180
		elif self._270_degree_scan.state == 'down':
			scanangle=270
		elif self._360_degree_scan.state == 'down':
			scanangle=360
		if self.low_res_fast.state == 'down':
			scanresolution=1
		elif self.full_res_short.state == 'down':
			scanresolution=1
		elif self.full_res_standard.state == 'down':
			scanresolution=1
		elif self.full_res_multiple.state == 'down':
			scanresolution=3
		scanstepscamera1=math.ceil((scanangle*scanresolution)/(float(scannercalibration[1])))
		scanstepscamera2=math.ceil((scanangle*scanresolution)/(float(scannercalibration[6])))
		scanstepangle1=float(scanangle/scanstepscamera1)
		scanstepangle2=float(scanangle/scanstepscamera2)
				
	def __init__(self, screen_manager):

		self.sm = screen_manager

		# Establish 's'erial comms and initialise
		self.s = serial_connection.SerialConnection(self, self.sm)
		self.s.establish_connection()


	# JOG FUNCTIONS
	def jog_relative(self, angle):
			strrotate='c'+(angle*30)
			self.s.write_command(strrotate.encode('utf-8'))
			current_angle=currentangle+scanstepangle2 # update this to feed back information from stepper
			time.sleep(1)#when we get feedback from the stepper, we can remove this. just need to make sure it's not moving when taking images
			return(current_angle)

	def camera1open(self,low_res_fast_state):
		i2c='i2cset -y 1 0x70 0x00 0x04' #set camera 1 i2c
		GPIO.output(chan_listc,(1,0,0)) #select camera 1 GPIO
		camera=PiCamera()
		if low_res_fast_state=1:
			camera.resolution=(1640,1232)
		elif low_res_fast_state=0:
			camera.resolution=(3280,2464)
		camera.meter_mode='backlit'
		camera.start_preview(fullscreen=False,window=(200,80,600,400)) # check that this ends up with our screen in the right place!
		time.sleep()
		
	def camera2open(self,low_res_fast_state):
		i2c='i2cset -y 1 0x70 0x00 0x06' #set camera 1 i2c
		GPIO.output(chan_listc,(0,1,0)) #select camera 1 GPIO
		camera=PiCamera()
		if low_res_fast_state=1:
			camera.resolution=(1640,1232)
		elif low_res_fast_state=0:
			camera.resolution=(3280,2464)
		camera.meter_mode='backlit'
		camera.start_preview(fullscreen=False,window=(200,80,600,400)) # check that this ends up with our screen in the right place!
		time.sleep()
		
		
	def cameraclose(self):
		camera.stop_preview()
		
	def camera1take(self,photo1num):
		loffname='c1loff'+photo1num+'.jpg'
		lonname='c1on'+photo1num+'.jpg'
		expt=camera.exposure_speed
        #query exposure speed from camera
        if expt < 4000:
        	camera.shutter_speed=4000
        #sets shutter speed to maximum (minimum speed)
        else:
            camera.shutter_speed=0
        #doesn't override
		camera.capture(loffname,'jpeg',use_video_port=True)
		GPIO.output(chan_listl, (1,0,0)) # this needs to be the IR laser, double check when able
		camera.capture(lonname,'jpeg',use_video_port=True)
		camera.shutter_speed=0 # allows camera to adjust after taking both photos
		
	def camera2take(self,photo2num):
		loffname='c2loff'+photo2num+'.jpg'
		lonname='c2on'+photo2num+'.jpg'
		expt=camera.exposure_speed
        #query exposure speed from camera
        if expt < 4000:
        	camera.shutter_speed=4000
        #sets shutter speed to maximum (minimum speed)
        else:
            camera.shutter_speed=0
        #doesn't override
		camera.capture(loffname,'jpeg',use_video_port=True)
		GPIO.output(chan_listl, (1,0,0)) # this needs to be the IR laser, double check when able
		camera.capture(lonname,'jpeg',use_video_port=True)
		camera.shutter_speed=0 # allows camera to adjust after taking both photos	
		
	def scancamera1(self,photo1num,photoangle1,scanstepangle1,scanstepscamera1):
		self.camera1open(low_res_fast.state)
		while stop_scan != 1:
			for photo1num in range(scanstepscamera1):
				photoangle1.append(current_angle)
				self.camera1take(photo1num)
				self.jog_relative(scanstepangle1)
		self.cameraclose()
		return(current_angle)
		
	def scancamera2(self,photo2num,photoangle2,scanstepangle2,scanstepscamera2):
		self.camera2open(low_res_fast.state)
		while stop_scan != 1:
			for photo2num in range(scanstepscamera2):
				photoangle1.append(current_angle)
				self.camera1take(photo1num)
				self.jog_relative(scanstepangle2)
		self.cameraclose()
		return(current_angle)
		
	def angleadjust(self,current_angle):
		returntozero=float(360-current_angle)
		self.jog_relative(returntozero)
		time.sleep(10) #won't be needed when feedback is enabled
		return(current_angle)
		
	def readimages1(self,scanstepscamera1,photoangle1()):
		or photographs in range(scanstepscamera1):
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
            retval, threshold = threshold(red1, threshamount, 255, cv.THRESH_TOZERO);
            #this then removes those from the image
            #(minVal, maxVal, MinLoc, maxLoc) = minMaxLoc(threshold) - #not sure if this is needed any more, so commented it out.
            #find the maximum value of the non blurred image
            maxvalue1 = np.argmax(threshold,axis=1)
            #Now find the maximum value in each column. Originally, this was then counted as the centre point of the laser, but we are now going to use a weighted average.
            os.remove(loffname1)
            os.remove(lonname1)
            #Delete the image files, to save space
            succesful=0
            
		
		