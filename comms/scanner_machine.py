
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
	
	def scansetup(self):
		file_object=open("CalibrationValues.txt","r")
		int=0
		while int <20:
			int=int+2
			scannercalibration.append(file_object.readline(int)) #Camera 1 H FOV 0, V FOV 1, Base 2, mount distance 3, calibration 4, camera 2 H FOV 5, v FOV 6, Base 7, mount distance 8, calibration 9 
		flie_object.close()
		
		

	def __init__(self, screen_manager):

		self.sm = screen_manager

		# Establish 's'erial comms and initialise
		self.s = serial_connection.SerialConnection(self, self.sm)
		self.s.establish_connection()


	# JOG FUNCTIONS
	def jog_relative(self, angle):
		pass
		# self.s.write_command(insert_stepper_command)
		# ADD IN COMMAND THAT TELLS MACHINE TO MOVE ANGLE
		
	def camera1open(self,low_res_fast.state):
		i2c='i2cset -y 1 0x70 0x00 0x04' #set camera 1 i2c
		GPIO.output(chan_listc,(1,0,0)) #select camera 1 GPIO
		camera=PiCamera()
		if low_res_fast.state=1:
			camera.resolution=(1640,1232)
		elif low_res_fast.state=0:
			camera.resolution=(3280,2464)
		camera.meter_mode='backlit'
		camera.start_preview(fullscreen=False,window=(200,80,600,400)) # check that this ends up with our screen in the right place!
		time.sleep()
		
	def camera2open(self,low_res_fast.state):
		i2c='i2cset -y 1 0x70 0x00 0x06' #set camera 1 i2c
		GPIO.output(chan_listc,(0,1,0)) #select camera 1 GPIO
		camera=PiCamera()
		if low_res_fast.state=1:
			camera.resolution=(1640,1232)
		elif low_res_fast.state=0:
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
		
	def scancamera1(self,photo1num,photoangle1,scanangle,scanresolution):
		camera1open()
		while stop_scan != 1:
			for photo1num in range()
		
		
		