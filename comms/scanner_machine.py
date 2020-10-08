
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
	scanstepangle1=float(0)
	scanstepangle2=float(0)
	scancameras=1
	for_calc_1=[]
	for_calc_2=[]
	output=[]
	
	def scan_setup(self):
		file_object=open("CalibrationValues.txt","r")
		int=0
		while int <24:
			int=int+2
			scannercalibration.append(file_object.readline(int)) #Camera 1 H FOV 0, V FOV 1, Base 2, Z mount distance 3, X Mount Distance 4, calibration 5, camera 2 H FOV 6, v FOV 67, Base 8, Z mount distance 9, X Mount Distance 10, calibration 11
		flie_object.close()
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

	def camera_1_open(self,low_res_fast_state):
		i2c='i2cset -y 1 0x70 0x00 0x04' #set camera 1 i2c
		GPIO.output(chan_listc,(1,0,0)) #select camera 1 GPIO
		camera=PiCamera()
		if low_res_fast_state=='down':
			camera.resolution=(1640,1232)
		elif low_res_fast_state=='up':
			camera.resolution=(3280,2464)
		camera.meter_mode='backlit'
		camera.start_preview(fullscreen=False,window=(200,80,600,400)) # check that this ends up with our screen in the right place!
		time.sleep()
		
	def camera2open(self,low_res_fast_state):
		i2c='i2cset -y 1 0x70 0x00 0x06' #set camera 1 i2c
		GPIO.output(chan_listc,(0,1,0)) #select camera 1 GPIO
		camera=PiCamera()
		if low_res_fast_state=='down':
			camera.resolution=(1640,1232)
		elif low_res_fast_state=='up':
			camera.resolution=(3280,2464)
		camera.meter_mode='backlit'
		camera.start_preview(fullscreen=False,window=(200,80,600,400)) # check that this ends up with our screen in the right place!
		time.sleep()
		
		
	def camera_close(self):
		camera.stop_preview()
		
	def camera_1_take(self,photonum):
		loffname='c1loff'+photonum+'.jpg'
		lonname='c1on'+photonum+'.jpg'
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
		
	def camera_2_take(self,photonum):
		loffname='c2loff'+photonum+'.jpg'
		lonname='c2on'+photonum+'.jpg'
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
		
	def scan_camera_1(self,photonum,photoangle,scanstepangle,scansteps):
		self.camera_1_open(low_res_fast.state)
		while self.stop_scan != 'down':
			for photonum in range(scansteps):
				photoangle1.append(current_angle)
				self.camera1take(photonum)
				self.jog_relative(scanstepangle)
		self.camera_close()
		return(current_angle)
		
	def scan_camera_2(self,photonum,photoangle,scanstepangle,scansteps):
		self.camera_2_open(low_res_fast.state)
		while self.stop_scan != 'down':
			for photonum in range(scansteps):
				photoangle2.append(current_angle)
				self.camera_1_take(photonum)
				self.jog_relative(scanstepangle)
		self.camera_close()
		return(current_angle)
		
	def angle_adjust(self,current_angle):
		returntozero=float(360-current_angle)
		self.jog_relative(returntozero)
		time.sleep(10) #won't be needed when feedback is enabled
		return(current_angle)
	
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
		
	def read_images_1(self,scansteps,photoangle()):
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
            		for_calc_1.append([i,laserctr,(photoangle[photographs]),0])
            		succesful1=succesful1+1
            	else: succesful1=succesful1
        print ("short range points captured %d" % (succesful1))
       
	def read_images_2(self,scansteps,photoangle()):
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
            		for_calc_2.append([i,laserctr,(photoangle[photographs]),0])
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
		if low_res_fast.state == 'down':
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
			if u = halfyres:
				x1=camxoffset # would be 0 where it falls in the centre of the CCD, then offset by the amount the camera is off centre
			if u > halfyres:
				u2=u-halfyres
				theta=math.atan(u2/a2const)
				x1=(w*math.tan(theta))+camxoffset # x would be a * value where it falls over halfway across CCD, then offset by amount camera is off centre
		hyp1=math.sqrt((math.pow(x1,2))+(math.pow(y1,2))) # Find the hypotenuse of the newly created triangle of points (where opposite = x, adjacent =y)
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
		laser=float(calibration[9])
		camxoffset=float(calibration[10])
		calib_value=float(calibration[11])
		if low_res_fast.state == 'down':
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
			w=math.sqrt((y1*y1)+(laser*laser)) # camera w vector 
			if u < halfyres:
				u2=halfyres-u
				theta=math.atan(u2/a2const)
				x1=(-(w*math.tan(theta)))+camxoffset
			if u = halfyres:
				x1=camxoffset
			if u > halfyres:
				u2=u-halfyres
				theta=math.atan(u2/a2const)
				x1=(w*math.tan(theta))+camxoffset				
		hyp1=math.sqrt((math.pow(x1,2))+(math.pow(y1,2))) # Find the hypotenuse of the newly created triangle of points (where opposite = x, adjacent =y)
		tan1=math.atan(x1/y1) #find theta angle for new triangle				
		rrad=r/(180/math.pi) # rotation of unit in radians
		xout=hyp1*math.sin(rrad+tan1) # x output adjusted for rotation around Z axis
		yout=hyp1*math.cos(rrad+tan1) # y output adjusted for rotation around Z axis
		output.append([xout,yout,0]) # X, Y, Z coordinates for output. Z is assumed to be 0 for easy import into CAD		
				