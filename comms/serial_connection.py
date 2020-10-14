import serial, sys, time, string, threading
from datetime import datetime
from os import listdir
from kivy.clock import Clock

import re
from functools import partial
from serial.serialutil import SerialException

def log(message):
	timestamp = datetime.now()
	print (timestamp.strftime('%H:%M:%S.%f' )[:12] + ' ' + str(message))


# USE THIS OBJECT TO ESTABLISH SERIAL CONNECTION AND HANDLE COMMUNICATING WITH SERIAL 

class SerialConnection(object):

	STATUS_INTERVAL = 0.1 # How often to poll general status to update UI (0.04 = 25Hz = smooth animation)

	s = None    # Serial comms object
	sm = None   # Screen manager object

	FLUSH_FLAG = False
	
	write_command_buffer = []
	write_realtime_buffer = []

	def __init__(self, machine, screen_manager):

		self.sm = screen_manager
		self.m = machine

	def is_connected(self):

		if self.s != None:
			return True
		else: 
			return False

	def establish_connection(self):

		log('Start to establish connection...')

		try: 
			self.s=serial.Serial('/dev/ttyAMA0',9600, timeout = 6, writeTimeout = 20)

			#Maybe we should add in some extra lines here to clear it as we found it didn't communicate immediately ?

		except: 
			log("Could not establish serial connection")

		log("Serial connection status: " + str(self.is_connected()))

	def start_services(self, dt):

		log('Starting services')
		self.s.flushInput()  # Flush startup text in serial input
		self.s.write(('\n\n').encode('utf-8'))
		self.s.write(('e1\n').encode('utf-8')) #Enables the stepper motor driver, turns out the program light.
		self.s.write(('z\n').encode('utf-8')) #Zeroes the encoder in the stepper
		self.next_poll_time = time.time()
		t = threading.Thread(target=self.serial_scan_loop)
		t.daemon = True
		t.start()

	VERBOSE_ALL_RESPONSE = False

	def serial_scan_loop(self):

		log('Running serial scanner thread')

		while True:
			if self.FLUSH_FLAG == True:
				self.s.flushInput()
				self.FLUSH_FLAG = False

			# PROCESS COMMANDS GOING TO SERIAL:
			command_counter = 0
			for command in self.write_command_buffer:
				self.write_direct(command)
				command_counter += 1
				
			del self.write_command_buffer[0:(command_counter)]

			# If there's a message received, deal with it depending on type:
			if self.s.inWaiting():
				# Read line in from serial buffer (rec_temp: received, temporary)
				try:
					rec_temp = self.s.readline().strip() #Block the executing thread indefinitely until a line arrives
				except Exception as e:
					log('serial.readline exception:\n' + str(e))
					rec_temp = ''
			else:
				rec_temp = ''

			# If something received from serial buffer, process it. 
			if len(rec_temp):  
				self.process_serial_output(rec_temp)

				if self.m.doing_scan_camera_1: 
					self.m.do_scan_step_camera1()

				if self.m.doing_scan_camera_2:
					self.m.do_scan_step_camera1()

	# SERIAL IS ONLY READING ANGLE: 
	def process_serial_output(self, output):
		self.m.current_angle_readout = output.decode("utf-8")
		log('angle moved is: ' + self.m.current_angle_readout)
		self.m.update_angle_moved()

	def write_command(self, serialCommand):
		self.write_command_buffer.append(str(serialCommand))

	def write_direct(self, serialCommand):

		if self.s:
			try:
				self.s.write((serialCommand).encode('utf-8')) # assume everything needs encoding??
				log('write ' + serialCommand + ' to serial')
			except:
				print("FAILED to write to SERIAL: " + serialCommand + " (Alt text: " + str(altDisplayText) + ")")

		else: 
			log("No serial! Command lost!: " + serialCommand + " (Alt text: " + str(altDisplayText) + ")")

