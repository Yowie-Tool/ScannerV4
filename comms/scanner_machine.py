
from kivy.clock import Clock
import sys, os
from datetime import datetime
import os.path
from os import path

from comms import serial_connection

def log(message):
	timestamp = datetime.now()
	print (timestamp.strftime('%H:%M:%S.%f' )[:12] + ' ' + str(message))


class ScannerMachine(object):

	xdist = []
	ydist = []
	current_angle = float(0)

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