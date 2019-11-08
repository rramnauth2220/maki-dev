#! /usr/bin/env python

#RUN AS:	rosrun maki_robot MAKI-Arbotix-Interface.py <PORT, default=USB0>

import rospy
import re
from std_msgs.msg import String
import serial
from serial.tools import list_ports

#from time import sleep
import signal
import sys

#from timeit import default_timer as timer

import random

from maki_robot_common import *
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt

# --------------------------------------------------------------------	
## ---- USER DEFINED GLOBALS ----

global SIM
# global VERBOSE_DEBUG
global ALIVE
global TTY_PORT
global BAUD_RATE
global maki_serial
global maki_port

DIAGNOSE_SERIAL_OVERFLOW = False	## default is False
SIM = False				## default is False
VERBOSE_DEBUG = False	## default is False, corresponding to log_level=rospy.INFO. True corresponds to log_level=rospy.DEBUG
LATCH = False			## if LATCH==True, any new subscribers will see the most recent message published
TTY_PORT = "USB0"		## default port for the MAKI Arbotix-M board

## ---- CONSTANTS ----
BAUD_RATE = 115200		## 9600
EC_TIMER_DURATION = 30	## 30s, or 30000ms

## ---- DYNAMIC GLOBALS ---- modified programmatically ----
ALIVE = False
maki_serial = None 				## init as Python's null object
feedback_req_template = ""		## init as empty string; dynamically populated as compiled regular expression
feedback_resp_template = ""		## init as empty string; dynamically populated as compiled regular expression
feedback_pub_dict = { }			## init as empty dictionary; dynamically populated
feedback_topic_name_dict = { }	## init as empty dictionary; dynamically populated
resetPositions = ""				## init as empty string; dynamically populated
resetSpeeds = ""				## init as empty string; dynamically populated
SIM_feedback_type = None

exp_pub = rospy.Publisher("experiment_info", String, queue_size = 10)

# --------------------------------------------------------------------
def usage(cmd_line_args):
	rospy.loginfo( "Usage: rosrun maki_robot MAKI-Arbotix-Interface.py <PORT, default=USB0>" )
	if cmd_line_args != None and cmd_line_args != "":
		rospy.loginfo( "Given command line args: " + str(cmd_line_args) )
		sys.exit()

## ------------------------------
def recvFromArduino():
	global SIM
	global ALIVE
	global maki_serial
	global TERM_CHAR_RECV

	## TOO VERBOSE
	#rospy.logdebug( "BEGIN" )

	if SIM:
		## shouldn't get here since publishFeedback will generate a SIM feedback message
		rospy.logwarn( "SIM = " + str(SIM) + "; nothing to receive" )
		return ''

	_recv_msg = ''
	_RECEIVING = True
	_exit_flag = False
	while _RECEIVING and ALIVE:
		try:
			if maki_serial.isOpen() and maki_serial.inWaiting() > 0:
				_m_char = maki_serial.read(1)	# read 1 byte from Arduino
				_recv_msg += _m_char
				if _m_char==TERM_CHAR_RECV:
					_RECEIVING = False
## KATE
			elif DIAGNOSE_SERIAL_OVERFLOW:
				return ''
			else:
				pass
		except ValueError as e1:
			rospy.logerr( "VALUE ERROR: Serial connection closed while reading: " + str(e1) )
			_exit_flag = True
		except IOError as e2:
			rospy.logerr( "IOError: Serial connection unplugged while waiting for transmission from the robot: " + str(e2) )
			_exit_flag = True

		if _exit_flag:
			rospy.logerr( "Reading from serial port disturbed..." )
			print "####################################################\n"
			print "(Is the robot's Arbotix-M board plugged in?)"
			print "\n####################################################"
			sys.exit()	## use this instead of exit (which is meant for interactive shells)

	if _recv_msg != '' and maki_serial.isOpen():
		maki_serial.flushInput();	# clear the input buffer

	## TOO VERBOSE
	#rospy.logdebug( "END" )
	return _recv_msg

def sendToMAKI (message): 
	global SIM
	global maki_serial
	global feedback_strings
	global maki_cmd_template
	_bytes_written = 0

	## TOO VERBOSE
	#rospy.logdebug( "message received:\t" + str(message) )

	try:
		if not SIM and maki_serial.isOpen:
			maki_serial.flushOutput()
	except serial.serialutil.portNotOpenError as _e1:
		rospy.logerr( "portNotOpenError: Unable to flush serial output: " + str(_e1) )
	except ValueError as _e1_1:
		rospy.logerr( "ValueError: Unable to flush serial output: " + str(_e1_1) )
	
	#handle feedback commands
	#feedback_strings = {'FMXZ', 'FMNZ', 'FPPZ', 'FPSZ', 'FPTZ', 'FPLZ', 'FERZ', 'FDPZ', 'FDSZ'}
	if message.data in feedback_strings:
		feedback(message.data);
		return

	#reset positions (need to query them on start)
	if (message.data == "reset"):
		if not SIM:
			rospy.logdebug( "resetting speeds: " + str(resetSpeeds) )
			try:
				if maki_serial.isOpen:
					_bytes_written = maki_serial.write(resetSpeeds)
					rospy.loginfo( "speeds reset DONE: " + str(resetSpeeds) + "... " + str(_bytes_written) + " bytes written" )
					rospy.sleep(0.1)	## 100ms to propogate to servos

					rospy.logdebug( "resetting positions: " + str(resetPositions) )
					_bytes_written = maki_serial.write(resetPositions)
					rospy.loginfo( "positions reset DONE: " + str(resetPositions) + "... " + str(_bytes_written) + " bytes written" )
					rospy.sleep(0.1)	## 100ms to propogate to servos
			except serial.serialutil.portNotOpenError as _e2:
				rospy.logerr( "portNotOpenError: Unable to write serial output: " + str(_e2) )
			except ValueError as _e2_1:
				rospy.logerr( "ValueError: Unable to write serial output: " + str(_e2_1) )
		
	#check for valid command formatting (regex)
	else:
		#regex2 = re.compile('(((((HP)|(HT)|(LL)|(LR)|(EP)|(ET))((GP)|(GS)))|(IPT))\d{3,5})+Z')
		#match = regex2.match(message.data)

		_bytes_written = 0
		match = maki_cmd_template.match( message.data )
		if (match):
			if SIM:
				rospy.logwarn( "SIM = " + str(SIM) + "; nowhere to send message " + str(message) )
			else:
				try:
					if maki_serial.isOpen:
						rospy.logdebug( "sending command to Arbotix-M over serial: " + str(message.data) )
						_bytes_written = maki_serial.write(message.data)
						rospy.loginfo( str(message.data) + " : command sent to Arbotix-M over serial... _bytes_written=" + str(_bytes_written) )
				except serial.serialutil.portNotOpenError as _e3:
					rospy.logerr( "portNotOpenError: Unable to write serial output: " + str(_e3) )
				except ValueError as _e3_1:
					rospy.logerr( "ValueError: Unable to write serial output: " + str(_e3_1) )
		else:
			rospy.logerr( "invalid format: " + str(message.data) )
			return
	#if position command, ask for present position
	#if 'GP' in message.data:
		#feedback("FPPZ")

## ------------------------------
##	ros publishers are stored as the contents of a dictionary
##	keys come from the FEEDBACK_SC list; servo control infix for type of feedback
## ------------------------------
def initPubFeedback():
	## get function name for logging purposes
	#_fname = sys._getframe().f_code.co_name	## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-traceback
	#_fname = initPubFeedback.__name__	## same as above but less modular

	global feedback_pub_dict, feedback_topic_name_dict
	global feedback_strings
	global FEEDBACK_SC, FEEDBACK_TOPIC
	global LATCH

	rospy.logdebug( "setup rostopic publishers to give feedback" )
			
	feedback_topic_name_dict = dict( zip(FEEDBACK_SC, FEEDBACK_TOPIC) )
	#print feedback_topic_name_dict	## debugging
	feedback_pub_dict = { }		# init as empty dictionary
	feedback_strings = [ ]		# init as empty list
	for _sc_dict_key, _feedbackTopic in feedback_topic_name_dict.iteritems():
		_pub = rospy.Publisher(_feedbackTopic, String, queue_size = 26, latch = LATCH)	## if LATCH==True, any new subscribers will see the most recent message published
		feedback_pub_dict[_sc_dict_key] = _pub
	
		# dynamically populate feedback strings based on FEEDBACK_SC; more scalable
		# {'FMXZ', 'FMNZ', 'FPPZ', 'FPSZ', 'FPTZ', 'FPLZ', 'FERZ', 'FDPZ', 'FDSZ'}
		feedback_strings.append( str(SC_FEEDBACK + _sc_dict_key + TERM_CHAR_SEND) )
	#print feedback_strings
	return

def initRegexFormat():
	global TERM_CHAR_SEND, TERM_CHAR_RECV, DELIMITER_RECV
	global feedback_req_template, feedback_resp_template
	global maki_cmd_template

	_feedback_request_format = "\A" + str(SC_FEEDBACK)	## F is the FEEDBACK request prefix
	_feedback_request_format += "([A-Z]{2})"	
	_feedback_request_format += str(TERM_CHAR_SEND) + "\Z"	## ends in Z (capital zed)
	feedback_req_template = re.compile(_feedback_request_format)

	_feedback_response_format = "\A([A-Z]{2})"	## 2 alphabetic char prefix
	_feedback_response_format += "(([0-9]+" + str(DELIMITER_RECV) + "){" + str(SERVOCOUNT-1) + "}[0-9]+)"
	_feedback_response_format += str(TERM_CHAR_RECV) + "\Z"	## ends in ;
	feedback_resp_template = re.compile(_feedback_response_format)

	## this replaces regex2
	_motor_prefix = '((HP)|(HT)|(LL)|(LR)|(EP)|(ET))'
	_goal_cmd = '(((GP)|(GS))[0-9]+)'
	_goal_cmd_ipt = '((IPT)[0-9]+)'
	_torque_cmd = '(((TM)|(TL)|(TS))[0-9]+)'
	_maki_cmd_format_p1 = '((' + _motor_prefix 
	_maki_cmd_format_p1 += _goal_cmd  + ')+'
	_maki_cmd_format_p1 += _goal_cmd_ipt + '?)'
	_maki_cmd_format_p2 = '(' + _motor_prefix 
	_maki_cmd_format_p2 += _torque_cmd + ')+'
	_maki_cmd = '(' + _maki_cmd_format_p1 + ')|(' + _maki_cmd_format_p2 + ')'
	maki_cmd_template = re.compile( _maki_cmd )
	rospy.logdebug( _maki_cmd )

	return

def feedback(feedbackString):
	_feedback_type = requestFeedback(feedbackString)
	## ktsui : Intentionally commented out; only want the main loop to read from the serial port
	#if _feedback_type != '':
	#	publishFeedback(_feedback_type)
	return

def requestFeedback(feedbackString):
	global feedback_req_template
	global SIM, SIM_feedback_type
	global maki_serial

	rospy.loginfo( "About to request feedback; feedbackString=" + str(feedbackString) )

	_bytes_written = 0
	_tmp = feedback_req_template.search(feedbackString)
	## Yes, feedbackString has the expected format
	if _tmp != None:
		_feedback_type = _tmp.group(1)
		if not SIM:
			try:
				if maki_serial.isOpen:
					_bytes_written = maki_serial.write(feedbackString)
			except serial.serialutil.portNotOpenError as _e4:
				rospy.logerr("portNotOpenError: Unable to write serial output: " + str(_e4) )
			except ValueError as _e4_1:
				rospy.logerr("ValueError: Unable to write serial output: " + str(_e4_1) )

			rospy.logdebug("_bytes_written=" + str(_bytes_written))
		else:
			rospy.logwarn( "SIM = " + str(SIM) + "; nowhere to request feedback " + str(feedbackString) )
			SIM_feedback_type = _feedback_type
		return _feedback_type
	else:
		rospy.logerr( "INVALID SYNTAX for feedback request: " + str(feedbackString) )
		return ''
	return

def publishFeedback():
	global feedback_resp_template
	global feedback_pub_dict
	global SIM, SIM_feedback_type

	_recv_msg = ''		## Init to empty string
	if not SIM:
		_recv_msg = recvFromArduino()
	else:
		## otherwise, generate placeholder messages
		if (SIM_feedback_type != None) and (SIM_feedback_type != ''):
			_recv_msg = generateSIMFeedback(SIM_feedback_type)
		else:
			return

## KATE
	if (DIAGNOSE_SERIAL_OVERFLOW and
		(_recv_msg == None) or (_recv_msg == '')):
		## TOO VERBOSE
		#rospy.logdebug("nothing read from serial")
		return

	_tmp = feedback_resp_template.search(_recv_msg)
	if _tmp != None:
		_prefix = _tmp.group(1)
		_feedback_values = _tmp.group(2)
		rospy.logdebug( "Validated: prefix='" + str(_prefix) + "' and feedback_values='" + str(_feedback_values) + "'" )
		if feedback_pub_dict.has_key(_prefix):
			feedback_pub_dict[_prefix].publish(_recv_msg)
			rospy.loginfo( "Published '" + str(_recv_msg) + "' on rostopic " + str(feedback_topic_name_dict[_prefix]) ) 
		
			## reset after publishing to appropriate rostopic
			if SIM:	SIM_feedback_type = None
	else:
		rospy.logerr( "INVALID MESSAGE RECEIVED: '" + str(_recv_msg) + "'" )

	return

def generateSIMFeedback(feedback_type):
	global F_VAL_SEQ

	## otherwise, generate placeholder messages
	_gen_msg = str(feedback_type)
	_i = 0
	while (_i < len(F_VAL_SEQ) ):
		_i += 1
		if (feedback_type == "PT"):
			_gen_msg += str( random.randint(0,65) )
		elif (feedback_type == "TS"):
			_gen_msg += str( random.randint(0,1) )
		else:
			_gen_msg += str( random.randint(0,1023) )
		if (_i == len(F_VAL_SEQ)):
			_gen_msg += str(TERM_CHAR_RECV) 	## semicolon is syntax for end of FEEDBACK
			break
		else:
			_gen_msg += str(DELIMITER_RECV) 	## colon is syntax for FEEDBACK delimiter
	print _gen_msg
	return _gen_msg

## ------------------------------
def defReset():
	global SIM
	global maki_serial

	if SIM:
		rospy.logwarn( "SIM = " + str(SIM) + "; can't request or receive Default Position and Default Speed" )
		return

	rospy.logdebug( "defining reset strings" )
	maki_serial.write("FDPZ")
	resetCommand = ""
	c = maki_serial.read()
	while (c != 'D'): # waits for feedback
		c = maki_serial.read()
	while (c != ':'): #eliminates DP and first value
		c = maki_serial.read()
	resetCommand += token("LLGP")
	resetCommand += token("EPGP")
	resetCommand += token("ETGP")
	resetCommand += token("HTGP")
	resetCommand += token("HPGP")
	resetCommand += 'Z'
	
	global resetPositions
	resetPositions = resetCommand
	
	maki_serial.write("FDSZ")
	resetCommand = ""
	c = maki_serial.read()
	while (c != 'D'): # waits for feedback
		c = maki_serial.read()
	while (c != ':'): #eliminates DS and first value
		c = maki_serial.read()
	resetCommand += token("LLGS")
	resetCommand += token("EPGS")
	resetCommand += token("ETGS")
	resetCommand += token("HTGS")
	resetCommand += token("HPGS")
	resetCommand += 'Z'
	
	global resetSpeeds
	resetSpeeds = resetCommand
		
	rospy.logdebug( "resetPositions: " + str(resetPositions) )
	rospy.logdebug( "resetSpeeds: " + str(resetSpeeds) )

## helper function for defReset()
def token(header):
	subCommand = header
	c = maki_serial.read()
	while (c != ':' and c != ';'):
		subCommand += c
		c = maki_serial.read()
	return subCommand		

## ------------------------------
def signal_handler(signal, frame):
	makiExit()
	rospy.loginfo( "CTRL+C says goodnight" )
	sys.exit()	## use this instead of exit (which is meant for interactive shells)

def makiExit():
	global ALIVE
	global maki_serial

	if ALIVE:
		try:
			if maki_serial != None and maki_serial.isOpen():
				rospy.loginfo( "Closing the Arduino port..." )
				maki_serial.close()
		except serial.serialutil.portNotOpenError as _e5:
			rospy.logerr( str(_e5) )
		except ValueError:
			pass
		except AttributeError:
			pass
		ALIVE = False
		rospy.sleep(1)	# give a chance for everything else to shutdown nicely
		rospy.logdebug( "And MAKI lived happily ever after..." )
	exit	## meant for interactive interpreter shell; unlikely this actually exits

def openMAKISerialPort( baud=BAUD_RATE, timeout=None):
	global maki_serial, maki_port

	if maki_serial == None:
		rospy.logdebug("Opening new serial connection to " + str(maki_port))
		try:
			maki_serial = serial.Serial()
			maki_serial.baudrate = baud
			maki_serial.timeout = timeout
			maki_serial.port = maki_port
			maki_serial.open()
			rospy.logdebug( str(maki_serial) )
		except serial.serialutil.SerialException as _e0:
			rospy.logerr( str(_e0) )
			return False
	else:
		rospy.loginfo("Attempting to reopen existing serial connection...")
		try:
			maki_serial.open()
			rospy.logdebug( str(maki_serial) )
			rospy.loginfo("SUCCESS: Reopened serial connection to " + str(maki_port))	
		except serial.serialutil.SerialException as _e1:
			rospy.logerr("FAILED: Unable to reopen serial connection to " + str(maki_port))	
			rospy.logerr( str(_e1) )
			return False

	#try:
	#	maki_serial = serial.Serial( maki_port, int(baud), timeout=timeout) # no timeout  timeout=None
	#	rospy.loginfo( str(maki_serial) )
	#except serial.serialutil.SerialException as e0:
	#	rospy.logerr( str(e0) )
	#	return False
	
	if maki_serial != None and maki_serial.isOpen():
		maki_serial.flushInput();	# clear the input buffer
		maki_serial.flushOutput();	# clear the output buffer
		#rospy.loginfo( "SUCCESS: Opened serial connection to MAKI on " + str(maki_port) ) 
		rospy.logdebug("Flushed input/output buffers")

		maki_serial.write("FPPZ")
		rospy.logdebug("Serial connection READY: flushed input/output buffers")

	return maki_serial.isOpen()	## should only return True

## ------------------------------
if __name__ == '__main__':
	global maki_serial
	global maki_port
	global exp_pub

	## ------------------------------
	## BEGIN INITIALIZATION
	## ------------------------------
	## STEP 0: INIT GLOBAL VARIABLES
	global ALIVE
	ALIVE = True

	## STEP 1: SIGNAL HANDLER
	# allow closing the program using CTRL+C
	signal.signal(signal.SIGINT, signal_handler)

	## STEP 2: ROS SETUP
	# Initialize ROS node
	# see http://wiki.ros.org/rospy/Overview/Logging
	if VERBOSE_DEBUG:
		rospy.init_node('maki_arbotix_interface', log_level=rospy.DEBUG)
	else:
		rospy.init_node('maki_arbotix_interface')	## defaults to log_level=rospy.INFO
	#rospy.init_node('maki_arbotix_interface')	## defaults to log_level=rospy.INFO
	# Register shutdown hook
	rospy.on_shutdown(makiExit)
	# Setup regular expression templates for parsing feedback messages and ROS messages from maki_command rostopic
	initRegexFormat()
	# Subscribe to the maki_command stream
	rospy.Subscriber("maki_command", String, sendToMAKI)
	# Publisher setup
	initPubFeedback()

	## STEP 3: ESTABLISH SERIAL COMMUNICATION WITH THE ROBOT
	## STEP 3A: INSTANTIATE THE CONNECTION
	#print "SYS: " + str( len(sys.argv) ) + ", " + str(sys.argv)	## debug
	#print "ROS: " + str( len(rospy.myargv()) ) + ", " + str(rospy.myargv())	## debug
	## NOTE: rospy.myargv() strips __name:=FOO __log:=BAR command line args run from roslaunch file
	_argc = len(rospy.myargv())
	if ( _argc > 1 ):
		global TTY_PORT
		TTY_PORT = str(rospy.myargv()[1])
	if ( _argc > 2 ):
		usage(rospy.myargv()[1:])	## call sys.exit()

	maki_port = "/dev/tty" + str(TTY_PORT) # default port for the MAKI Arbotix Board
	#try:
	#	maki_serial = serial.Serial(_maki_port, int(BAUD_RATE), timeout=None) # no timeout  timeout=None
	#	rospy.loginfo( str(maki_serial) )
	#except serial.serialutil.SerialException as e0:
	#	rospy.logerr( str(e0) )
## KATE
	if DIAGNOSE_SERIAL_OVERFLOW:
		openMAKISerialPort( timeout=1 )
	else:
		openMAKISerialPort()

	## STEP 3B: ENSURE SERIAL COMMUNICATION WITH THE ROBOT
	if maki_serial != None and maki_serial.isOpen():
		maki_serial.flushInput();	# clear the input buffer
		maki_serial.flushOutput();	# clear the output buffer
		rospy.loginfo( "SUCCESS (1/2): Opened serial connection to MAKI on " + str(maki_port) + "\t...Waiting on initial transmission from Arbotix-M board" ) 
	else:
		if not SIM:
			rospy.logerr( "Unable to connect to MAKI on " + str(maki_port) + ". Exiting..." )
			sys.exit()	## use this instead of exit (which is meant for interactive shells)
		else:
			rospy.logwarn( "SIM = " + str(SIM) + "; continuing..." )

	## wait until Arbotix-M board transmits before continuing 
	## THIS WHILE LOOP IS BLOCKING
	_exit_flag = False
	_auto_feedback_ER_timer_start = rospy.get_time()
	#print "Start time: " + str(_auto_feedback_ER_timer_start)	## debugging
	requestFeedback( "FPPZ" )	## 2016-06-09: MAKI should NOT recenter on startup; check pres position
	_i = 0
	_n = 0
	if not SIM:	_n = maki_serial.inWaiting()
	while (_n <= 0) and (not SIM) and (not rospy.is_shutdown()):
		rospy.logdebug( str(_i) + ") maki_serial.inWaiting() = " + str(_n) )

		#print "Elapsed time: " + str( int(rospy.get_time() - _auto_feedback_ER_timer_start) )	## debugging
		if ( int(rospy.get_time() - _auto_feedback_ER_timer_start) > int(EC_TIMER_DURATION) ):
			rospy.logwarn( "Nothing received from serial port..." )
			print "####################################################\n"
			print "(Does the robot have power? Is the power switch on?)"
			print "\n####################################################"
			## reset the warning timer
			_auto_feedback_ER_timer_start = rospy.get_time()
			
		if _exit_flag:
			if ( int(rospy.get_time() - _auto_feedback_ER_timer_start) > 10 ):
				rospy.logerr( "Nothing received from serial port. Exiting..." )
				print "####################################################\n"
				print "(Does the robot have power? Is the power switch on?)"
				print "\n####################################################"
			sys.exit()	## use this instead of exit (which is meant for interactive shells)
		else:
			rospy.sleep(1)	# 1s
			_i += 1

		try:
			if maki_serial.isOpen():
				_n = maki_serial.inWaiting()
		except ValueError as e1:
			rospy.logerr( "VALUE ERROR: Serial connection closed while establishing communication: " + str(e1) )
			_exit_flag = True
		except IOError as e2:
			rospy.logerr( "IOError: Serial connection unplugged while waiting for transmission from the robot: " + str(e2) )
			_exit_flag = True

		if not ALIVE:
			_exit_flag = True

		
	# clear the input buffer; we don't actually care about the contents
	if not SIM: maki_serial.flushInput();

	## STEP 4: INIT ROBOT STATE
	# Reset MAKI to default position and speed
	defReset()
## KATE
	requestFeedback( "FPPZ" )	## 2016-06-09: MAKI should NOT recenter on startup; check pres position
	rospy.loginfo( "SUCCESS (2/2): Robot successfully connected.")
	exp_pub.publish('Robot is ready.')

## KATE
	## 2016-06-09: Disable HT motor on startup
	##	safer than waiting for this to be done by a user
	try:
		if maki_serial.isOpen:
			maki_serial.write( "HTTL0Z" )
			rospy.loginfo( "Disable head tilt motor: DONE" )
	except serial.serialutil.portNotOpenError as _e_ht_tl:
		rospy.logerr( "portNotOpenError: Unable to write serial output: " + str(_e_ht_tl) )
	except ValueError as _e_ht_tl_1:
		rospy.logerr( "ValueError: Unable to write serial output: " + str(_e_ht_tl_1) )
	requestFeedback( "FTLZ" )

	## ------------------------------
	## END OF INITIALIZATION
	## ------------------------------
	
	## main loop will process messages received from the Arbotix-M board
	# And now... go!
	#rospy.spin()	## sleeps until rospy.is_shutdown() == True; prevent main thread from exiting
	while ALIVE and not rospy.is_shutdown():
		publishFeedback()	## calls recvFromArduino() and generateSIMFeedback() if SIM=True
		#rospy.sleep(0.5)	# 500ms		## too much latency between feedback requests and feedback responses
		rospy.sleep(0.05)	# 50ms		## without rospy.get_time(), can't tell loop speed difference between 100ms and 50ms

	print " __main__: Bye bye"	## rosnode shutdown, can't use rospy.log* when rosnode is down

