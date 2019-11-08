#! /usr/bin/env python

#RUN AS: rosrun maki_robot time_test_maki_command.py

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
import os

import math
from time import sleep	## TODO: replace sleep() with rospy.sleep()
import sys
import string
import signal 

import thread
import threading
from array import *
import re		# see http://stackoverflow.com/questions/5749195/how-can-i-split-and-parse-a-string-in-python
import io

import random

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions

from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt

from test_saccade import saccadeTest
from test_startle import startleTest
from test_head_nod import headNodTest
from test_blink import blinkTest
from test_asleep_awake import asleepAwakeTest
from test_turn_to_screen import turnToScreenTest

## subset of servo control infix for type of feedback
FEEDBACK_SC = [ #SC_GET_MX,
		#SC_GET_MN,
		SC_GET_PP,
		SC_GET_GP	#,
		#SC_GET_PS,
		#SC_GET_GS,
		#SC_GET_PT,
		#SC_GET_PL,
		#SC_GET_ER,
		#SC_GET_DP,
		#SC_GET_DS,
		#SC_GET_TM,
		#SC_GET_TL,
		#SC_GET_TS
		 ]
FEEDBACK_TOPIC = [ #"maki_feedback_max_pos",
		#     "maki_feedback_min_pos",
			"maki_feedback_pres_pos",
			"maki_feedback_goal_pos"	#,
		#     "maki_feedback_pres_speed",
		#     "maki_feedback_goal_speed",
		#     "maki_feedback_pres_temp",
		#     "maki_feedback_pres_load",
		#     "maki_feedback_error",
		#     "maki_feedback_default_pos",
		#     "maki_feedback_default_speed",
	 	#     "maki_feedback_torque_max",
	 	#     "maki_feedback_torque_limit",
		#     "maki_feedback_torque_enable"
		]

## GLOBAL VARIABLES FOR PYTHON SCRIPT
VERBOSE_DEBUG = False	## default is False
#ALIVE #= False		## make sure that all threads cleanly end
#INIT #= True		## flag to read first message from MAKI (will contain PP)
#makiPP
#makiGP

#last_blink_time = 0
#last_movement_time = 0




def macroHeadTurn():
	_face_right = "HP" + SC_SET_GP + str(HP_RIGHT) + SC_SET_IPT + str(IPT_FACE) + TERM_CHAR_SEND
	_face_left = "HP" + SC_SET_GP + str(HP_LEFT) + SC_SET_IPT + str(IPT_FACE) + TERM_CHAR_SEND
	_face_front = "HP" + SC_SET_GP + str(HP_FRONT) + SC_SET_IPT + str(IPT_FACE) + TERM_CHAR_SEND

#####################
def timedTestUpdate( makiPP ):
	global eyeSaccade, startle, blink, headNod
	global asleepAwake, turnToScreen

	if eyeSaccade != None:	eyeSaccade.update( makiPP )
	if startle != None:	startle.update( makiPP )
	if headNod != None:	headNod.update( makiPP )
	if blink != None:	blink.update( makiPP )
	if asleepAwake != None:	asleepAwake.update( makiPP )
	if turnToScreen != None:	turnToScreen.update( makiPP )

def timedTestShutdown( ):
	global eyeSaccade, startle, blink, headNod
	global asleepAwake, turnToScreen

	_allTimedTests = ( eyeSaccade, startle, blink, headNod, asleepAwake, turnToScreen )

	for _test in _allTimedTests:
		if _test != None:
			#rospy.logdebug("_test = " + str(_test) )
			_test.stopTimedTest()
			sleep(1)
			_test.exitTimedTest()

	pubTo_maki_command( "HT" + str(SC_SET_TL) + str(ht_tl_disable) + str(TERM_CHAR_SEND) )
	#rospy.logdebug("HERE timedTestShutdown: " + str(_allTimedTests))

#####################
def parseRecvMsg ( recv_msg ):
	global makiPP, makiGP
	global init_dict
	global FEEDBACK_SC
	global VERBOSE_DEBUG
	global feedback_template
	global start_movement_time, finish_movement_time
	global expected_movement_duration

	if VERBOSE_DEBUG:
		#rospy.logdebug( "parseRecvMsg: BEGIN" )
		rospy.logdebug( "Received: " + str(recv_msg) )

	tmp = feedback_template.search(recv_msg)
	if tmp != None:
		prefix = tmp.group(1)
		feedback_values = tmp.group(2)
		#print "Validated: prefix='" + prefix + "' and feedback_values='" + feedback_values + "'"
	else:
		rospy.logerr( "Received with ERROR! Invalid message format: " + str(recv_msg) )
		return	## return without an expression argument returns None. Falling off the end of a function also returns None

	values = re.findall("([0-9]+)", feedback_values)	## this is a list of strings
	## need to conver to int (see http://stackoverflow.com/questions/22672598/converting-lists-of-digits-stored-as-strings-into-integers-python-2-7) 
	tmp_dict = dict( zip(F_VAL_SEQ, map(int, values)) )

	if prefix=="PP":	#SC_GET_PP	# present position
		if not ( tmp_dict == makiPP ):
			makiPP.update(tmp_dict)
			timedTestUpdate( makiPP )
			if VERBOSE_DEBUG: rospy.logdebug( prefix + "=" + str(makiPP) )
		if not init_dict[prefix]:
			init_dict[prefix] = True
		else:
			finish_movement_time = rospy.get_time()
			rospy.logdebug( "---- finish_movement_time = " + str(finish_movement_time) )

	elif prefix=="GP":	#SC_GET_GP	# goal position
		if not ( tmp_dict == makiGP ):
			makiGP.update(tmp_dict)
			if VERBOSE_DEBUG: rospy.logdebug( prefix + "=" + str(makiGP) )
		if not init_dict[prefix]:	init_dict[prefix] = True

	else:
		rospy.logerr( "parseRecvMsg: ERROR! Unknown prefix: " + str(prefix) )
		return

	#print "parseRecvMsg: END"


#####################
def getMacroCommand( msg ):
	global eyeSaccade, startle, blink, headNod
	global asleepAwake, turnToScreen
	#rospy.logdebug(rospy.get_caller_id() + ": I heard %s", msg.data)

	if (msg.data == "reset") or (msg.data == "end"):
		timedTestShutdown()

	elif (msg.data == "nod_test"):
		if headNod == None:
			headNod = headNodTest( VERBOSE_DEBUG, pub_cmd )
		## dynamically create separate thread only as needed
		try:
			thread.start_new_thread( headNod.macroHeadNod, () )
			headNod.startTimedTest( makiPP )
		except:
			rospy.logerr("Error: Unable to start thread for headNod.macroHeadNod")

	elif (msg.data == "saccade_test"):
		if eyeSaccade == None:
			eyeSaccade = saccadeTest( VERBOSE_DEBUG, pub_cmd )
		## dynamically create separate thread only as needed
		try:
			thread.start_new_thread( eyeSaccade.macroEyeSaccade, () )
			eyeSaccade.startTimedTest( makiPP )
		except:
			rospy.logerr("Error: Unable to start thread for eyeSaccade.macroEyeSaccade")

	elif (msg.data == "blink_test"):
		if blink == None:
			blink = blinkTest( VERBOSE_DEBUG, pub_cmd )
		## dynamically create separate thread only as needed
		try:
			thread.start_new_thread( blink.macroBlink, () )
			blink.startTimedTest( makiPP )
		except:
			rospy.logerr("Error: Unable to start thread for blink.macroBlink")

	elif (msg.data == "startle_test"):
		if startle == None:
			startle = startleTest( VERBOSE_DEBUG, pub_cmd )
		## dynamically create separate thread only as needed
		try:
			thread.start_new_thread( startle.macroStartle, () )
			startle.startTimedTest( makiPP )
		except:
			rospy.logerr("Error: Unable to start thread for startle.macroStartle")

	elif (msg.data == "asleep_test"):
		if asleepAwake == None:
			asleepAwake = asleepAwakeTest( VERBOSE_DEBUG, pub_cmd )
		## dynamically create separate thread only as needed
		try:
			thread.start_new_thread( asleepAwake.macroAsleep, () )
			asleepAwake.startTimedTest( makiPP )
		except:
			rospy.logerr("Error: Unable to start thread for asleepAwake.macroAsleep")

	elif (msg.data == "awake_test"):
		if asleepAwake == None:
			asleepAwake = asleepAwakeTest( VERBOSE_DEBUG, pub_cmd )
		## dynamically create separate thread only as needed
		try:
			thread.start_new_thread( asleepAwake.macroAwake, () )
			asleepAwake.startTimedTest( makiPP )
		except:
			rospy.logerr("Error: Unable to start thread for asleepAwake.macroAwake")

	elif (msg.data == "turn_screen_test"):
		if turnToScreen == None:
			turnToScreen = turnToScreenTest( VERBOSE_DEBUG, pub_cmd )
		## dynamically create separate thread only as needed
		try:
			thread.start_new_thread( turnToScreen.macroTurnToScreen, () )
			turnToScreen.startTimedTest( makiPP )
		except:
			rospy.logerr("Error: Unable to start thread for turnToScreen.macroTurnToScreen")

	else:
		rospy.logerr("Error: Unknown " + str(msg.data))

	return
	
def updateMAKICommand( msg ):
	global maki_command
	#rospy.logdebug(rospy.get_caller_id() + ": I heard %s", msg.data)

	## filter out feedback requests using regex
	_tmp = re.search("^F[A-Y]{2}Z$", msg.data)

	## YES, msg.data is a feedback request
	if _tmp != None:
		pass
	else:
		maki_command = msg.data 
		parseMAKICommand( maki_command )

	return

def parseMAKICommand( m_cmd ):
	global start_movement_time, finish_movement_time
	global expected_movement_duration
	global mc_count

	_emd_print = ""

	_tmp = re.search("^([A-Y]+GP[0-9]+)+(IPT([0-9]+))*Z$", m_cmd)
	if _tmp != None:
		## if the start_movement_timer already exists
		if start_movement_time != None:
			finish_movement_time = rospy.get_time()
			reportMovementDuration()

		start_movement_time = rospy.get_time()
		rospy.logdebug( "----- start_movement_time = " + str(start_movement_time) )
		#print _tmp.group(1)
		#print _tmp.group(2)
		#print _tmp.group(3)

		if _tmp.group(3) != None:
			## convert milliseconds to float seconds
			expected_movement_duration = float(_tmp.group(3)) / 1000.0
			_emd_print = "expected_movement_duration = " + str(expected_movement_duration) 
		else:
			expected_movement_duration = None
			_emd_print = "expected_movement_duration NOT SPECIFIED" 

		mc_count += 1
		rospy.logdebug( "maki_command #" + str(mc_count) + " : " + str(m_cmd) )
		rospy.logdebug( str(_emd_print) )


######################
#def sleepWhileWaitingMS( ms_sleep_time, increment=0.25, early=True):
#	## convert from IPT in milliseconds to sleep in seconds
#
#	_new_ms_sleep_time = float(ms_sleep_time)/1000.0 
#	if early:
#		_new_ms_sleep_time = _new_ms_sleep_time - float(increment)
#	
#	#print "ms_sleep_time = " + str( ms_sleep_time )
#	#print "increment = " + str( increment )
#	#print "new_ns_sleep_time = " + str( _new_ms_sleep_time )
#	sleepWhileWaiting( _new_ms_sleep_time, increment )
#
#def sleepWhileWaiting( sleep_time, increment=1 ):
#	global PIC_INTERRUPT
#	global ALIVE
#
#	if VERBOSE_DEBUG: rospy.logdebug( "BEGIN: sleepWhileWaiting for " + str(sleep_time) + " seconds" )
#	increment = max(0.01, increment)	## previous max values: 1.0, 0.25	## in seconds
#	_start_sleep_time = rospy.get_time()
#	sleep(increment)	# ktsui, INSPIRE 4, pilot 2; emulate a do-while
#	while ( ALIVE and
#		not rospy.is_shutdown() and
#		not PIC_INTERRUPT and
#		((rospy.get_time() - _start_sleep_time) < sleep_time) ):
#		sleep(increment)
#
#	if VERBOSE_DEBUG: rospy.logdebug( "DONE: sleepWhileWaiting for " + str(sleep_time) + " seconds" )

#####################
def pubTo_maki_command( commandOut ):
	global VERBOSE_DEBUG
	global pub_cmd
	global maki_msg_format
	_pub_flag = False

	## make sure that commandOut ends in only one TERM_CHAR_SEND
	_tmp = re.search( maki_msg_format, commandOut )
	if _tmp != None:
		## Yes, commandOut ends in only one TERM_CHAR_SEND
		_pub_flag = True
		#if VERBOSE_DEBUG:       rospy.logdebug( str(commandOut) + " matched maki_msg_format" )
	elif (commandOut == "reset"):
		## special case handled by MAKI-Arbotix-Interface.py driver
		_pub_flag = True
	elif not commandOut.endswith( str(TERM_CHAR_SEND) ):
		## append the missing TERM_CHAR_SEND
		commandOut += str(TERM_CHAR_SEND)
		_pub_flag = True
		if VERBOSE_DEBUG:       rospy.logdebug( str(commandOut) + " added TERM_CHAR_SEND" )
	else:
		rospy.logerr( "Incorrect message format" + str(commandOut) )

	if VERBOSE_DEBUG: rospy.logdebug( str(commandOut) )

	if _pub_flag and not rospy.is_shutdown():
		pub_cmd.publish( commandOut )

#####################
def signal_handler(signal, frame):
	global ALIVE

	timedTestShutdown( )
	sleep(1)
	ALIVE = False
	sleep(1)
	sys.exit()

#####################
def parseFeedback( msg ):
	#rospy.logdebug(rospy.get_caller_id() + ": I heard %s", msg.data)
	parseRecvMsg ( msg.data )
	return

#####################
def initROS():
	global VERBOSE_DEBUG

	## get function name for logging purposes
        _fname = sys._getframe().f_code.co_name        ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
	print str(_fname) + ": BEGIN"	## THIS IS BEFORE ROSNODE INIT

	# Initialize ROS node
        # see http://wiki.ros.org/rospy/Overview/Logging
        if VERBOSE_DEBUG:
                rospy.init_node('macro_time_test', log_level=rospy.DEBUG)
		rospy.logdebug("log_level=rospy.DEBUG")
        else:
                rospy.init_node('macro_time_test')       ## defaults to log_level=rospy.INFO
	rospy.logdebug( str(_fname) + ": END")
	return

def initPubMAKI():
	global pub_cmd
	global maki_msg_format

	## get function name for logging purposes
        _fname = sys._getframe().f_code.co_name        ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
	rospy.logdebug( str(_fname) + ": BEGIN")

	## make sure that commandOut ends in only one TERM_CHAR_SEND
	maki_msg_format = "\A[a-yA-Y]+[a-yA-Y0-9]*"
	maki_msg_format += str(TERM_CHAR_SEND)
	maki_msg_format += "{1}$"

	# Setup publisher
	pub_cmd = rospy.Publisher("maki_command", String, queue_size = 10)

	rospy.logdebug( str(_fname) + ": END")
	return

def initSubFeedback():
	global feedback_template

	## get function name for logging purposes
        _fname = sys._getframe().f_code.co_name        ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
	rospy.logdebug( str(_fname) + ": BEGIN")

	## STEP 0: Setup regex template for expected feedback syntax
	feedback_msg_format = "\A([A-Z]{2})"	## 2 alphabetic char prefix
	feedback_msg_format += "(([0-9]+" + DELIMITER_RECV + "){" + str(SERVOCOUNT-1) + "}[0-9]+)" 
	feedback_msg_format += TERM_CHAR_RECV + "\Z"
	#print feedback_msg_format
	feedback_template = re.compile(feedback_msg_format)

	# STEP 1: Setup subscribers
	for topic in FEEDBACK_TOPIC:
		rospy.Subscriber( topic, String, parseFeedback )
		rospy.logdebug( "now subscribed to " + str(topic) )

	rospy.Subscriber( "/maki_command", String, updateMAKICommand )
	rospy.logdebug( "now subscribed to /maki_command" )
	#rospy.Subscriber( "/maki_macro", String, getMacroCommand )
	#rospy.logdebug( "now subscribed to /maki_macro" )
	rospy.Subscriber( "/maki_test", String, getMacroCommand )
	rospy.logdebug( "now subscribed to /maki_test" )

	rospy.logdebug( str(_fname) + ": END")
	return


#####################
def reportMovementDuration():
	global start_movement_time, finish_movement_time
	global expected_movement_duration

	if ((start_movement_time != None) and
		(finish_movement_time != None) and
		(expected_movement_duration != None)):
		rospy.loginfo( "maki_command #" + str(mc_count) + " : elapsed time = "
			+ str( abs(finish_movement_time - start_movement_time) ) 
			+ "; expected time = " + str(expected_movement_duration) )
		## reset
		resetTimer()


def resetTimer():
	global start_movement_time, finish_movement_time
	global expected_movement_duration

	start_movement_time = None
	finish_movement_time = None
	expected_movement_duration = None

#####################
if __name__ == '__main__':

	global ALIVE
	global VERBOSE_DEBUG
	global INIT, init_dict
	global makiPP, makiGP
	global maki_command
	global pub_cmd
	global start_movement_time, finish_movement_time
	global expected_movement_duration
	global mc_count
	global PIC_INTERRUPT
	global DC_helper
	global SWW_WI
	global eyeSaccade, startle, blink, headNod
	global asleepAwake, turnToScreen

	## ---------------------------------
	## INITIALIZATION
	## ---------------------------------
	## STEP 0: INIT ROS
	initROS()

	## STEP 1: INIT GLOBAL VARIABLES
	ALIVE = False
	INIT = True
	init_dict = dict( zip(FEEDBACK_SC, [ False ] * len(FEEDBACK_SC) ) )
	makiPP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )
	makiGP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )
	maki_command = ""
	mc_count = 0
	resetTimer()
	PIC_INTERRUPT = False
	DC_helper = dynamixelConversions()
	SWW_WI = ROS_sleepWhileWaiting_withInterrupt( verbose_debug=VERBOSE_DEBUG )
	eyeSaccade = None
	startle = None
	headNod = None
	blink = None
	asleepAwake = None
	turnToScreen = None

	## STEP 2: SIGNAL HANDLER
	#to allow closing the program using ctrl+C
	signal.signal(signal.SIGINT, signal_handler)

	## STEP 3: INIT ROBOT STATE
	## establish communication with the robot via rostopics
	initPubMAKI()
	initSubFeedback()
	## wait here if nothing has been published to rostopics yet
	while INIT and not rospy.is_shutdown():
		my_init = INIT
		for _servo_command in FEEDBACK_SC:
			#print _servo_command
			#print my_init
			my_init = my_init and init_dict[ _servo_command ]
			#print my_init
			sleep(0.5)	#0.5s
			pubTo_maki_command( str(SC_FEEDBACK + _servo_command + TERM_CHAR_SEND) )

		if not my_init:
			sleep(1.0)
		else:
			INIT = False
			rospy.loginfo( "Init MAKI state -- DONE" )

	## ---------------------------------
	## END OF INITIALIZATION
	## ---------------------------------

	resetTimer()
	ALIVE = True
	while ALIVE and not rospy.is_shutdown():
		reportMovementDuration()
		pass	## nop to allow stdin to pass through

	#rospy.spin()	## keeps python from exiting until this node is stopped

	print "__main__: END"



