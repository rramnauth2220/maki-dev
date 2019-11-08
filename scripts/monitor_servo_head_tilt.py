#! /usr/bin/env python

#RUN AS: rosrun maki_robot monitor_servo_head_tilt.py

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty
import os

import serial
import math
from time import sleep
import sys
import string
import signal 

from timeit import default_timer as timer	## wall clock. Unix 1/100 second granularity

import threading
import re		# see http://stackoverflow.com/questions/5749195/how-can-i-split-and-parse-a-string-in-python
import io
from datetime import datetime, date, time


## NOTE: These globals are #define at the top of the Arduino servo driver
## MAKIv1_4_servo_controller_LITE.ino
INVALID_INT = 9999
DELIMITER_RECV = ':'	## syntax for FEEDBACK delimiter
TERM_CHAR_RECV = ';'	## syntax for end of FEEDBACK
TERM_CHAR_SEND = 'Z'	## syntax for end of servo command
SC_SET_GP = "GP"	## servo command syntax for setting a specified servo's GOAL POSITION
SC_SET_GS = "GS"	## servo command syntax for setting a specified servo's GOAL SPEED
SC_SET_IPT = "IPT"	## servo command syntax for setting the INTERPOLATION POSE TIME
SC_SET_TM = "TM"  	## servo command syntax for setting a specified servo's MAX TORQUE. Note: This value exists in EEPROM and persists when power is removed; default 1023
SC_SET_TL = "TL"  	## servo command syntax for setting a specified servo's TORQUE LIMIT. Note: If set to 0 by ALARM SHUTDOWN, servo won't move until set to another value[1,1023]; default MAX TORQUE
SC_SET_TS = "TS"  	## servo command syntax for setting a specified servo's TORQUE ENABLE. Note: Boolean [0, 1]; default 1. Doesn't appear to disable servo's movement when set to 0

SC_FEEDBACK = "F"	## servo command syntax for FEEDBACK or status of all servos
SC_GET_MX = "MX"	## servo command syntax for FEEDBACK of all servo MAXIMUM POSITION
SC_GET_MN = "MN"	## servo command syntax for FEEDBACK of all servo MINIMUM POSITION
SC_GET_PP = "PP"	## servo command syntax for feedback with PRESENT POSITION
SC_GET_GP = "GP"	## servo command syntax for feedback with GOAL POSITION
SC_GET_PS = "PS"	## servo command syntax for feedback with PRESENT SPEED
SC_GET_GS = "GS"	## servo command syntax for feedback with GOAL SPEED
SC_GET_PT = "PT"	## servo command syntax for feedback with PRESENT TEMPERATURE (in Celsius)
SC_GET_PL = "PL"	## servo command syntax for feedback with PRESENT LOAD
SC_GET_TM = "TM" 	## servo command syntax for feedback with MAX TORQUE
SC_GET_TL = "TL"  	## servo command syntax for feedback with TORQUE LIMIT
SC_GET_TS = "TS"  	## servo command syntax for feedback with TORQUE ENABLE
SC_GET_ER = "ER"  	## servo command syntax for feedback with error returned from AX_ALARM_LED
SC_GET_DP = "DP"  	## servo command syntax for default positions
SC_GET_DS = "DS"  	## servo command syntax for default speed
baud_rate = 19200	#9600

## from MAKIv14.h
SERVOCOUNT = 6	## MAKIv1.4 has 6 servos
LR = 1	## EYELID_RIGHT
LL = 2	## EYELID_LEFT
EP = 3	## EYE_PAN
ET = 4	## EYE_TILT
HT = 5	## HEAD_TILT
HP = 6	## HEAD_PAN
F_VAL_SEQ = [ "LR", "LL", "EP", "ET", "HT", "HP" ]

## from MAKI-Arbotix-Interface.py
## servo control infix for type of feedback
FEEDBACK_SC = [ SC_GET_PP,
                SC_GET_GP,
		SC_GET_PS,
                SC_GET_PT,
                #SC_GET_PL,
                SC_GET_ER,
                SC_GET_TL
                 ]
FEEDBACK_TOPIC = [ "maki_feedback_pres_pos",
                        "maki_feedback_goal_pos",
                        "maki_feedback_pres_speed",
                        "maki_feedback_pres_temp",
                        #"maki_feedback_pres_load",
                        "maki_feedback_error",
                        "maki_feedback_torque_limit",
                        ]
## Yet more MAKI related global variables; 2016-02-22
prewarn_temp_thresh = 60	## Celcius
warn_temp_thresh = 67	## Celcius
fail_temp_thresh = 70	## Celcius
ht_tl_enable = 1023
ht_tl_disable = 0
compliance_margin = 10	#5	## ax-12 default is 1 (cw compliance) + 1 (ccw compliance)
last_HT_hold_time = None
ht_holding_thresh = 60.0	## 60s 	## INSPIRE4 pilot3
PROTECT = True

## GLOBAL VARIABLES FOR PYTHON SCRIPT
VERBOSE_DEBUG = True	#False	## default is False
ARBOTIX_SIM = True
#ALIVE #= False		## make sure that all threads cleanly end
#INIT #= True		## flag to read first message from MAKI (will contain PP)


#####################
def setHeadTiltServoState( b_state ):
	global ht_tl_enable, ht_tl_disable
	global HT_TL_STATUS

	ht_tl_cmd = "HT" + SC_SET_TL 
	if b_state:
		ht_tl_cmd += str(ht_tl_enable) 
	else:
		ht_tl_cmd += str(ht_tl_disable) 
	ht_tl_cmd += TERM_CHAR_SEND 
	pubTo_maki_command( str(ht_tl_cmd) )
	sleep(0.5)
	pubTo_maki_command( SC_FEEDBACK + SC_GET_TL + TERM_CHAR_SEND )

	## if head tilt torque limit is already at 1023,
	## explicitly reset last_HT_hold_time
	if HT_TL_STATUS and b_state:
		resetAutoRelaxTimer()

#####################
def autoFlexHeadTiltServo():
	global makiGP, makiPP
	global makiTL
	global compliance_margin
	global ht_tl_disable, ht_tl_enable
	global override_auto_flex

	#print "autoFlexHeadTiltServo: BEGIN"
	if override_auto_flex:	return

	## if goal position and present position ARE NOT within *some threshold*, then ENABLE the head tilt servo
	#print "makiTL[HT]=" + str(makiTL["HT"])
	if (makiTL["HT"] == ht_tl_disable):
		#print "makiGP[HT]=" + str(makiGP["HT"]) + ", makiPP[HT]=" + str(makiPP["HT"]) + ". diff=" + str(abs(makiGP["HT"]-makiPP["HT"]))
		if (abs(makiGP["HT"] - makiPP["HT"]) > compliance_margin):
			rospy.loginfo("run autoFlexHeadTiltServo")
			if VERBOSE_DEBUG:	rospy.logdebug("Nicely enable head tilt motor's torque limit; don't want to jerk MAKI's neck")

			# step 1: save the goal position
			_ht_gp = makiGP["HT"]
			# step 2: set goal position to present position
			# precent robot from jerking into position
			pubTo_maki_command( "HT" + SC_SET_GP + str(makiPP["HT"]) + TERM_CHAR_SEND )
			sleep(1)
			# step 3: enable HT TL
			pubTo_maki_command( "HT" + SC_SET_TL + str(ht_tl_enable) + TERM_CHAR_SEND )
			sleep(0.25)
			# step 4: reset goal position
			pubTo_maki_command( "HT" + SC_SET_GP + str(_ht_gp) + TERM_CHAR_SEND )
			sleep(0.25)
			return True
		else:
			return False
#			# enable HT TL
#			if VERBOSE_DEBUG:	rospy.logdebug("Enable head tilt motor's torque limit... No need to be nice; head tilt shouldn't jerk")
#			pubTo_maki_command( "HT" + SC_SET_TL + str(ht_tl_enable) + TERM_CHAR_SEND )
	
	return False
	#print "autoFlexHeadTiltServo: END"

def autoRelaxHeadTiltServo():
	global makiTL
	global ht_tl_enable, ht_tl_disable
	global last_HT_hold_time
	_ret = False

	#print "autoRelaxHeadTiltServo: BEGIN"

	## if goal position and present position ARE within *some threshold*
	## and we are just holding the position, then DISABLE the head tilt servo
	## after *some period of time*

	#print "makiTL[HT]=" + str(makiTL["HT"])	# debugging
	if (makiTL["HT"] == ht_tl_enable):
		if timedoutHoldingHeadTiltServo():
			rospy.loginfo("run autoRelaxHeadTiltServo")
			if VERBOSE_DEBUG:	rospy.logdebug("auto relaxing head tilt servo after timing out from holding")
			pubTo_maki_command( "HT" + SC_SET_TL + str(ht_tl_disable) + TERM_CHAR_SEND)
			pubTo_maki_command( str(SC_FEEDBACK + SC_GET_TL + TERM_CHAR_SEND) )
			_ret = True
	else:
		if (last_HT_hold_time != None):
			#last_HT_hold_time = None

			## reset if HT ht_tl_disable
			resetAutoRelaxTimer()
			rospy.logdebug( "reset last_HT_hold_time to None if ht_tl_disable" )
	#print "autoRelaxHeadTiltServo: END"
	return _ret

## helper function for autoRelaxHeadTiltServo
def timedoutHoldingHeadTiltServo():
	global makiGP, makiPP
	global makiPS
	global compliance_margin
	global last_HT_hold_time, ht_holding_thresh

	#print "timedoutHoldingHeadTiltServo: BEGIN"
	_ret = False

	#rospy.logdebug( "last_HT_hold_time = " + str(last_HT_hold_time) )

	## TODO: INSTEAD OF FIXED THRESHOLD FOR compliance_margin,
	## WE SHOULD LOOK TO SEE IF THE DIFFERENCE IS STABLE OVER TIME
	if (abs(makiGP["HT"] - makiPP["HT"]) <= compliance_margin):
		## present position is within goal position +/- compliance margin
		#print "present position is within goal position +/- compliance margin"
		if (last_HT_hold_time == None):
			rospy.logdebug( "last_HT_hold_time = None; set new..." )
			last_HT_hold_time = timer()
			_ret = False
		elif ((timer() - last_HT_hold_time) > ht_holding_thresh):
			#rospy.logdebug( "*** ding ding ding ***" )
			_ret = True
			#last_HT_hold_time = None
			rospy.loginfo( "All done!!!!!! More than " + str(ht_holding_thresh) + " seconds elapsed. current_time=" + str(timer()) + ", last_HT_hold_time=" + str(last_HT_hold_time) + ". diff=" + (str(timer() - last_HT_hold_time)) )
		else:
			#rospy.logdebug( "needs more time. current_time=" + str(timer()) + ", last_HT_hold_time=" + str(last_HT_hold_time) + ". diff=" + (str(timer() - last_HT_hold_time)) )
			#rospy.logdebug( "needs more time. diff=" + (str(timer() - last_HT_hold_time)) )
			_ret = False
	else:
	#	if (makiPS["HT"] > 0):
	#		rospy.logdebug( "still moving to GP..." )
	#		## present position is moving towards goal position
	#		if (last_HT_hold_time != None):
	#			rospy.logdebug( "update to last_HT_hold_time = None" )
	#			last_HT_hold_time = None
		if (last_HT_hold_time != None):
			rospy.logwarn( "Beyond compliance margin... reset last_HT_hold_time" )
			#last_HT_hold_time = None
			resetAutoRelaxTimer()
		_ret = False

	#print "timedoutHoldingHeadTiltServo: END"
	return _ret
		
def resetAutoRelaxTimer():
	global last_HT_hold_time
	last_HT_hold_time = None

def autoProtectHeadTiltServo():
	global makiPT, makiTL
	global ht_tl_disable, ht_tl_enable
	global PROTECT
	global fail_temp_thresh, warn_temp_thresh, prewarn_temp_thresh
	global override_auto_flex

	#print "autoProtectHeadTiltServo: BEGIN"
	if (override_auto_flex and
		(makiPT["HT"] <= prewarn_temp_thresh)):
		override_auto_flex = False

	ht_tl_disable_cmd = "HT" + SC_SET_TL + str(ht_tl_disable) + TERM_CHAR_SEND 
	ht_tl_feedback_cmd = SC_FEEDBACK + SC_GET_TL + TERM_CHAR_SEND

	if (makiTL["HT"] == ht_tl_enable):
		#print "makiPT[HT]=" + str(makiPT["HT"])
		if (makiPT["HT"] > fail_temp_thresh):
			rospy.logerr("HT servo above warn temperature zone; nearing fail temperature zone: " + str(fail_temp_thresh))
			pubTo_maki_command( str(ht_tl_disable_cmd) )
			rospy.logerr("HT protection action: AUTO disable by setting torque limit to 0 -- DONE!")
			override_auto_flex = True
		elif (makiPT["HT"] > warn_temp_thresh):
			rospy.logwarn("HT servo nearing warn temperature zone: " + str(warn_temp_thresh))
			if PROTECT:	
				pubTo_maki_command( str(ht_tl_disable_cmd) )
				rospy.logwarn("HT protection action: AUTO disable by setting torque limit to 0 -- DONE!")
			else:
				rospy.logwarn("HT protection action: MANUAL disable needed by setting torque limit to 0 -- HELP!")
			override_auto_flex = True
		else:
			override_auto_flex = False
			pass
	#print "autoProtectHeadTiltServo: END"

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
		#if VERBOSE_DEBUG:	rospy.logdebug( str(commandOut) + " matched maki_msg_format" )
	elif not commandOut.endswith( str(TERM_CHAR_SEND) ):
		## append the missing TERM_CHAR_SEND
		commandOut += str(TERM_CHAR_SEND)
		_pub_flag = True
		if VERBOSE_DEBUG:	rospy.logdebug( str(commandOut) + " added TERM_CHAR_SEND" )
	else:
		rospy.logerr( "Incorrect message format" + str(commandOut) )

	#if VERBOSE_DEBUG: rospy.logdebug( str(commandOut) )
	
	if _pub_flag and not rospy.is_shutdown():
		rospy.loginfo( str(commandOut) )
		pub_cmd.publish( commandOut )


#####################
def parseRecvMsg ( recv_msg ):
	global makiPP, makiGP
	global makiPS
	global makiER
	global makiTL
	global makiPL, makiPT
	global init_dict
	global FEEDBACK_SC
	global VERBOSE_DEBUG
	global feedback_template
	global HT_TL_STATUS
	global last_recv_msg_time
	global run_once_aFHTS, run_once_aRHTS
	global last_HT_hold_time

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
			if VERBOSE_DEBUG: rospy.logdebug( prefix + "=" + str(makiPP) )
		if not init_dict[prefix]:	init_dict[prefix] = True
		last_recv_msg_time = timer()

	elif prefix=="GP":	#SC_GET_GP	# goal position
		if not ( tmp_dict == makiGP ):
			makiGP.update(tmp_dict)
			if VERBOSE_DEBUG: rospy.logdebug( prefix + "=" + str(makiGP) )
		if not init_dict[prefix]:	init_dict[prefix] = True
		last_recv_msg_time = timer()

	elif prefix=="PS":	#SC_GET_PS	# present speed
		if not ( tmp_dict == makiPS ):
			makiPS.update(tmp_dict)
			if VERBOSE_DEBUG: rospy.logdebug( prefix + "=" + str(makiPS) )
		if not init_dict[prefix]:	init_dict[prefix] = True
		last_recv_msg_time = timer()

		## reset for autoRelax
		if (makiPS["HT"] > 0):
			rospy.logdebug( "still moving to GP..." )
			## present position is moving towards goal position
			if (last_HT_hold_time != None):
				rospy.logdebug( "update to last_HT_hold_time = None" )
				resetAutoRelaxTimer()
				#last_HT_hold_time = None

	elif prefix=="ER":	#SC_GET_ER:	# error
		if not ( tmp_dict == makiER ):
			makiER.update(tmp_dict)
			if VERBOSE_DEBUG: rospy.logdebug( prefix + "=" + str(makiER) )
		if not init_dict[prefix]:	init_dict[prefix] = True
		## do not reset last_recv_msg_time here since
		## we expect these messages every 30s

	elif prefix=="PT":	#SC_GET_PT	# present temperature
		if not ( tmp_dict == makiPT ):
			makiPT.update(tmp_dict)
			if VERBOSE_DEBUG: rospy.logdebug( prefix + "=" + str(makiPT) )
		if not init_dict[prefix]:	init_dict[prefix] = True
		last_recv_msg_time = timer()

	elif prefix=="PL":	#SC_GET_PL	# present load
		if not ( tmp_dict == makiPL ):
			makiPL.update(tmp_dict)
			if VERBOSE_DEBUG: rospy.logdebug( prefix + "=" + str(makiPL) )
		if not init_dict[prefix]:	init_dict[prefix] = True
		last_recv_msg_time = timer()

	elif prefix=="TL":	#SC_GET_TL	# torque limit
		if not ( tmp_dict == makiTL ):
			makiTL.update(tmp_dict)
			if (makiTL["HT"] > ht_tl_disable):
				HT_TL_STATUS = True
			else:
				HT_TL_STATUS = False
				run_once_aFHTS = False
				run_once_aRHTS = False
			if VERBOSE_DEBUG: rospy.logdebug( prefix + "=" + str(makiTL) )
		if not init_dict[prefix]:	init_dict[prefix] = True
		last_recv_msg_time = timer()

	else:
		rospy.logerr( "parseRecvMsg: ERROR! Unknown prefix: " + str(prefix) )
		return

	#print "parseRecvMsg: END"


#####################
## TODO: Replace this with a service
def updateHTServoStatus( b_status ):
	rospy.loginfo( str(b_status) )
	if b_status.data:	## enable head tilt motor's torque limit
		setHeadTiltServoState(True)
	else:	## disable head tilt motor's torque limit
		setHeadTiltServoState(False)
	return

#####################
#def updateMAKICommand( msg ):
#	global maki_command
#	#rospy.logdebug(rospy.get_caller_id() + ": I heard %s", msg.data)
#
#	## filter out feedback requests using regex
#	_tmp = re.search("^F[A-Y]{2}Z$", msg.data)
#
#	## YES, msg.data is a feedback request
#	if _tmp != None:
#		pass
#	else:
#		maki_command = msg.data 
#
#	return

def updateStatus( ):
	global FEEDBACK_SC
	global ALIVE
	global last_recv_msg_time
	_rate = 1.5	## emperically determined; 2016-02-22
	_elapsed_recv_msg_time = 0

	while ALIVE and not rospy.is_shutdown():
		_elapsed_recv_msg_time = abs(timer() - last_recv_msg_time)
		if (_elapsed_recv_msg_time > 30):
			rospy.logwarn( "No feedback received from /maki_feedback_* topics for " + str(_elapsed_recv_msg_time) + " seconds...!")

		for _servo_command in FEEDBACK_SC:
			#print _servo_command
			pubTo_maki_command( str(SC_FEEDBACK + _servo_command + TERM_CHAR_SEND) )
			sleep(_rate)	# seconds
	# end	while ALIVE
	return


#####################
def signal_handler(signal, frame):
	global ALIVE

	setHeadTiltServoState(False)
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
                rospy.init_node('maki_head_tilt_servo_monitor', log_level=rospy.DEBUG)
		rospy.logdebug("log_level=rospy.DEBUG")
        else:
                rospy.init_node('maki_head_tilt_servo_monitor')       ## defaults to log_level=rospy.INFO
	rospy.logdebug( str(_fname) + ": END")
	return

def initPubMAKI():
	global pub_cmd
	global pub_status
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
	pub_status = rospy.Publisher( "maki_head_tilt_servo/status", Bool, queue_size = 1 )

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

	rospy.Subscriber( "/maki_head_tilt_servo/status", Bool, updateHTServoStatus )
	rospy.logdebug( "now subscribed to /maki_head_tilt_servo/status" )

	#rospy.Subscriber( "/maki_command", String, updateMAKICommand )
	#rospy.logdebug( "now subscribed to /maki_command" )

	rospy.logdebug( str(_fname) + ": END")
	return


#####################
if __name__ == '__main__':

	global ALIVE
	global VERBOSE_DEBUG
	global INIT, init_dict
	global makiPP, makiGP
	global makiER
	global makiTL
	global makiPL, makiPT
	global override_auto_flex
	#global data_logger_status, maki_command
	global HT_TL_STATUS
	global run_once_aFHTS, run_once_aRHTS
	global last_recv_msg_time
	global last_HT_hold_time

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
	makiPS = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )
	makiER = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )
	makiTL = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )
	makiPT = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )
	makiPL = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )
	override_auto_flex = False	## only True when head tilt motor is getting too hot!!
	#maki_command = ""
	HT_TL_STATUS = None
	last_HT_hold_time = None
	run_once_aFHTS = False
	run_once_aRHTS = False

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

	## STEP 4: START POLLING ROBOT FOR STATUS
	ALIVE = True
	thread_updateStatus = threading.Thread(target=updateStatus, args=())
	thread_updateStatus.setDaemon(True)	# make sure to set this; otherwise, stuck with this thread open
	thread_updateStatus.start()

	## ---------------------------------
	## END OF INITIALIZATION
	## ---------------------------------

	## main loop will automatically accomodate head tilt motor
	last_HT_hold_time = None
	ALIVE = True
	run_once_aFHTS = False
	run_once_aRHTS = False
	while ALIVE and not rospy.is_shutdown():
		autoProtectHeadTiltServo()
		## prevent head nodding side effect
		if not run_once_aFHTS:
			run_once_aFHTS = autoFlexHeadTiltServo()
		## prevent from flooding serial
		if not run_once_aRHTS:
			run_once_aRHTS = autoRelaxHeadTiltServo()
		sleep(0.25)
#		pass	## nop to allow stdin to pass through

	#rospy.spin()	## keeps python from exiting until this node is stopped

	print "__main__: END"



