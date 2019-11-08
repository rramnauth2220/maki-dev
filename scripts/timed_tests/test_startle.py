#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from timed_test import timedTest
from head_tilt_timed_test import headTiltTimedTest
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt

HT_STARTLE = 525

########################
## Test startle behavior
##
## Description: from neutral, MAKI rapidly opens eyes wide (LL_OPEN_MAX)
##	and head lifts slightly (HT). Eye tilt (ET) compensates for HT
########################
#class startleTest( timedTest, headTiltTimedTest ):
class startleTest( headTiltTimedTest ):
	## variables private to this class
	## all instances of this class share the same value
	# none


	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		headTiltTimedTest.__init__( self, verbose_debug, ros_pub )
		## add anything else needed by an instance of this subclass
		self.DC_helper = dynamixelConversions()

	#########################
	## base class listing of functions and variables
	#########################
	#def startTimedTest(self, makiPP):
	#	self.mTT_ITERRUPT = False
	#	self.makiPP = makiPP
	#
	#def stopTimedTest(self):
	#	self.mTT_ITERRUPT = True
	#
	#def exitTimedTest(self):
	#	self.ALIVE = False
	#	self.mTT_ITERRUPT = True
	#
	#def update(self, makiPP):
	#	self.makiPP = makiPP

	def macroStartle( self ):
		_sww_wi = ROS_sleepWhileWaiting_withInterrupt( verbose_debug=self.VERBOSE_DEBUG )
		self.ALIVE = True

		## this is a nested while loop
		_print_once = True
		while self.ALIVE:
			if not rospy.is_shutdown():
				pass
			else:
				break	## break out of outer while loop

			if _print_once:
				rospy.logdebug("Entering macroStartle outer while loop")
				_print_once = False

			if self.mTT_ITERRUPT:	
				#print "start sleep 5"
				self.SWW_WI.sleepWhileWaiting( 5 )
				#print "end sleep 5"
				continue	## begin loop again from the beginning skipping below
				#print "shouldn't get here"


			rospy.logdebug("Entering macroStartle inner while loop")
			_startle_count = 0
			while not self.mTT_ITERRUPT:

				_startle_count = startleTest.helper_macroStartle( self, _startle_count )

				## try to nicely end testing 
				if self.mTT_ITERRUPT:
					_sww_wi.sleepWhileWaiting( 1 )	## make sure to wait for message to reach Dynamixel servo
					timedTest.pubTo_maki_command( self, "reset" )
					_sww_wi.sleepWhileWaiting( 1 )	## make sure to wait for message to reach Dynamixel servo
				else:
					pass

			# end	while not self.mTT_ITERRUPT
		# end	while self.ALIVE 
		rospy.loginfo("END: After outer while loop in macroStartleTest()")

	def helper_macroStartle( self, startle_count, read_time=1.0 ):
		_sww_wi = ROS_sleepWhileWaiting_withInterrupt( verbose_debug=self.VERBOSE_DEBUG )

		_start_lid_open_time = None
		_finish_lid_open_time = None
		_total_lid_open_time = 0

		## neutral to LL_OPEN_MAX
		NEUTRAL_LL = self.makiPP["LL"]
		_delta_ticks_LL = abs( NEUTRAL_LL - LL_OPEN_MAX )
		## neutral to HT_STARTLE
		#NEUTRAL_HT = self.makiPP["HT"]
		NEUTRAL_HT = HT_MIDDLE
		rospy.logerr("HT_STARTLE=" + str(HT_STARTLE))
		_delta_ticks_HT = abs( NEUTRAL_HT - HT_STARTLE )
		## neutral to ET_UP
		NEUTRAL_ET = ET_MIDDLE
		_delta_ticks_ET = abs( NEUTRAL_ET - ET_UP )

		## maki_cmd 
		_m_cmd_prefix_LL = "LL" + str(SC_SET_GS)
		_m_cmd_lid_neutral_open = "LL" + str(SC_SET_GP) + str(LL_OPEN_MAX) #+ str(TERM_CHAR_SEND)
		_m_cmd_lid_open_neutral = "LL" + str(SC_SET_GP) + str(NEUTRAL_LL) #+ str(TERM_CHAR_SEND)
		_m_cmd_prefix_HT = "HT" + str(SC_SET_GS)
		_m_cmd_head_neutral_up = "HT" + str(SC_SET_GP) + str(HT_STARTLE) #+ str(TERM_CHAR_SEND)
		_m_cmd_head_up_neutral = "HT" + str(SC_SET_GP) + str(NEUTRAL_HT) #+ str(TERM_CHAR_SEND)
		_m_cmd_prefix_ET = "ET" + str(SC_SET_GS)
		_m_cmd_eye_neutral_up = "ET" + str(SC_SET_GP) + str(ET_UP) #+ str(TERM_CHAR_SEND)
		_m_cmd_eye_up_neutral = "ET" + str(SC_SET_GP) + str(NEUTRAL_ET) #+ str(TERM_CHAR_SEND)

		headTiltTimedTest.enableHT( self )

		rospy.loginfo("-----------------")
		for _ms_duration in range(300, 100, -10):
			## calculate GoalSpeed
			_gs_LL = int( self.DC_helper.getGoalSpeed_ticks_durationMS(_delta_ticks_LL, _ms_duration) ) + 1
			_gs_HT = int( self.DC_helper.getGoalSpeed_ticks_durationMS(_delta_ticks_HT, _ms_duration) ) + 1
			_gs_ET = int( self.DC_helper.getGoalSpeed_ticks_durationMS(_delta_ticks_ET, _ms_duration) ) + 1

			## preset eyelid to open GoalSpeed
			_pub_cmd = _m_cmd_prefix_LL + str(_gs_LL) 
			_pub_cmd += _m_cmd_prefix_HT + str(_gs_HT)
			_pub_cmd += _m_cmd_prefix_ET + str(_gs_ET)
			_pub_cmd += str(TERM_CHAR_SEND)
			timedTest.pubTo_maki_command( self, str(_pub_cmd) )
			_sww_wi.sleepWhileWaitingMS( 100, 0.05 )

			## publish eyelid neutral to LL_OPEN_MAX GoalPosition
			timedTest.pubTo_maki_command( self, str(_m_cmd_lid_neutral_open + _m_cmd_head_neutral_up + _m_cmd_eye_neutral_up + TERM_CHAR_SEND) )
			_start_lid_open_time = rospy.get_time()
			_sww_wi.sleepWhileWaitingMS( _ms_duration, 0.01 )
			_finish_lid_open_time = rospy.get_time()

			_sww_wi.sleepWhileWaitingMS( 500, 0.01 )
			## preset eyelid to neutral GoalSpeed
			## return to neutral slowly
			_pub_cmd = _m_cmd_prefix_LL + str(31)	## used spreadsheet to calculate these values
			_pub_cmd += _m_cmd_prefix_HT + str(26)
			_pub_cmd += _m_cmd_prefix_ET + str(100)
			_pub_cmd += str(TERM_CHAR_SEND)
			timedTest.pubTo_maki_command( self, str(_pub_cmd) )
			_sww_wi.sleepWhileWaitingMS( 100, 0.05 )

			## publish eyelid open to neutral GoalPosition
			timedTest.pubTo_maki_command( self, str(_m_cmd_lid_open_neutral + _m_cmd_head_up_neutral + _m_cmd_eye_up_neutral + TERM_CHAR_SEND) )
			_sww_wi.sleepWhileWaiting( 1 )

			_total_lid_open_time = abs( _finish_lid_open_time - _start_lid_open_time )
			startle_count += 1
			rospy.loginfo( "Startle #" + str(startle_count) 
				+ ": GS_LL = " + str(_gs_LL)
				+ ", GS_HT = " + str(_gs_HT)
				+ ", GS_ET = " + str(_gs_ET)
				+ "; expected time = " + str( _ms_duration )
				+ "ms; actual lid_open time = " + str( _total_lid_open_time ) 
				+ " seconds" )

			rospy.loginfo("-----------------")
			## make it easier to read
			_sww_wi.sleepWhileWaiting( read_time, 0.5 )

			## try to nicely end test
			if self.mTT_ITERRUPT:
				break	## break out of for loop
			else:
				## reset timers
				_start_lid_open_time = None
				_finish_lid_open_time = None
				_total_lid_open_time = 0
		# end	for _ms_duration

		headTiltTimedTest.disableHT( self )
		return startle_count


	def pubTo_maki_command_presetGoalSpeed( self, dict_servo_gs ):
		_pub_cmd = ""
		_pub_flag = False

		for _servo, _gs in dict_servo_gs.iteritems():
			if (_gs > 0) and (_gs < 1024):
				_pub_cmd += _servo + str(SC_SET_GS) + str(_gs)
				_pub_flag = True
			elif (_gs == 0):
				rospy.logerr("setting GoalSpeed to 0 (unlimited) is not allowed!")
				rospy.logwarn( _servo + " GoalSpeed will not be changed")
			else:
				rospy.logerr("calculated GoalSpeed is out of bounds (0, 1023]; _gs = " + str(_gs))

		if _pub_flag:
			_pub_cmd += str(TERM_CHAR_SEND)
			rospy.logerr( str(_pub_cmd) )
			timedTest.pubTo_maki_command( self, str(_pub_cmd) )
		return _pub_cmd


	def helper_macroEmptyTest( self, pass_count, read_time=1.0 ):
		_sww_wi = ROS_sleepWhileWaiting_withInterrupt( verbose_debug=self.VERBOSE_DEBUG )

		_start_pass_time = None
		_finish_pass_time = None
		_total_pass_time = 0

		## maki_command 
		_m_cmd = "reset"

		rospy.loginfo("-----------------")
		for _pass_time in range(0,3):	## 0, 1, 2
			_start_pass_time = rospy.get_time()
			rospy.logdebug( "resetting..." )
			timedTest.pubTo_maki_command( self, str(_m_cmd) )
			_sww_wi.sleepWhileWaitingMS( (_pass_time+1)*1000, 0.01 )
			_finish_pass_time = rospy.get_time()
			_total_pass_time = abs(_finish_pass_time - _start_pass_time)

			pass_count += 1
			#rospy.loginfo( "Completed " + str(pass_count) + " passes" )
			rospy.loginfo( "Pass #" + str(pass_count) + ": full pass time = " 
				+ str( _total_pass_time ) )

			rospy.loginfo("-----------------")
			## make it easier to read
			_sww_wi.sleepWhileWaiting( read_time, 0.5 )

			## try to nicely end test
			if self.mTT_ITERRUPT:
				break	## break out of for loop
			else:
				## reset timers
				_start_pass_time = None
				_finish_pass_time = None
				_total_pass_time = 0
		# end	for _pass_time in range(0,3):

		return pass_count




