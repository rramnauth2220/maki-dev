#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string

import random

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from timed_test import timedTest
from head_tilt_timed_test import headTiltTimedTest
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt


########################
## Test asleep / awake behavior
##
## Description: ASLEEP: from neutral, MAKI closes eyes and 
##	puts head down and to its left
##
##	AWAKE: from ASLEEP, MAKI opens eyes and raises head 
##	to neutral and faces front
########################
class asleepAwakeTest( headTiltTimedTest ):
	## variables private to this class
	## all instances of this class share the same value
	# none


	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		headTiltTimedTest.__init__( self, verbose_debug, ros_pub )
		## add anything else needed by an instance of this subclass
		self.DC_helper = dynamixelConversions()

	####################
	##
	##	To run, publish to /maki_macro:
	##		alseep_test
	##
	##	To stop, publish to /maki_macro:
	##		end
	##		reset
	####################
	def macroAsleep( self ):
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
				rospy.logdebug("Entering macroAsleep outer while loop")
				_print_once = False

			if self.mTT_INTERRUPT:	
				#print "start sleep 5"
				_sww_wi.sleepWhileWaiting( 5 )
				#print "end sleep 5"
				continue	## begin loop again from the beginning skipping below
				#print "shouldn't get here"


			rospy.logdebug("Entering macroAsleep inner while loop")
			_asleep_count = 0
			while not self.mTT_INTERRUPT:

				_asleep_count = asleepAwakeTest.helper_macroAsleep( self, _asleep_count )

				## try to nicely end testing 
				if self.mTT_INTERRUPT:
					_sww_wi.sleepWhileWaiting( 1 )	## make sure to wait for message to reach Dynamixel servo
					timedTest.pubTo_maki_command( self, "reset" )
					_sww_wi.sleepWhileWaiting( 1 )	## make sure to wait for message to reach Dynamixel servo
				else:
					pass

			# end	while not self.mTT_INTERRUPT
		# end	while self.ALIVE 
		timedTest.pubTo_maki_command( self, "reset" )
		_sww_wi.sleepWhileWaiting(resetting_time)
		headTiltTimedTest.disableHT()
		timedTest.pubTo_maki_command( self, "HT" + str(SC_SET_TL) + str(ht_tl_disable) + str(TERM_CHAR_SEND) )
		rospy.loginfo("END: After outer while loop in macroAsleepTest()")

	def helper_macroAsleep( self, asleep_count, read_time=1.0 ):
		_sww_wi = ROS_sleepWhileWaiting_withInterrupt( verbose_debug=self.VERBOSE_DEBUG )

		_start_asleep_time = None
		_finish_asleep_time = None
		_total_asleep_time = 0

		## neutral to LL_CLOSE_MAX
		#NEUTRAL_LL = self.makiPP["LL"]
		NEUTRAL_LL = LL_OPEN_DEFAULT
		_delta_ticks_LL = abs( NEUTRAL_LL - LL_CLOSE_MAX )
		## neutral to HT_DOWN
		NEUTRAL_HT = HT_MIDDLE
		_delta_ticks_HT = abs( NEUTRAL_HT - HT_DOWN )
		## neutral to HP_LEFT (same delta as HP_RIGHT)
		NEUTRAL_HP = HP_FRONT
		_delta_ticks_HP = abs( NEUTRAL_HP - HP_LEFT )
		## neutral to ET_DOWN
		NEUTRAL_ET = ET_MIDDLE
		_delta_ticks_ET = abs( NEUTRAL_ET - ET_DOWN )
		## neutral to EP_LEFT (same delta as EP_RIGHT)
		NEUTRAL_EP = EP_FRONT
		_delta_ticks_EP = abs( NEUTRAL_EP - EP_LEFT )
		rospy.logerr("delta_ticks[LL, HT, HP, ET, EP] = " + str(_delta_ticks_LL)
			+ ", " + str(_delta_ticks_HT)
			+ ", " + str(_delta_ticks_HP)
			+ ", " + str(_delta_ticks_ET)
			+ ", " + str(_delta_ticks_EP) )

		## maki_cmd 
		_m_cmd_prefix_LL = "LL" + str(SC_SET_GP)
		_m_cmd_lid_neutral_close = _m_cmd_prefix_LL + str(LL_CLOSE_MAX) 
		_m_cmd_lid_close_neutral = _m_cmd_prefix_LL + str(NEUTRAL_LL) 
		_m_cmd_prefix_LL = "LL" + str(SC_SET_GS)

		_m_cmd_prefix_HT = "HT" + str(SC_SET_GP)
		_m_cmd_head_neutral_down = _m_cmd_prefix_HT + str(HT_DOWN) 
		_m_cmd_head_down_neutral = _m_cmd_prefix_HT + str(NEUTRAL_HT) 
		_m_cmd_prefix_HT = "HT" + str(SC_SET_GS)

		_m_cmd_prefix_HP = "HP" + str(SC_SET_GP)
		_m_cmd_head_neutral_left = _m_cmd_prefix_HP + str(HP_LEFT)
		_m_cmd_head_neutral_right = _m_cmd_prefix_HP + str(HP_RIGHT)
		_m_cmd_head_lr_neutral = _m_cmd_prefix_HP + str(NEUTRAL_HP)
		_m_cmd_prefix_HP = "HP" + str(SC_SET_GS)

		_m_cmd_prefix_ET = "ET" + str(SC_SET_GP)
		_m_cmd_eye_neutral_down = _m_cmd_prefix_ET + str(ET_DOWN) 
		_m_cmd_eye_down_neutral = _m_cmd_prefix_ET + str(NEUTRAL_ET) 
		_m_cmd_prefix_ET = "ET" + str(SC_SET_GS)

		_m_cmd_prefix_EP = "EP" + str(SC_SET_GP)
		_m_cmd_eye_neutral_left = _m_cmd_prefix_EP + str(EP_LEFT)
		_m_cmd_eye_neutral_right = _m_cmd_prefix_EP + str(EP_RIGHT)
		_m_cmd_eye_lr_neutral = _m_cmd_prefix_EP + str(NEUTRAL_EP)
		_m_cmd_prefix_HP = "EP" + str(SC_SET_GS)

		headTiltTimedTest.enableHT( self )
		_sww_wi.sleepWhileWaitingMS(1000, 0.05)
		rospy.loginfo("-----------------")
		for _ms_duration in range(5000, 1000, -500):	## [5000, 1500] ms
			headTiltTimedTest.enableHT( self )

			## calculate GoalSpeed
			_gs_HT = int( self.DC_helper.getGoalSpeed_ticks_durationMS(_delta_ticks_HT, _ms_duration) ) + 1
			_gs_HP = int( self.DC_helper.getGoalSpeed_ticks_durationMS(_delta_ticks_HP, _ms_duration) ) + 1
			## eyes should finish first
			_ms_duration_eyes = int( (float(_ms_duration) * 0.3) + 0.5 )
			_gs_LL = int( self.DC_helper.getGoalSpeed_ticks_durationMS(_delta_ticks_LL, _ms_duration_eyes) ) + 1
			_gs_ET = int( self.DC_helper.getGoalSpeed_ticks_durationMS(_delta_ticks_ET, _ms_duration_eyes) ) + 1
			_gs_EP = int( self.DC_helper.getGoalSpeed_ticks_durationMS(_delta_ticks_EP, _ms_duration_eyes) ) + 1
			rospy.logerr("ms_duration_eyes = " + str(_ms_duration_eyes) + "ms")

			### preset eyelid to open GoalSpeed
			#_pub_cmd = _m_cmd_prefix_LL + str(_gs_LL) 
			#_pub_cmd += _m_cmd_prefix_HT + str(_gs_HT)
			#_pub_cmd += _m_cmd_prefix_HP + str(_gs_HP)
			#_pub_cmd += _m_cmd_prefix_ET + str(_gs_ET)
			#_pub_cmd += _m_cmd_prefix_EP + str(_gs_EP)
			#_pub_cmd += str(TERM_CHAR_SEND)
			#timedTest.pubTo_maki_command( self, str(_pub_cmd) )
			#_sww_wi.sleepWhileWaitingMS( 100, 0.05 )

			## publish neutral to GoalPosition: head down, eyes down, eyelids closed
			_pub_cmd = str(_m_cmd_lid_neutral_close + _m_cmd_head_neutral_down + _m_cmd_eye_neutral_down )
			## used for choosing direction of falling asleep
			## 0 = left, 1 = right
			#if (random.randint(0,1) == 0):
			if (0 == 0):
				_pub_cmd += str(_m_cmd_head_neutral_left + _m_cmd_eye_neutral_left)
			else:
				_pub_cmd += str(_m_cmd_head_neutral_right + _m_cmd_eye_neutral_right)
			_pub_cmd += str(SC_SET_IPT) + str(_ms_duration)
			_pub_cmd += str(TERM_CHAR_SEND) 
			timedTest.pubTo_maki_command( self, str(_pub_cmd) )
			_start_asleep_time = rospy.get_time()
			#rospy.logerr("Start ASLEEP...")
			_sww_wi.sleepWhileWaitingMS( _ms_duration, 0.01, False )
			_finish_asleep_time = rospy.get_time()
			_total_asleep_time = abs( _finish_asleep_time - _start_asleep_time )
			asleep_count += 1
			rospy.logerr( "Asleep #" + str(asleep_count) 
				+ ": GS_LL = " + str(_gs_LL)
				+ ", GS_HT = " + str(_gs_HT)
				+ ", GS_HP = " + str(_gs_HP)
				+ ", GS_ET = " + str(_gs_ET)
				+ ", GS_EP = " + str(_gs_EP)
				+ "; expected time = " + str( _ms_duration )
				+ "ms; actual alseep time = " + str( _total_asleep_time ) 
				+ " seconds" )

			rospy.loginfo("-----------------")
			## make it easier to read
			_sww_wi.sleepWhileWaiting( read_time, 0.5 )

			#_sww_wi.sleepWhileWaitingMS( 1000, 0.01 )
			## preset eyelid to neutral GoalSpeed
			## return to neutral in a discernable manner from falling asleep
			#_pub_cmd = _m_cmd_prefix_LL + str(16)	## used spreadsheet to calculate these values
			#_pub_cmd += _m_cmd_prefix_HT + str(13)
			#_pub_cmd += _m_cmd_prefix_HP + str(88)
			#_pub_cmd += _m_cmd_prefix_EP + str(22)
			#_pub_cmd += _m_cmd_prefix_ET + str(50)
			#_pub_cmd += str(TERM_CHAR_SEND)
			#timedTest.pubTo_maki_command( self, str(_pub_cmd) )
			#_sww_wi.sleepWhileWaitingMS( 100, 0.05 )
			##timedTest.pubTo_maki_command( self, "reset" )
			##_sww_wi.sleepWhileWaitingMS( 100, 0.05 )
			##timedTest.pubTo_maki_command( self, "reset" )
			##_sww_wi.sleepWhileWaitingMS( 100, 0.05 )

			## publish eyelid open to neutral GoalPosition
			_pub_cmd = str(_m_cmd_lid_close_neutral + _m_cmd_head_down_neutral + _m_cmd_eye_down_neutral )
			_pub_cmd += str(_m_cmd_head_lr_neutral + _m_cmd_eye_lr_neutral)
			_pub_cmd += str(SC_SET_IPT) + str(resetting_time)
			_pub_cmd += str(TERM_CHAR_SEND) 
			timedTest.pubTo_maki_command( self, str(_pub_cmd) )
			rospy.loginfo("Start waiting for reset neutral position and speed")
			_sww_wi.sleepWhileWaitingMS( resetting_time+1000, 0.05 )
			rospy.loginfo("Done waiting for reset neutral position and speed")

			#_total_asleep_time = abs( _finish_asleep_time - _start_asleep_time )
			#asleep_count += 1
			#rospy.loginfo( "Asleep #" + str(asleep_count) 
			#	+ ": GS_LL = " + str(_gs_LL)
			#	+ ", GS_HT = " + str(_gs_HT)
			#	+ ", GS_HP = " + str(_gs_HP)
			#	+ ", GS_ET = " + str(_gs_ET)
			#	+ ", GS_EP = " + str(_gs_EP)
			#	+ "; expected time = " + str( _ms_duration )
			#	+ "ms; actual alseep time = " + str( _total_asleep_time ) 
			#	+ " seconds" )

			#rospy.loginfo("-----------------")
			### make it easier to read
			#_sww_wi.sleepWhileWaiting( read_time, 0.5 )

			## try to nicely end test
			if self.mTT_INTERRUPT:
				break	## break out of for loop
			else:
				## reset timers
				_start_asleep_time = None
				_finish_asleep_time = None
				_total_asleep_time = 0
		# end	for _ms_duration

		return asleep_count


	####################
	##
	##	To run, publish to /maki_macro:
	##		awake_test
	##
	##	To stop, publish to /maki_macro:
	##		end
	##		reset
	####################
	def macroAwake( self ):
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
				rospy.logdebug("Entering macroAwake outer while loop")
				_print_once = False

			if self.mTT_INTERRUPT:	
				#print "start sleep 5"
				_sww_wi.sleepWhileWaiting( 5 )
				#print "end sleep 5"
				continue	## begin loop again from the beginning skipping below
				#print "shouldn't get here"


			rospy.logdebug("Entering macroAwake inner while loop")
			_awake_count = 0
			while not self.mTT_INTERRUPT:

				_awake_count = asleepAwakeTest.helper_macroAwake( self, _awake_count )

				## try to nicely end testing 
				if self.mTT_INTERRUPT:
					_sww_wi.sleepWhileWaiting( 1 )	## make sure to wait for message to reach Dynamixel servo
					timedTest.pubTo_maki_command( self, "reset" )
					_sww_wi.sleepWhileWaiting( 1 )	## make sure to wait for message to reach Dynamixel servo
				else:
					pass

			# end	while not self.mTT_INTERRUPT
		# end	while self.ALIVE 
		timedTest.pubTo_maki_command( self, "reset" )
		_sww_wi.sleepWhileWaiting(resetting_time)
		headTiltTimedTest.disableHT()
		timedTest.pubTo_maki_command( self, "HT" + str(SC_SET_TL) + str(ht_tl_disable) + str(TERM_CHAR_SEND) )
		rospy.loginfo("END: After outer while loop in macroAwakeTest()")

	def helper_macroAwake( self, awake_count, read_time=1.0 ):
		_sww_wi = ROS_sleepWhileWaiting_withInterrupt( verbose_debug=self.VERBOSE_DEBUG )

		_start_awake_time = None
		_finish_awake_time = None
		_total_awake_time = 0

		## neutral to LL_CLOSE_MAX
		NEUTRAL_LL = LL_OPEN_DEFAULT
		_delta_ticks_LL = abs( NEUTRAL_LL - LL_CLOSE_MAX )
		## neutral to HT_DOWN
		NEUTRAL_HT = HT_MIDDLE
		_delta_ticks_HT = abs( NEUTRAL_HT - HT_DOWN )
		## neutral to HP_LEFT (same delta as HP_RIGHT)
		NEUTRAL_HP = HP_FRONT
		_delta_ticks_HP = abs( NEUTRAL_HP - HP_LEFT )
		## neutral to ET_DOWN
		NEUTRAL_ET = ET_MIDDLE
		_delta_ticks_ET = abs( NEUTRAL_ET - ET_DOWN )
		## neutral to EP_LEFT (same delta as EP_RIGHT)
		NEUTRAL_EP = EP_FRONT
		_delta_ticks_EP = abs( NEUTRAL_EP - EP_LEFT )
		rospy.logerr("delta_ticks[LL, HT, HP, ET, EP] = " + str(_delta_ticks_LL)
			+ ", " + str(_delta_ticks_HT)
			+ ", " + str(_delta_ticks_HP)
			+ ", " + str(_delta_ticks_ET)
			+ ", " + str(_delta_ticks_EP) )

		## maki_cmd 
		_m_cmd_prefix_LL = "LL" + str(SC_SET_GP)
		_m_cmd_lid_neutral_close = _m_cmd_prefix_LL + str(LL_CLOSE_MAX) 
		_m_cmd_lid_close_neutral = _m_cmd_prefix_LL + str(NEUTRAL_LL) 
		_m_cmd_prefix_LL = "LL" + str(SC_SET_GS)

		_m_cmd_prefix_HT = "HT" + str(SC_SET_GP)
		_m_cmd_head_neutral_down = _m_cmd_prefix_HT + str(HT_DOWN) 
		_m_cmd_head_down_neutral = _m_cmd_prefix_HT + str(NEUTRAL_HT) 
		_m_cmd_prefix_HT = "HT" + str(SC_SET_GS)

		_m_cmd_prefix_HP = "HP" + str(SC_SET_GP)
		_m_cmd_head_neutral_left = _m_cmd_prefix_HP + str(HP_LEFT)
		_m_cmd_head_neutral_right = _m_cmd_prefix_HP + str(HP_RIGHT)
		_m_cmd_head_lr_neutral = _m_cmd_prefix_HP + str(NEUTRAL_HP)
		_m_cmd_prefix_HP = "HP" + str(SC_SET_GS)

		_m_cmd_prefix_ET = "ET" + str(SC_SET_GP)
		_m_cmd_eye_neutral_down = _m_cmd_prefix_ET + str(ET_DOWN) 
		_m_cmd_eye_down_neutral = _m_cmd_prefix_ET + str(NEUTRAL_ET) 
		_m_cmd_prefix_ET = "ET" + str(SC_SET_GS)

		_m_cmd_prefix_EP = "EP" + str(SC_SET_GP)
		_m_cmd_eye_neutral_left = _m_cmd_prefix_EP + str(EP_LEFT)
		_m_cmd_eye_neutral_right = _m_cmd_prefix_EP + str(EP_RIGHT)
		_m_cmd_eye_lr_neutral = _m_cmd_prefix_EP + str(NEUTRAL_EP)
		_m_cmd_prefix_HP = "EP" + str(SC_SET_GS)

		headTiltTimedTest.enableHT( self )
		_sww_wi.sleepWhileWaitingMS(1000, 0.05)
		rospy.loginfo("-----------------")
		for _ms_duration in range(5000, 1000, -500):	## [5000, 1500] ms
			headTiltTimedTest.enableHT( self )

			## calculate GoalSpeed
			_gs_HT = int( self.DC_helper.getGoalSpeed_ticks_durationMS(_delta_ticks_HT, _ms_duration) ) + 1
			_gs_HP = int( self.DC_helper.getGoalSpeed_ticks_durationMS(_delta_ticks_HP, _ms_duration) ) + 1
			## eyes should finish first
			_ms_duration_eyes = int( (float(_ms_duration) * 0.3) + 0.5 )
			_gs_LL = int( self.DC_helper.getGoalSpeed_ticks_durationMS(_delta_ticks_LL, _ms_duration_eyes) ) + 1
			_gs_ET = int( self.DC_helper.getGoalSpeed_ticks_durationMS(_delta_ticks_ET, _ms_duration_eyes) ) + 1
			_gs_EP = int( self.DC_helper.getGoalSpeed_ticks_durationMS(_delta_ticks_EP, _ms_duration_eyes) ) + 1
			rospy.logerr("ms_duration_eyes = " + str(_ms_duration_eyes) + "ms")

			## preset eyelid to neutral GoalSpeed
			#_pub_cmd = _m_cmd_prefix_LL + str(16)	## used spreadsheet to calculate these values
			#_pub_cmd += _m_cmd_prefix_HT + str(13)
			#_pub_cmd += _m_cmd_prefix_HP + str(88)
			#_pub_cmd += _m_cmd_prefix_EP + str(22)
			#_pub_cmd += _m_cmd_prefix_ET + str(50)
			#_pub_cmd += str(TERM_CHAR_SEND)
			#timedTest.pubTo_maki_command( self, str(_pub_cmd) )
			#_sww_wi.sleepWhileWaitingMS( 100, 0.05 )

			## move to asleep position in a discernable manner from waking up
			_pub_cmd = str(_m_cmd_lid_neutral_close + _m_cmd_head_neutral_down + _m_cmd_eye_neutral_down )
			_pub_cmd += str(_m_cmd_head_neutral_left + _m_cmd_eye_neutral_left)
			_pub_cmd += str(SC_SET_IPT) + str(waking_up_time)
			_pub_cmd += str(TERM_CHAR_SEND) 
			timedTest.pubTo_maki_command( self, str(_pub_cmd) )
			_sww_wi.sleepWhileWaitingMS( waking_up_time+1000, 0.05 )

			### preset eyelid to open GoalSpeed
			#_pub_cmd = _m_cmd_prefix_LL + str(_gs_LL) 
			#_pub_cmd += _m_cmd_prefix_HT + str(_gs_HT)
			#_pub_cmd += _m_cmd_prefix_HP + str(_gs_HP)
			#_pub_cmd += _m_cmd_prefix_ET + str(_gs_ET)
			#_pub_cmd += _m_cmd_prefix_EP + str(_gs_EP)
			#_pub_cmd += str(TERM_CHAR_SEND)
			#timedTest.pubTo_maki_command( self, str(_pub_cmd) )
			#_sww_wi.sleepWhileWaitingMS( 100, 0.05 )

			## publish waking up: eyelid open to neutral GoalPosition
			_pub_cmd = str(_m_cmd_lid_close_neutral + _m_cmd_head_down_neutral + _m_cmd_eye_down_neutral )
			_pub_cmd += str(_m_cmd_head_lr_neutral + _m_cmd_eye_lr_neutral)
			_pub_cmd += str(SC_SET_IPT) + str(_ms_duration)
			_pub_cmd += str(TERM_CHAR_SEND) 
			timedTest.pubTo_maki_command( self, str(_pub_cmd) )
			_start_awake_time = rospy.get_time()
			rospy.loginfo("Start waking up")
			_sww_wi.sleepWhileWaitingMS( _ms_duration, 0.01, False )
			_finish_awake_time = rospy.get_time()
			rospy.loginfo("Done waking up")

			_total_awake_time = abs( _finish_awake_time - _start_awake_time )
			awake_count += 1
			rospy.loginfo( "Awake #" + str(awake_count) 
				+ ": GS_LL = " + str(_gs_LL)
				+ ", GS_HT = " + str(_gs_HT)
				+ ", GS_HP = " + str(_gs_HP)
				+ ", GS_ET = " + str(_gs_ET)
				+ ", GS_EP = " + str(_gs_EP)
				+ "; expected time = " + str( _ms_duration )
				+ "ms; actual awake time = " + str( _total_awake_time ) 
				+ " seconds" )

			rospy.loginfo("-----------------")
			## make it easier to read
			_sww_wi.sleepWhileWaiting( read_time, 0.5 )

			## try to nicely end test
			if self.mTT_INTERRUPT:
				break	## break out of for loop
			else:
				## reset timers
				_start_awake_time = None
				_finish_awake_time = None
				_total_awake_time = 0
		# end	for _ms_duration

		return awake_count


