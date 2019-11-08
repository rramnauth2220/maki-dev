#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from timed_test import timedTest
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt


########################
## Test blink behavior
##
## Description: MAKI eyelid close and open
########################
class blinkTest( timedTest ):
	## variables private to this class
	## all instances of this class share the same value
	# none


	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		timedTest.__init__( self, verbose_debug, ros_pub )
		## add anything else needed by an instance of this subclass
		self.DC_helper = dynamixelConversions()


	#############################
	##	To run, publish to /maki_macro:
	##		reset
	##		blink_test
	##
	##	To stop, publish to /maki_macro:
	##		end
	##		reset
	#############################
	def macroBlink( self ):
		_sww_wi = ROS_sleepWhileWaiting_withInterrupt()

		## this is a nested while loop
		_print_once = True
		while self.ALIVE:
			if not rospy.is_shutdown():
				pass
			else:
				break	## break out of outer while loop
			if _print_once:
				rospy.logdebug("Entering macroBlink outer while loop")
				_print_once = False

			if self.mTT_INTERRUPT:	
				#print "start sleep 5"
				_sww_wi.sleepWhileWaiting(5)	# 5 seconds
				#print "end sleep 5"
				continue	## begin loop again from the beginning skipping below
				#print "shouldn't get here"

			rospy.logdebug("Entering macroBlink inner while loop")
			_blink_count = 0
			while not self.mTT_INTERRUPT:

				for _blink_rep in range (1,2):	#range(1,6):	## [1,6)
					#_blink_count = blinkTest.helper_macroBlink( self, _blink_count, True, _blink_rep, read_time=2.0 )
					#_blink_count = blinkTest.helper_macroBlink( self, _blink_count, False, _blink_rep, read_time=2.0 )
					#_blink_count = blinkTest.helper_macroBlink( self, _blink_count, True, 3, read_time=0.5 )
					_blink_count = blinkTest.helper_macroBlink( self, _blink_count, False, 3, read_time=0.5 )
					_sww_wi.sleepWhileWaiting(2)
					rospy.logdebug("***************")

					## try to nicely end testing the eye blink
					if self.mTT_INTERRUPT:
						_sww_wi.sleepWhileWaiting(1)	## make sure to wait for message to reach Dynamixel servo
						timedTest.pubTo_maki_command( self, "reset" )
						_sww_wi.sleepWhileWaiting(1)	## make sure to wait for message to reach Dynamixel servo
					else:
						pass
				# end	for _blink_rep in range()

			# end	while not self.mTT_INTERRUPT
		# end	while self.ALIVE 

	def helper_macroBlink( self, blink_count, full_blink, blink_rep, read_time=1.0 ):
		_sww_wi = ROS_sleepWhileWaiting_withInterrupt()
		#global blink_time		## list of ints

		_start_blink_time = None
		_start_blink_close_time = None
		_finish_blink_close_time = None
		_start_blink_open_time = None
		_finish_blink_open_time = None
		_finish_blink_time = None
		_total_blink_time = 0

		## maki_command prefix
		_m_cmd_prefix = "LL" + SC_SET_GP
	
		rospy.loginfo("blink_rep=" + str(blink_rep))
		rospy.loginfo("-----------------")
		for _blink_time in blink_time:
			_half_blink_time = int( float(_blink_time) * 0.5 + 0.5 )
			rospy.loginfo( "blink_time=" + str(_blink_time) + "; half_blink_time=" + str(_half_blink_time) )

			#_close_blink_time = int( float(_blink_time) * 0.35 + 0.5 )
			#_open_blink_time = _blink_time - _close_blink_time

			## maki_command suffix
			#_m_cmd_suffix = SC_SET_IPT + str( _half_blink_time ) + TERM_CHAR_SEND
			_m_cmd_suffix = TERM_CHAR_SEND
			_blink_close = _m_cmd_prefix
			if full_blink:
				_blink_close += str(LL_CLOSE_MAX) + _m_cmd_suffix
				blinkTest.presetGoalSpeed_ticks_durationMS( self, "LL", abs(LL_CLOSE_MAX - LL_OPEN_DEFAULT), _half_blink_time )
			else:
				_blink_close += str(LL_CLOSE_HALF) + _m_cmd_suffix
				blinkTest.presetGoalSpeed_ticks_durationMS( self, "LL", abs(LL_CLOSE_HALF - LL_OPEN_DEFAULT), _half_blink_time )
			_blink_open = _m_cmd_prefix + str(LL_OPEN_DEFAULT) + _m_cmd_suffix
		
			_start_blink_time = rospy.get_time()
			#for blink_rep in range(1, 6):
			for _flutter_count in range(0, blink_rep):
				rospy.logdebug( str(_flutter_count) )

				## blink close
				_start_blink_close_time = rospy.get_time()
				rospy.logdebug( "blink close" )
				timedTest.pubTo_maki_command( self, str(_blink_close) )
				_sww_wi.sleepWhileWaitingMS( _half_blink_time, 0.01 )
				#_sww_wi.sleepWhileWaitingMS( _close_blink_time, 0.01 )
				_finish_blink_close_time = rospy.get_time()

				## blink open
				_start_blink_open_time = rospy.get_time()
				rospy.logdebug( "blink open" )
				timedTest.pubTo_maki_command( self, str(_blink_open) )
				_sww_wi.sleepWhileWaitingMS( _half_blink_time, 0.01 )
				#_sww_wi.sleepWhileWaitingMS( _open_blink_time, 0.01 )
				_finish_blink_open_time = rospy.get_time()
				_finish_blink_time = rospy.get_time()

				if self.mTT_INTERRUPT:
					rospy.logerr("Abort blink_test")
					break	## break out of inner for loop
			# end	for _flutter_count in range(1, blink_rep):
			_finish_blink_time = rospy.get_time()

			## make it easier to read
			_sww_wi.sleepWhileWaiting( read_time, .5 )

			blink_count += 1
			_total_blink_time = abs(_finish_blink_time - _start_blink_time)
			#rospy.loginfo( "Completed " + str(blink_count) + " full eye blinks" )
			rospy.loginfo( "Eye blink #" + str(blink_count) + ": full eye blink = " 
				+ str( _total_blink_time )
				+ "; blink open->close = " + str( abs(_finish_blink_close_time - _start_blink_close_time) )
				+ "; blink close->open = " + str( abs(_finish_blink_open_time - _start_blink_open_time) ) )

			rospy.loginfo("-----------------")

			## try to nicely end testing the eye blink
			if self.mTT_INTERRUPT:
				break	## break out of outer for loop
			else:
				## reset timers
				_start_blink_time = None
				_start_blink_close_time = None
				_finish_blink_close_time = None
				_start_blink_open_time = None
				_finish_blink_open_time = None
				_finish_blink_time = None
				_total_blink_time = 0

		# end	for _blink_time in blink_time:
		return blink_count

	def presetGoalSpeed_ticks_duration( self, servo, ticks, s_duration ):
		_gs = int( self.DC_helper.getGoalSpeed_ticks_duration(ticks, s_duration) ) + 1
		if (_gs > 0) and (_gs < 1024):
			_pub_cmd = servo + str(SC_SET_GS) + str(_gs) + str(TERM_CHAR_SEND)
			rospy.loginfo( str(_pub_cmd) )
			timedTest.pubTo_maki_command( self, str(_pub_cmd) )
		elif (_gs == 0):
			rospy.logerr("setting GoalSpeed to 0 (unlimited) is not allowed!")
			rospy.logwarn("LL GoalSpeed is not changed")
		else:
			rospy.logerr("calculated GoalSpeed is out of bounds (0, 1023]; _gs = " + str(_gs))
	
	def presetGoalSpeed_ticks_durationMS( self, servo, ticks, ms_duration ):
		blinkTest.presetGoalSpeed_ticks_duration( self, servo, ticks, float( ms_duration / 1000.0 ) )
	
