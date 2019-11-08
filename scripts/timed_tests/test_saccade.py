#! /usr/bin/env python

import rospy
from std_msgs.msg import String

import math
import string

import random


from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt

from timed_test import timedTest

########################
##  Test plausible velocity for eye saccade
########################
class saccadeTest(timedTest):
	## variables private to this class
	## all instances of this class share the same value
	# none

	def __init__( self, verbose_debug, ros_pub ):
		## invoke base class' __init__
		timedTest.__init__( self, verbose_debug, ros_pub )

	#######################
	## To run, publish to /maki_command
	##	reset
	##	saccade_test
	## To stop, publish to /maki_command
	##	reset
	#######################
	def macroEyeSaccade( self ):
		_sww_wi = ROS_sleepWhileWaiting_withInterrupt( verbose_debug=self.VERBOSE_DEBUG )

		## this is a nested while loop
		_print_once = True
		while self.ALIVE:
			if rospy.is_shutdown():
				break	## break out of macroEyeSaccade out while loop

			if _print_once:
				rospy.logdebug("Entering macroEyeSaccade outer while loop")
				_print_once = False

			if self.mTT_INTERRUPT:	
				#print "start sleep 5"
				_sww_wi.sleepWhileWaiting( 5 )	## 5 seconds
				#print "end sleep 5"
				continue	## begin loop again from the beginning skipping below
				#print "shouldn't get here"

			## where is MAKI's EP closest to currently? EP_LEFT, EP_FRONT, EP_RIGHT
			_ep_pp = self.makiPP["EP"]
			_delta_ep_pp = abs( _ep_pp - EP_FRONT )
			_ep_macro_pose = EP_FRONT
			if ( abs( _ep_pp - EP_LEFT ) < _delta_ep_pp ):
				_delta_ep_pp = abs( _ep_pp - EP_LEFT )
				_ep_macro_pose = EP_LEFT
			if ( abs( _ep_pp - EP_RIGHT ) < _delta_ep_pp ):
				_delta_ep_pp = abs( _ep_pp - EP_RIGHT )
				_ep_macro_pose = EP_RIGHT
			## publish GP of _ep_macro_pose in case in between poses
			timedTest.pubTo_maki_command( self, "EP" + SC_SET_GP + str(_ep_macro_pose) + TERM_CHAR_SEND )

			## raise LL so that it doesn't impede saccade
			timedTest.pubTo_maki_command( self, "LL" + SC_SET_GP + str(LL_OPEN_MAX) + TERM_CHAR_SEND )

			rospy.logdebug("Entering macroEyeSaccade inner while loop")
			_saccade_count = 0
			while not self.mTT_INTERRUPT:

				_saccade_count = saccadeTest.helper_macroEyeSaccade( self, _saccade_count )

				## try to nicely end testing the eye saccade
				if self.mTT_INTERRUPT:
					_sww_wi.sleepWhileWaiting( 1 )	## make sure to wait for message to reach Dynamixel servo
					timedTest.pubTo_maki_command( self, "reset" )
					_sww_wi.sleepWhileWaiting( 1 )	## make sure to wait for message to reach Dynamixel servo
				else:
					pass

			# end	while not self.mTT_INTERRUPT
		# end	while self.ALIVE
		rospy.loginfo("END: After outer while loop in macroEyeSaccade()")

	def helper_macroEyeSaccade( self, saccade_count, read_time=1.0 ):
		_sww_wi = ROS_sleepWhileWaiting_withInterrupt( verbose_debug=self.VERBOSE_DEBUG )

		_start_saccade_time = None
		_start_saccade_side_time = None
		_finish_saccade_side_time = None
		_start_saccade_front_time = None
		_finish_saccade_front_time = None
		_finish_saccade_time = None
		_total_saccade_time = 0
	
		## maki_command prefix
		_m_cmd_prefix = "EP" + SC_SET_GP

		rospy.loginfo("-----------------")
		for _eye_saccade_time in eye_saccade_time:
			## maki_command suffix
			_m_cmd_suffix = SC_SET_IPT + str( _eye_saccade_time ) + TERM_CHAR_SEND
			_saccade_front_right = _m_cmd_prefix + str(EP_RIGHT) + _m_cmd_suffix
			_saccade_front_left = _m_cmd_prefix + str(EP_LEFT) + _m_cmd_suffix
			_saccade_front = _m_cmd_prefix + str(EP_FRONT) + _m_cmd_suffix

			## used for choosing direction of eye saccade
			## 0 = left, 1 = right
			_rand_saccade_right = random.randint(0,1)

			_start_saccade_time = rospy.get_time()
			_start_saccade_side_time = rospy.get_time()
			if _rand_saccade_right == 1:
				rospy.logdebug( "saccade RIGHT" )
				timedTest.pubTo_maki_command( self, str(_saccade_front_right) )
			else:
				rospy.logdebug( "saccade LEFT" )
				timedTest.pubTo_maki_command( self, str(_saccade_front_left) )
			_sww_wi.sleepWhileWaitingMS( _eye_saccade_time, 0.01 )
			_finish_saccade_side_time = rospy.get_time()
			_finish_saccade_time = rospy.get_time()
			_total_saccade_time = abs(_finish_saccade_time - _start_saccade_time)

			## make it easier to read
			_sww_wi.sleepWhileWaiting( read_time, 0.5 )

			_start_saccade_time = rospy.get_time()
			## return to eye pan looking front
			_start_saccade_front_time = rospy.get_time()
			timedTest.pubTo_maki_command( self, str(_saccade_front) )
			_sww_wi.sleepWhileWaitingMS( _eye_saccade_time, 0.01 )
			_finish_saccade_front_time = rospy.get_time()
			_finish_saccade_time = rospy.get_time()
			_total_saccade_time += abs(_finish_saccade_time - _start_saccade_time)

			## make it easier to read
			_sww_wi.sleepWhileWaiting( read_time, .5 )

			saccade_count += 1
			#rospy.loginfo( "Completed " + str(saccade_count) + " full eye saccades" )
			rospy.loginfo( "Eye saccade #" + str(saccade_count) + ": full eye saccade = " 
				+ str( _total_saccade_time )
				+ "; eye saccade front->side = " + str( abs(_finish_saccade_side_time - _start_saccade_side_time) )
				+ "; eye saccade side->front = " + str( abs(_finish_saccade_front_time - _start_saccade_front_time) ) )
			rospy.loginfo("-----------------")

			## try to nicely end testing the eye saccade
			if self.mTT_INTERRUPT:
				break	## break out of for loop
			else:
				## reset timers
				_start_saccade_time = None
				_start_saccade_side_time = None
				_finish_saccade_side_time = None
				_start_saccade_front_time = None
				_finish_saccade_front_time = None
				_finish_saccade_time = None
				_total_saccade_time = 0

		# end	for _eye_saccade_time in eye_saccade_time:

		return saccade_count


