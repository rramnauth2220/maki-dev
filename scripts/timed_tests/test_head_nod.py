#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string

import sys
sys.path.append('/home/scaz-baxter/catkin_ws/src/maki_robot-hirsch_dev/scripts')

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from timed_test import timedTest
#from head_tilt_timed_test import headTiltTimedTest
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt


########################
## Test head nodding behavior
##
## Description: from neutral, MAKI nods up (HT_UP)
##	and  then moves to HT_DOWN
##	TODO: Eye tilt (ET) and/or eyelid (LL) compensates for HT
########################
#class headNodTest( headTiltTimedTest ):
class headNodTest( timedTest ):
	## variables private to this class
	## all instances of this class share the same value
	# none


	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		headTiltTimedTest.__init__( self, verbose_debug, ros_pub )
		## add anything else needed by an instance of this subclass
		self.DC_helper = dynamixelConversions()


	###########################
	##
	##	To run, publish to /maki_macro
	##		reset
	##		nod_test
	##
	##	To stop, publish to /maki_macro
	##		end
	##		reset
	##
	###########################
	def macroHeadNod( self ):
		## different versions of head nodding
		_v1 = False
		_v2 = True

		_sww_wi = ROS_sleepWhileWaiting_withInterrupt( verbose_debug=self.VERBOSE_DEBUG )
		self.ALIVE = True


		rospy.logdebug("macroHeadNod: BEGIN")

		_start_headnod_time = None
		_start_headnod_up_time = None
		_start_headnod_down_time = None
		_finish_headnod_up_time = None
		_finish_headnod_down_time = None
		_finish_headnod_time = None

		_headnod_max_min_dist = float( abs( HT_UP - HT_DOWN ) )
		_headnod_middle_down_dist = float( abs( HT_MIDDLE - HT_DOWN ) )
		_headnod_middle_up_dist = float( abs( HT_MIDDLE - HT_UP ) )
		## debugging
		#print "_headnod_max_min_dist = " 
		#print _headnod_max_min_dist 
		#print "str(" + str(_headnod_max_min_dist) + ")"
		##
		#print "_headnod_middle_down_dist = " 
		#print _headnod_middle_down_dist 
		#print "str(" + str(_headnod_middle_down_dist) + ")"
		##
		#print "_headnod_middle_up_dist = " 
		#print _headnod_middle_up_dist 
		#print "str(" + str(_headnod_middle_up_dist) + ")"
		##
		#print _headnod_middle_down_dist / _headnod_max_min_dist
		#print _headnod_middle_up_dist / _headnod_max_min_dist
		_ipt_nod_middle_down = float( (_headnod_middle_down_dist / _headnod_max_min_dist) * IPT_NOD ) 
		_ipt_nod_middle_up = float( (_headnod_middle_up_dist / _headnod_max_min_dist) * IPT_NOD ) 
		_ipt_nod_middle_down = int(_ipt_nod_middle_down + 0.5)	## implicit rounding
		_ipt_nod_middle_up = int(_ipt_nod_middle_up + 0.5)	## implicit rounding
		rospy.logdebug( "(full)\tIPT_NOD = " + str(IPT_NOD) + "ms" )
		rospy.logdebug( "(partial)\t_ipt_nod_middle_down = " + str(_ipt_nod_middle_down) + "ms" )
		rospy.logdebug( "(partial)\t_ipt_nod_middle_up = " + str(_ipt_nod_middle_up) + "ms" )
		rospy.logdebug( "(summed partial)\t_ipt_nod_middle_* = " + str( _ipt_nod_middle_down + _ipt_nod_middle_up ) + "ms" )

		## maki_command prefix
		_m_cmd_prefix = "HT" + SC_SET_GP
		## nod macros
		_nod_up_down = _m_cmd_prefix + str(HT_DOWN) + SC_SET_IPT + str(IPT_NOD) + TERM_CHAR_SEND
		_nod_down_up = _m_cmd_prefix + str(HT_UP) + SC_SET_IPT + str(IPT_NOD) + TERM_CHAR_SEND
		_nod_middle_up = _m_cmd_prefix + str(HT_UP) + SC_SET_IPT + str(_ipt_nod_middle_up) + TERM_CHAR_SEND 
		_nod_middle_down = _m_cmd_prefix + str(HT_DOWN) + SC_SET_IPT + str(_ipt_nod_middle_down) + TERM_CHAR_SEND 
		_nod_up_middle = _m_cmd_prefix + str(HT_MIDDLE) + SC_SET_IPT + str(_ipt_nod_middle_up) + TERM_CHAR_SEND 
		_nod_down_middle = _m_cmd_prefix + str(HT_MIDDLE) + SC_SET_IPT + str(_ipt_nod_middle_down) + TERM_CHAR_SEND 

		## this is a nested while loop
		_print_once = True
		while self.ALIVE:
			if not rospy.is_shutdown():
				pass
			else:
				break	## break out of outer while loop

			if _print_once:
				rospy.logdebug("Entering macroHeadNod outer while loop")
				_print_once = False

			if self.mTT_INTERRUPT:	
				#print "start sleep 5"
				_sww_wi.sleepWhileWaiting(5)	# 5 seconds
				#print "end sleep 5"
				continue	## begin loop again from the beginning skipping below
				#print "shouldn't get here"

			## try to nicely startup headnod testing without jerking MAKI's head tilt servo
			headTiltTimedTest.enableHT( self )

			## where is MAKI's HT closest to currently? HT_UP, HT_MIDDLE, HT_DOWN
			_ht_pp = self.makiPP["HT"]
			_delta_ht_pp = abs( _ht_pp - HT_MIDDLE )
			_ht_macro_pose = HT_MIDDLE
			if ( abs( _ht_pp - HT_UP ) < _delta_ht_pp ):
				_delta_ht_pp = abs( _ht_pp - HT_UP )
				_ht_macro_pose = HT_UP
			if ( abs( _ht_pp - HT_DOWN ) < _delta_ht_pp ):
				_delta_ht_pp = abs( _ht_pp - HT_DOWN )
				_ht_macro_pose = HT_DOWN
			## publish GP of _ht_macro_pose in case in between poses
			timedTest.pubTo_maki_command( self, "HT" + SC_SET_GP + str(_ht_macro_pose) + TERM_CHAR_SEND )

			if (_ht_macro_pose == HT_MIDDLE):	
				## looking up first has strongest nodding cue
				timedTest.pubTo_maki_command( self, str(_nod_middle_up) )
				_sww_wi.sleepWhileWaitingMS( _ipt_nod_middle_up )

			rospy.logdebug("Entering macroHeadNod inner while loop")
			_nod_count = 0
			while not self.mTT_INTERRUPT:
				rospy.loginfo("-----------------")

				## VERSION 1: UP --> MIDDLE --> DOWN --> MIDDLE --> UP
				if _v1:
					_start_headnod_time = rospy.get_time()
					_start_headnod_down_time = rospy.get_time()
					timedTest.pubTo_maki_command( self, str(_nod_up_middle) )
					_sww_wi.sleepWhileWaitingMS( _ipt_nod_middle_up )
	
					timedTest.pubTo_maki_command( self, str(_nod_middle_down) )
					_sww_wi.sleepWhileWaitingMS( _ipt_nod_middle_down )

					timedTest.pubTo_maki_command( self, str(_nod_middle_down) )
					_sww_wi.sleepWhileWaitingMS( _ipt_nod_middle_down )
					_finish_headnod_down_time = rospy.get_time()

					_start_headnod_up_time = rospy.get_time()
					timedTest.pubTo_maki_command( self, str(_nod_down_middle) )
					_sww_wi.sleepWhileWaitingMS( _ipt_nod_middle_down )
		
					timedTest.pubTo_maki_command( self, str(_nod_middle_up) )
					_sww_wi.sleepWhileWaitingMS( _ipt_nod_middle_up )
					_finish_headnod_up_time = rospy.get_time()
					_finish_headnod_time = rospy.get_time()

				## VERSION 2: UP --> DOWN --> UP
				if _v2:
					_start_headnod_time = rospy.get_time()
					_start_headnod_down_time = rospy.get_time()
					timedTest.pubTo_maki_command( self, str(_nod_up_down) )
					_sww_wi.sleepWhileWaitingMS( IPT_NOD )
					_finish_headnod_down_time = rospy.get_time()

					_start_headnod_up_time = rospy.get_time()
					timedTest.pubTo_maki_command( self, str(_nod_down_up) )
					_sww_wi.sleepWhileWaitingMS( IPT_NOD )
					_finish_headnod_up_time = rospy.get_time()
					_finish_headnod_time = rospy.get_time()

				_nod_count += 1
				#rospy.loginfo( "Completed " + str(_nod_count) + " full head nods" )
				rospy.loginfo( "Head nod #" + str(_nod_count) + ": full nod = " 
					+ str( abs(_finish_headnod_time - _start_headnod_time) )
					+ "; nod up->down = " + str( abs(_finish_headnod_down_time - _start_headnod_down_time) )
					+ "; nod down->up = " + str( abs(_finish_headnod_up_time - _start_headnod_up_time) ) )
				rospy.loginfo("-----------------")


				## try to nicely end testing the headnod
				if self.mTT_INTERRUPT:
					_sww_wi.sleepWhileWaiting(1)	## make sure to wait for message to reach head tilt Dynamixel servo
					timedTest.pubTo_maki_command( self, "reset" )
					_sww_wi.sleepWhileWaiting(1)	## make sure to wait for message to reach head tilt Dynamixel servo
					headTiltTimedTest.disableHT( self )

			#end	while not self.mTT_INTERRUPT:
			headTiltTimedTest.disableHT( self )

		#end	while self.ALIVE:
		headTiltTimedTest.disableHT( self )

		rospy.logdebug("macroHeadNod: END")


