#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from base_behavior import * 	## classes baseBehavior and headTiltBaseBehavior
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt


########################
## Head shake behavior
##
## Description: from neutral, MAKI nods left (HP_LEFT)
##	and  then moves to HP_RIGHT
##
## TODO: 
##	* Eye tilt (ET) and/or eyelid (LL) compensates for HT
##	* Change from static definitions of HP_LEFT, HP_RIGHT. 
##	* Use dynamixelConversions instead of IPT
##	* Access repetion variable
##	* Add style variable (e.g, default, full range, subtle/backchannel
##
## MOTION CRITIQUE:
##	v3 motion still has slight pause when transitioning from arriving at
##	HP_LEFT before going down
########################
class headShake( headTiltBaseBehavior ):
	## variables private to this class
	## all instances of this class share the same value
	v1 = None
	v2 = None
	v3 = None

	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		headTiltBaseBehavior.__init__( self, verbose_debug, ros_pub )
		## add anything else needed by an instance of this subclass
		self.DC_helper = dynamixelConversions()

		## different versions of head nodding
		headShake.v1 = False ## WIDE SHAKE
		headShake.v2 = True ## RIGHT --> LEFTMID --> MID
		headShake.v3 = False ## RIGHT --> MID
		headShake.v4 = False ## ??LEFT --> MID

		self.repetition = 1

		self.sww_wi = ROS_sleepWhileWaiting_withInterrupt( verbose_debug=self.VERBOSE_DEBUG )
		if self.makiPP == None:
			self.makiPP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )

		self.ALIVE = True

		self.headshake_max_min_dist = float( abs( HP_LEFT - HP_RIGHT ) )
		self.headshake_front_right_dist = float( abs( HP_FRONT - HP_RIGHT ) )
		self.headshake_front_left_dist = float( abs( HP_FRONT - HP_LEFT ) )
		## debugging
		#print "headshake_max_min_dist = " 
		#print self.headshake_max_min_dist 
		#print "str(" + str(self.headshake_max_min_dist) + ")"
		##
		#print "headshake_front_right_dist = " 
		#print self.headshake_front_right_dist 
		#print "str(" + str(self.headshake_front_right_dist) + ")"
		##
		#print "headshake_front_left_dist = " 
		#print self.headshake_front_left_dist 
		#print "str(" + str(self.headshake_front_left_dist) + ")"
		##
		#print self.headshake_front_right_dist / self.headshake_max_min_dist
		#print self.headshake_front_left_dist / self.headshake_max_min_dist
		self.ipt_shake_front_right = float( (self.headshake_front_right_dist / self.headshake_max_min_dist) * IPT_FACE ) 
		self.ipt_shake_front_left = float( (self.headshake_front_left_dist / self.headshake_max_min_dist) * IPT_FACE ) 
		self.ipt_shake_front_right = int(self.ipt_shake_front_right + 0.5)	## implicit rounding
		self.ipt_shake_front_left = int(self.ipt_shake_front_left + 0.5)	## implicit rounding
		## debugging
		# rospy.logdebug( "(full)\tIPT_NOD = " + str(IPT_FACE) + "ms" )
		# rospy.logdebug( "(partial)\t ipt_shake_front_right = " + str(self.ipt_shake_front_right) + "ms" )
		# rospy.logdebug( "(partial)\t ipt_shake_front_left = " + str(self.ipt_shake_front_left) + "ms" )
		# rospy.logdebug( "(summed partial)\t ipt_nod_middle_* = " + str( self.ipt_shake_front_right + self.ipt_shake_front_left ) + "ms" )

		## maki_command prefix
		_m_cmd_prefix = "HP" + SC_SET_GP
		## shake macros
		self.shake_left_right = _m_cmd_prefix + str(HP_RIGHT) + SC_SET_IPT + str(IPT_FACE) + TERM_CHAR_SEND
		self.shake_right_left = _m_cmd_prefix + str(HP_LEFT) + SC_SET_IPT + str(IPT_FACE) + TERM_CHAR_SEND
		self.shake_front_left = _m_cmd_prefix + str(HP_LEFT) + SC_SET_IPT + str(self.ipt_shake_front_left) + TERM_CHAR_SEND 
		self.shake_front_right = _m_cmd_prefix + str(HP_RIGHT) + SC_SET_IPT + str(self.ipt_shake_front_right) + TERM_CHAR_SEND 
		self.shake_left_front = _m_cmd_prefix + str(HP_FRONT) + SC_SET_IPT + str(self.ipt_shake_front_left) + TERM_CHAR_SEND 
		self.shake_right_front = _m_cmd_prefix + str(HP_FRONT) + SC_SET_IPT + str(self.ipt_shake_front_right) + TERM_CHAR_SEND 


	###########################
	##
	##	To run, publish to /maki_macro
	##		shake
	##
	###########################
	def macroHeadShake( self ):
		rospy.logdebug("macroHeadShake: BEGIN")

		## this is a nested while loop
		#_print_once = True
		#while self.ALIVE:
		if self.ALIVE:
			if not rospy.is_shutdown():
				rospy.logdebug("NOT rospy.is_shutdown()")
				pass
			else:
				#break	## break out of outer while loop
				return

			#if _print_once:
			#	rospy.logdebug("Entering macroHeadShake outer while loop")
			#	_print_once = False

			if self.mTT_INTERRUPT:	
				rospy.logdebug("mTT_INTERRUPT=" + str(mTT_INTERRUPT))
				##print "start sleep 5"
				#self.sww_wi.sleepWhileWaiting(5)	# 5 seconds
				##print "end sleep 5"
				#continue	## begin loop again from the beginning skipping below
				##print "shouldn't get here"
				return

			## ----- COMMENTED OUT ------
			## NOTE: Maki-ro should be able to shake regardless of initial HT position

			## where is MAKI's HT closest to currently? HP_LEFT, HP_FRONT, HP_RIGHT
			#_ht_pp = self.makiPP["HT"]
			#_delta_ht_pp = abs( _ht_pp - HP_FRONT )
			#_delta_ht_pp = DELTA_PP
			#_ht_macro_pose = HP_FRONT
			#if ( abs( _ht_pp - HP_LEFT ) < _delta_ht_pp ):
			#	_delta_ht_pp = abs( _ht_pp - HP_LEFT )
			#	_ht_macro_pose = HP_LEFT
			#if ( abs( _ht_pp - HP_RIGHT ) < _delta_ht_pp ):
			#	_delta_ht_pp = abs( _ht_pp - HP_RIGHT )
			#	_ht_macro_pose = HP_RIGHT
			#rospy.logdebug( "_ht_macro_pose = " + str(_ht_macro_pose) )
			## publish GP of _ht_macro_pose in case in between poses
			#baseBehavior.pubTo_maki_command( self, "HT" + SC_SET_GP + str(_ht_macro_pose) + TERM_CHAR_SEND, cmd_prop=False )
			#
			#if (_ht_macro_pose == HP_FRONT):	
			#	## START NOD FROM HEAD UP POSITION
			#	## looking up first has strongest nodding cue
			#	while ( abs( self.makiPP["HT"] - HP_LEFT) > _delta_ht_pp ):
			#		baseBehavior.pubTo_maki_command( self, str(self.shake_front_left), time_ms=self.ipt_shake_front_left, time_inc=0.005 )
					#baseBehavior.pubTo_maki_command( self, str(self.shake_front_left), time_ms=IPT_FACE, time_inc=0.005 )

			## START NOD FROM HEAD UP POSITION
			## looking up first has strongest nodding cue
			baseBehavior.monitorMoveToGP( self, str(self.shake_front_left), hp_gp=HP_LEFT, delta_pp=3.5*DELTA_PP )

			rospy.logdebug("Entering macroHeadShake inner while loop")
			_loop_count = 0
			while (not rospy.is_shutdown()) and (not self.mTT_INTERRUPT) and (_loop_count < self.repetition):
				_loop_count = _loop_count + 1
				rospy.loginfo("-------" + str(_loop_count) + " start ----------")

				## VERSION 1: FROM UP --> MIDDLE --> DOWN --> 
				if headShake.v1:
					baseBehavior.monitorMoveToGP( self, str(self.shake_front_left), hp_gp=HP_LEFT, delta_pp=3.5*DELTA_PP )

					baseBehavior.pubTo_maki_command( self, str(self.shake_left_front), time_ms=self.ipt_shake_front_left, time_inc=0.001 )
	
					baseBehavior.pubTo_maki_command( self, str(self.shake_front_right), time_ms=self.ipt_shake_front_right, time_inc=0.001 )

					baseBehavior.pubTo_maki_command( self, str(self.shake_front_right), time_ms=self.ipt_shake_front_right, time_inc=0.001 )

		

				## VERSION 2: UP --> DOWN --> 
				if headShake.v2:
					baseBehavior.monitorMoveToGP( self, str(self.shake_front_left), hp_gp=HP_LEFT, delta_pp=3.5*DELTA_PP )

					baseBehavior.pubTo_maki_command( self, str(self.shake_right_left), time_ms=self.ipt_shake_front_left )

					baseBehavior.pubTo_maki_command( self, str(self.shake_left_right), time_ms=IPT_FACE )


				## VERSION 3: FROM UP --> DOWN --> 
				if headShake.v3:
					baseBehavior.monitorMoveToGP( self, str(self.shake_front_left), hp_gp=HP_LEFT, delta_pp=3.5*DELTA_PP )

					baseBehavior.monitorMoveToGP( self, str(self.shake_left_right), ht_gp=HP_RIGHT, delta_pp=3.5*DELTA_PP )

				## ALL: DOWN --> UP -->
				if (headShake.v1 or headShake.v2 or headShake.v3) and (_loop_count < self.repetition):
					#baseBehavior.pubTo_maki_command( self, str(self.shake_right_left), time_ms=IPT_FACE )
					baseBehavior.monitorMoveToGP( self, str(self.shake_right_left), ht_gp=HP_LEFT, delta_pp=3.5*DELTA_PP )

				## VERSION 3: FROM UP --> DOWN --> 
				if headShake.v4:
										
					baseBehavior.monitorMoveToGP( self, str(self.shake_left_right), ht_gp=HP_RIGHT, delta_pp=3.5*DELTA_PP )

			#end	while not self.mTT_INTERRUPT:

			## ALL: DOWN --> MIDDLE
			### time_ms = IPT_FACE needed to allow head tilt to recenter to middle
			#baseBehavior.pubTo_maki_command( self, str(self.shake_right_front), time_ms=IPT_FACE )
			#while (not rospy.is_shutdown()) and ( abs( self.makiPP["HT"] - HP_FRONT) > _delta_ht_pp ):
			#	baseBehavior.pubTo_maki_command( self, str(self.shake_right_front), time_ms=self.ipt_shake_front_left, time_inc=0.005 )
			baseBehavior.monitorMoveToGP( self, str(self.shake_right_front), hp_gp=HP_FRONT )

			rospy.loginfo("-----------------")
		#end	if self.ALIVE:

		rospy.logdebug("macroHeadShake: END")
		return

	def parse_maki_macro( self, msg ):
		print msg.data

		if msg.data == "shake full":
			### try to nicely startup headshake testing without jerking MAKI's head tilt servo
			headTiltBaseBehavior.start(self)
			self.macroHeadShake()
			headTiltBaseBehavior.stop(self)
		return

if __name__ == '__main__':
        print "__main__: BEGIN"
	shake = headShake( True, None )

	rospy.Subscriber( "/maki_macro", String, shake.parse_maki_macro )
        rospy.logdebug( "now subscribed to /maki_macro" )

	rospy.spin()   ## keeps python from exiting until this node is stopped

        print "__main__: END"


