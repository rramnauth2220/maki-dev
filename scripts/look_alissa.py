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
## Maki-ro looks at Alissa behavior
##
## Description: from neutral, MAKI turns to its right
##	and turns head up to look at Alissa's face
########################
class lookAlissa( headTiltBaseBehavior ):
	## variables private to this class
	## all instances of this class share the same value
	# none

	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		headTiltBaseBehavior.__init__( self, verbose_debug, ros_pub )
		## add anything else needed by an instance of this subclass
		self.DC_helper = dynamixelConversions()


		self.sww_wi = ROS_sleepWhileWaiting_withInterrupt( verbose_debug=self.VERBOSE_DEBUG )
		if self.makiPP == None:
			self.makiPP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )

		self.ALIVE = True
		ipt_turn = 1000	##ms

		## look at and away macros
		#self.look_at = "HPGP" + str(HP_ALISSA) + "HTGP" + str(HT_ALISSA) + TERM_CHAR_SEND
		#self.look_neutral = "HPGP" + str(HP_FRONT) + "HTGP" + str(HT_MIDDLE) + TERM_CHAR_SEND
		#self.look_away = "HPGP" + str(HP_LEFT) + "HTGP" + str(HT_MIDDLE) + TERM_CHAR_SEND
		self.look_at = "HPGP" + str(HP_ALISSA) + "HTGP" + str(HT_ALISSA) + str(SC_SET_IPT) + str( ipt_turn) + TERM_CHAR_SEND
		self.look_neutral = "HPGP" + str(HP_FRONT) + "HTGP" + str(HT_MIDDLE) + str(SC_SET_IPT) + str( ipt_turn ) + TERM_CHAR_SEND
		self.look_away = "HPGP" + str(HP_LEFT) + "HTGP" + str(HT_MIDDLE) + str(SC_SET_IPT) + str( ipt_turn) + TERM_CHAR_SEND

	###########################
	##
	##	To run, publish to /maki_macro
	##		lookAtAlissa
	##
	###########################
	def macroLookAtAlissa( self ):
		rospy.logdebug("macrolookAtAlissa: BEGIN")
		print("====================HP_ALISSA VALUE: {}==================".format(HP_ALISSA))

		if self.ALIVE:
			if not rospy.is_shutdown():
				rospy.logdebug("NOT rospy.is_shutdown()")
				pass
			else:
				return

			if self.mTT_INTERRUPT:	
				rospy.logdebug("mTT_INTERRUPT=" + str(mTT_INTERRUPT))
				return

			if not self.mTT_INTERRUPT:
				rospy.loginfo("-----------------")

				baseBehavior.pubTo_maki_command( self, str(self.look_at) )
				self.sww_wi.sleepWhileWaitingMS( 2000 )

				rospy.loginfo("-----------------")

			#end	if not self.mTT_INTERRUPT:
		#end	if self.ALIVE:

		rospy.logdebug("macroLookAtAlissa: END")
		return

	###########################
	##
	##	To run, publish to /maki_macro
	##		lookAwayAlissa
	##
	###########################
	def macroLookAwayFromAlissa( self ):
		rospy.logdebug("macroLookAwayFromAlissa: BEGIN")

		if self.ALIVE:
			if not rospy.is_shutdown():
				rospy.logdebug("NOT rospy.is_shutdown()")
				pass
			else:
				return

			if self.mTT_INTERRUPT:	
				rospy.logdebug("mTT_INTERRUPT=" + str(mTT_INTERRUPT))
				return

			if not self.mTT_INTERRUPT:
				rospy.loginfo("-----------------")

				baseBehavior.pubTo_maki_command( self, str(self.look_away) )
				self.sww_wi.sleepWhileWaitingMS( 2000 )

				rospy.loginfo("-----------------")

			#end	if not self.mTT_INTERRUPT:
		#end	if self.ALIVE:

		rospy.logdebug("macroLookAwayFromAlissa: END")
		return

	###########################
	##
	##	To run, publish to /maki_macro
	##		lookNeutral
	##
	###########################
	def macroLookNeutral( self ):
		rospy.logdebug("macroNeutral: BEGIN")

		if self.ALIVE:
			if not rospy.is_shutdown():
				rospy.logdebug("NOT rospy.is_shutdown()")
				pass
			else:
				return

			if self.mTT_INTERRUPT:	
				rospy.logdebug("mTT_INTERRUPT=" + str(mTT_INTERRUPT))
				return

			if not self.mTT_INTERRUPT:
				rospy.loginfo("-----------------")

				baseBehavior.pubTo_maki_command( self, str(self.look_neutral) )
				self.sww_wi.sleepWhileWaitingMS( 2000 )

				rospy.loginfo("-----------------")

			#end	if not self.mTT_INTERRUPT:
		#end	if self.ALIVE:

		rospy.logdebug("macroLookNeutral: END")
		return

	def parse_maki_macro( self, msg ):
		print msg.data

		if msg.data == "lookAtAlissa":
			## try to nicely startup without jerking MAKI's head tilt servo
			headTiltBaseBehavior.start(self)

			self.macroLookAtAlissa()
			headTiltBaseBehavior.stop(self)

		elif msg.data == "lookAwayFromAlissa":
			## try to nicely startup without jerking MAKI's head tilt servo
			headTiltBaseBehavior.start(self)

			self.macroLookAwayFromAlissa()
			headTiltBaseBehavior.stop(self)

		elif msg.data == "lookNeutral":
			## try to nicely startup without jerking MAKI's head tilt servo
			headTiltBaseBehavior.start(self)

			self.macroLookNeutral()
			headTiltBaseBehavior.stop(self)

		else:
			pass

		return

if __name__ == '__main__':
        print "__main__: BEGIN"
	look = lookAlissa( True, None )

	rospy.Subscriber( "/maki_macro", String, look.parse_maki_macro )
        rospy.logdebug( "now subscribed to /maki_macro" )

	## TODO: subscribe to /maki_feedback_pres_pos
	## TODO: update self.makiPP from /maki_feedback_pres_pos

	rospy.spin()   ## keeps python from exiting until this node is stopped

        print "__main__: END"


