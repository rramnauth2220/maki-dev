#! /usr/bin/env python

## TODO: update with changes made to headTiltBaseBehavior

import rospy
from std_msgs.msg import String
import os

import math
import sys
import string

import re		# see http://stackoverflow.com/questions/5749195/how-can-i-split-and-parse-a-string-in-python


from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from timed_test import timedTest
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt


########################
## Specific behavior tests involving head tilt (HT) will use this as base class
########################
class headTiltTimedTest(timedTest):
	## all instances of this class share the same value
	## variables private to this class
	__ht_enabled = None
	__ht_enable_cmd = None
	__ht_disable_cmd = None


	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		timedTest.__init__( self, verbose_debug, ros_pub )
		if headTiltTimedTest.__ht_enabled == None:
			headTiltTimedTest.__ht_enabled = False
		if headTiltTimedTest.__ht_enable_cmd == None:
			headTiltTimedTest.__ht_enable_cmd = "HT" + str(SC_SET_TL) + str(ht_tl_enable) + str(TERM_CHAR_SEND)
		if headTiltTimedTest.__ht_disable_cmd == None:
			headTiltTimedTest.__ht_disable_cmd = "HT" + str(SC_SET_TL) + str(ht_tl_disable) + str(TERM_CHAR_SEND)

	def enableHT( self ):
		if headTiltTimedTest.__ht_enabled == True:
			## already enabled
			return

		if (self.makiPP["HT"] <= HT_UP and self.makiPP["HT"] >= HT_DOWN):
			_pub_cmd_GP = "HT" + str(SC_SET_GP) + str(self.makiPP["HT"]) + str(TERM_CHAR_SEND)
			timedTest.pubTo_maki_command( self, str(_pub_cmd_GP) )
			self.SWW_WI.sleepWhileWaitingMS( 100, 0.05 )	## make sure command propogates
		
		timedTest.pubTo_maki_command( self, str(headTiltTimedTest.__ht_enable_cmd) )
		self.SWW_WI.sleepWhileWaitingMS( 100, 0.05 )	## make sure command propogates
		headTiltTimedTest.__ht_enabled = True

	def disableHT( self ):
		if headTiltTimedTest.__ht_enabled == False:
			## already disabled
			return

		timedTest.pubTo_maki_command( self, str(headTiltTimedTest.__ht_disable_cmd) )
		self.SWW_WI.sleepWhileWaitingMS( 100, 0.05 )	## make sure command propogates
		headTiltTimedTest.__ht_enabled = False

	#def startTimedTest(self, makiPP):
	#	self.mTT_INTERRUPT = False
	#	self.makiPP = makiPP

	#def stopTimedTest(self):
	#	self.mTT_INTERRUPT = True

	#def exitTimedTest(self):
	#	self.ALIVE = False
	#	self.mTT_INTERRUPT = True

	#def update( self, makiPP ):
	#	self.makiPP = makiPP




