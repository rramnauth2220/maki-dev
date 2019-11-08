#! /usr/bin/env python

## TODO: update with changes made to baseBehavior

import rospy
from std_msgs.msg import String
import os

import math
import sys
import string

import re		# see http://stackoverflow.com/questions/5749195/how-can-i-split-and-parse-a-string-in-python


from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions

from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt


########################
## Specific behavior tests will use this as base class
########################
class timedTest(object):
	## all instances of this class share the same value
	## variables private to this class
	__maki_msg_format = None


	def __init__(self, verbose_debug, ros_pub):
		self.ALIVE = True
		self.mTT_INTERRUPT = True
		self.VERBOSE_DEBUG = verbose_debug	## default is False
		self.SWW_WI = ROS_sleepWhileWaiting_withInterrupt()
		if ros_pub == None:
			self.initROS( self )
		else:
			self.ros_pub = ros_pub		## can we pass a ros publisher??? Apparently so!
		if timedTest.__maki_msg_format == None:
			timedTest.initPubMAKIFormat( self )
		#rospy.logdebug( str(timedTest.__maki_msg_format) )
		self.makiPP = None

	def startTimedTest(self, makiPP):
		self.ALIVE = True
		self.mTT_INTERRUPT = False
		self.makiPP = makiPP

	def stopTimedTest(self):
		self.mTT_INTERRUPT = True

	def exitTimedTest(self):
		self.ALIVE = False
		self.mTT_INTERRUPT = True

	def update( self, makiPP ):
		self.makiPP = makiPP

	###############################
	## Example contents of timed test
	###############################
	def macroEmptyTest(self):
		## this is a nested while loop
		_print_once = True
		#while self.ALIVE and not rospy.is_shutdown():
		while self.ALIVE:
			if not rospy.is_shutdown():
				pass
			else:
				break	## break out of outer while loop

			if _print_once:
				rospy.logdebug("Entering macroEmptyTest outer while loop")
				_print_once = False

			if self.mTT_INTERRUPT:	
				print "start sleep 5"
				self.SWW_WI.sleepWhileWaiting( 5 )
				print "end sleep 5"
				continue	## begin loop again from the beginning skipping below
				#print "shouldn't get here"


			rospy.logdebug("Entering macroEmptyTest inner while loop")
			_pass_count = 0
			while not self.mTT_INTERRUPT:

				_pass_count = timedTest.helper_macroEmptyTest( self, _pass_count )

				## try to nicely end testing 
				if self.mTT_INTERRUPT:
					self.SWW_WI.sleepWhileWaiting( 1 )	## make sure to wait for message to reach Dynamixel servo
					timedTest.pubTo_maki_command( self, "reset" )
					self.SWW_WI.sleepWhileWaiting( 1 )	## make sure to wait for message to reach Dynamixel servo
				else:
					pass

			# end	while not mTT_INTERRUPT
		# end	while self.ALIVE and not rospy.is_shutdown():
		rospy.loginfo("END: After outer while loop in macroEmptyTest()")

	def helper_macroEmptyTest( self, pass_count, read_time=1.0 ):
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
			self.SWW_WI.sleepWhileWaitingMS( (_pass_time+1)*1000, 0.01 )
			_finish_pass_time = rospy.get_time()
			_total_pass_time = abs(_finish_pass_time - _start_pass_time)

			pass_count += 1
			#rospy.loginfo( "Completed " + str(pass_count) + " passes" )
			rospy.loginfo( "Pass #" + str(pass_count) + ": full pass time = " 
				+ str( _total_pass_time ) )

			rospy.loginfo("-----------------")
			## make it easier to read
			self.SWW_WI.sleepWhileWaiting( read_time, 0.5 )

			## try to nicely end test
			if self.mTT_INTERRUPT:
				break	## break out of for loop
			else:
				## reset timers
				_start_pass_time = None
				_finish_pass_time = None
				_total_pass_time = 0
		# end	for _pass_time in range(0,3):

		return pass_count

	#####################
	## THESE ARE COMMON FOR ALL TESTS
	#####################
	def pubTo_maki_command( self, commandOut ):
		_pub_flag = False

		## make sure that commandOut ends in only one TERM_CHAR_SEND
		_tmp = re.search( timedTest.__maki_msg_format, commandOut )
		if _tmp != None:
			## Yes, commandOut ends in only one TERM_CHAR_SEND
			_pub_flag = True
			#if self.VERBOSE_DEBUG:       rospy.logdebug( str(commandOut) + " matched maki_msg_format" )
		elif (commandOut == "reset"):
			## special case handled by MAKI-Arbotix-Interface.py driver
			_pub_flag = True
		elif not commandOut.endswith( str(TERM_CHAR_SEND) ):
			## append the missing TERM_CHAR_SEND
			commandOut += str(TERM_CHAR_SEND)
			_pub_flag = True
			if self.VERBOSE_DEBUG:       rospy.logdebug( str(commandOut) + " added TERM_CHAR_SEND" )
		else:
			rospy.logerr( "Incorrect message format" + str(commandOut) )

		if self.VERBOSE_DEBUG: rospy.logdebug( str(commandOut) )

		if _pub_flag and not rospy.is_shutdown():
			self.ros_pub.publish( commandOut )


	#####################
	## Initialize ROS node 
	#####################
	def initROS( self, nodename="anon" ):
		## get function name for logging purposes
		_fname = sys._getframe().f_code.co_name        ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
		print str(_fname) + ": BEGIN"	## THIS IS BEFORE ROSNODE INIT

		_anon_rosnode = False
		if nodename == "anon":	_anon_rosnode = True

        	# see http://wiki.ros.org/rospy/Overview/Logging
        	if self.VERBOSE_DEBUG:
        	        rospy.init_node(nodename, anonymous=_anon_rosnode, log_level=rospy.DEBUG)
			rospy.logdebug("log_level=rospy.DEBUG")
        	else:
        	        rospy.init_node(nodename, anonymous=_anon_rosnode)       ## defaults to log_level=rospy.INFO
		rospy.logdebug("anonymous=" + str(_anon_rosnode))

		rospy.loginfo( str(_fname) + ": END")
		return

	#####################
	## Set up publisher to /maki_command
	#####################
	def initPubMAKI(self):
		## get function name for logging purposes
		_fname = sys._getframe().f_code.co_name        ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
		rospy.logdebug( str(_fname) + ": BEGIN")

		# Setup publisher
		self.ros_pub = rospy.Publisher("maki_command", String, queue_size = 10)

		rospy.loginfo( str(_fname) + ": END")
		return

	#####################
	## Set up regex format for publishing on /maki_command
	#####################
	def initPubMAKIFormat(self):
		## get function name for logging purposes
		_fname = sys._getframe().f_code.co_name        ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
		rospy.logdebug( str(_fname) + ": BEGIN")

		## make sure that commandOut ends in only one TERM_CHAR_SEND
		#self.__maki_msg_format = "\A[a-yA-Y]+[a-yA-Y0-9]*"
		#self.__maki_msg_format += str(TERM_CHAR_SEND)
		#self.__maki_msg_format += "{1}$"
		timedTest.__maki_msg_format = "\A[a-yA-Y]+[a-yA-Y0-9]*"
		timedTest.__maki_msg_format += str(TERM_CHAR_SEND)
		timedTest.__maki_msg_format += "{1}$"

		rospy.loginfo( str(_fname) + ": END")
		return

