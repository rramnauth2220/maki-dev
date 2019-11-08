#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string
import random
import thread

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from base_behavior import * 	## classes baseBehavior and headTiltBaseBehavior
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt


########################
## Selective attention behaviors
##
## VISUAL SCANNING:
##	Definition: From "Encyclopedia of Child Behavior and Development"
##	by Benjamin K. Barton and Paul Jorritsma, pages 1546-1548.
##	Visual scanning refers to the pattern of fixations and saccades
##	while an individual is examiining visual stimuli. Scanning is
##	first rather disorganized in infants, then becomes more
##	exploratory but fairly limited, and finally develops into
##	controlled, goal-directed behavior. In older children and adults
##	visual scanning emerges as more sohpisticated visual search skills
##	and is part of a larger set of selective attention abilities.
##
##	Description: Maki-ro moves its eyes +/- delta around its neutral
##	eye pan and eye tilt positions
##
## TODO: 
##	* Eye tilt (ET) and/or eyelid (LL) compensates for HT
## 	* Gaussiaan distribution towards eye pan/tilt neutral
##	* Automatically turn head (head pan) based on  eye pan position ???
##	* in macroVisualScan, monitorMoveToGP and sleepWhileWaiting are blocking
##		so may miss "visualScan stop" message. Change to using time delta instead
##	* Add duration input
##
## MOTION CRITIQUE:
##
########################
class selectiveAttention( headTiltBaseBehavior ):
	## variables private to this class
	## all instances of this class share the same value
	__is_scanning = None


	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		headTiltBaseBehavior.__init__( self, verbose_debug, ros_pub )
		## add anything else needed by an instance of this subclass
		self.DC_helper = dynamixelConversions()

		if self.makiPP == None:
			self.makiPP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )

		## Does Maki-ro remain in end position (shift=True)
		## or revert to ground position (shift=False)
		self.shift = False	## default is False
		self.origin_ep = EP_FRONT		## default is neutral
		self.origin_et = ET_MIDDLE	## default is neutral

		## set random range for eye pan/tilt location centered around neutral
		## 30 ticks looks paranoid
		#self.ep_delta_range = 30	## ticks
		#self.et_delta_range = 30	## ticks	
		## 10 ticks looks like visual scanning on a focused point
		self.ep_delta_range = 10	## ticks
		self.et_delta_range = 10	#20	#15	#10	## ticks	
		## in addition to 100ms command propogation delay, provide option
		## additional rest period
		self.visual_scan_rest_occurence_percent = 35	#25	## [1,100)
		self.visual_scan_rest_enabled = True
		self.visual_scan_rest_min = 400 #200	#250	#100	#50	## milliseconds
		self.visual_scan_rest_max = 800	#600 #750	#400	#300	## milliseconds
		## NOTE: Scaz wanted longer rest durations between visual scan movements

		if selectiveAttention.__is_scanning == None:
			selectiveAttention.__is_scanning = False

		self.ALIVE = True
		return


	###########################
	##
	##	To run, publish to /maki_macro
	##		visualScan start
	##
	##	To stop, publish to /maki_macro
	##		visualScan stop
	##
	###########################
	def macroVisualScan( self, shift=False ):
		rospy.logdebug("macroVisualScan: BEGIN")

		## Does Maki-ro remain in end position (shift=True)
		## or revert to ground position (shift=False)
		self.shift = shift

		selectiveAttention.requestFeedback( self, SC_GET_PP )
		_previous_ep = self.makiPP["EP"]
		_previous_et = self.makiPP["ET"]
		if not shift:
			## when visualScan ends, revert to starting eye pan/tilt
			self.origin_ep = _previous_ep
			self.origin_et = _previous_et
		## set ranges for visual scanning
		selectiveAttention.setVisualScanRange( self, _previous_ep, _previous_et)

		_loop_count = 0
		_start_time = rospy.get_time()
		## this is a nested while loop
		while (selectiveAttention.__is_scanning and
			self.ALIVE and not rospy.is_shutdown()):

			if self.mTT_INTERRUPT:	
				rospy.logdebug("mTT_INTERRUPT=" + str(self.mTT_INTERRUPT))
				break	##  break out of while loop

			if not selectiveAttention.__is_scanning:
				rospy.logdebug("selectiveAttention.__is_scanning=" + str(selectiveAttention.__is_scanning))
				break

			### FPPZ request published every 100ms (10Hz)
			selectiveAttention.requestFeedback( self, SC_GET_PP )
			#selectiveAttention.requestFeedback( self, SC_GET_PP, time_ms=200 )

			## choose a random eye pan/tilt location within full range
			#self.et_rand = random.randint(ET_DOWN, ET_UP)
			#self.ep_rand = random.randint(EP_LEFT, EP_RIGHT)

			## TODO: Gaussiaan distribution towards eye pan/tilt neutral
			## random.randint is a uniform distribution
			self.et_rand = random.randint(self.et_rand_min, self.et_rand_max)
			self.ep_rand = random.randint(self.ep_rand_min, self.ep_rand_max)

			## TODO: Compensate eyelid position based on eye tilt
			_delta_et = self.et_rand - _previous_et
			_delta_ep = self.ep_rand - _previous_ep
			rospy.logdebug( "EPan = (" + str(_previous_ep) + ", " + str(self.ep_rand) + ", _delta_ep=" + str(_delta_ep) + ")")
			rospy.logdebug( "ETilt = (" + str(_previous_et) + ", " + str(self.et_rand) + ", _delta_et=" + str(_delta_et) + ")")

			_pub_cmd = "EPGP" + str(self.ep_rand) + "ETGP" + str(self.et_rand) + str(TERM_CHAR_SEND) 
			selectiveAttention.monitorMoveToGP( self, _pub_cmd, ep_gp=self.ep_rand, et_gp=self.et_rand )

			## in addition to 100ms command propogation delay, provide option
			## additional rest period
			if self.visual_scan_rest_enabled:
				_tmp_rest = random.randint(self.visual_scan_rest_min, self.visual_scan_rest_max)
				#rospy.logdebug("Extra rest is " + str(_tmp_rest) + " ms")
				rospy.logerr("Extra rest is " + str(_tmp_rest) + " ms")
				self.SWW_WI.sleepWhileWaitingMS( _tmp_rest, end_early=False)

			if (_loop_count % random.randint(5,10) == 0):
				_tmp_rest = random.randint(300, 800)
				rospy.logerr("_loop_count extra rest is " + str(_tmp_rest) + " ms")
				self.SWW_WI.sleepWhileWaitingMS( _tmp_rest, end_early=False)
				#_pub_cmd = "EPGP" + str(self.ep_rand) + "ETGP" + str(self.etrand) + str(TERM_CHAR_SEND) 
				#selectiveAttention.monitorMoveToGP( self, _pub_cmd, ep_gp=self.ep_rand, et_gp=self.etrand )

			if (random.randrange(1,100) < self.visual_scan_rest_occurence_percent):	
				self.visual_scan_rest_enabled=True
			else:
				self.visual_scan_rest_enabled=False

			## store previous values
			_previous_ep = self.makiPP["EP"]
			_previous_et = self.makiPP["ET"]
			if shift:
				## if shift==True, Maki-ro's eyes will remain in end position
				self.origin_ep = _previous_ep
				self.origin_et = _previous_et

			_loop_count = _loop_count +1
			_duration = abs(rospy.get_time() - _start_time)
		#end	while self.ALIVE and not rospy.is_shutdown():

		rospy.loginfo( "NUMBER OF VISUAL SCAN MOVMENTS: " + str(_loop_count) )
		rospy.loginfo( "Duration: " + str(_duration) + " seconds" )
		return

	def setVisualScanRange( self, ep, et, ep_delta=None, et_delta=None):
		if ep_delta == None:	ep_delta=self.ep_delta_range	
		if et_delta == None:	et_delta=self.et_delta_range	

		self.ep_rand_min = ep - ep_delta
		self.ep_rand_max = ep + ep_delta
		self.et_rand_min = et - et_delta
		self.et_rand_max = et + et_delta

		## check rand range values
		if self.ep_rand_min < EP_LEFT:	self.ep_rand_min = EP_LEFT
		if self.ep_rand_max > EP_RIGHT:	self.ep_rand_max = EP_RIGHT
		if self.et_rand_min < ET_DOWN:	self.et_rand_min = ET_DOWN
		if self.et_rand_max > ET_UP:	self.et_rand_max = ET_UP
		rospy.logdebug("EPan range: (" + str(self.ep_rand_min) + ", " + str(self.ep_rand_max) + ")")
		rospy.logdebug("ETilt range: (" + str(self.et_rand_min) + ", " + str(self.et_rand_max) + ")")
		return

	def setVisualScanTargetPose( self, ep, et, ipt=None ):
		self.origin_ep = ep
		self.origin_et = et

		_pub_cmd = "EPGP" + str(self.origin_ep) + "ETGP" + str(self.origin_et) 
		if (ipt != None and isinstance(ipt, int) and ipt > 0):
			_pub_cmd += SC_SET_IPT + str(ipt)
		_pub_cmd += str(TERM_CHAR_SEND) 
		selectiveAttention.monitorMoveToGP( self, _pub_cmd, ep_gp=self.origin_ep, et_gp=self.origin_et )

	def stopVisualScan( self, disable_ht=True ):
		selectiveAttention.__is_scanning = False

		## set eye pan and tilt  
		## if shift==False, Maki-ro's eyes will return to where they were when
		## visual scan started
		## if shift==True, Maki-ro's eyes will remain (not move)

		_pub_cmd = "EPGP" + str(self.origin_ep) + "ETGP" + str(self.origin_et) + str(TERM_CHAR_SEND) 
		selectiveAttention.monitorMoveToGP( self, _pub_cmd, ep_gp=self.origin_ep, et_gp=self.origin_et )
		headTiltBaseBehavior.stop( self, disable_ht=disable_ht )

	def parse_maki_macro( self, msg ):
		print msg.data

		if msg.data == "visualScan start":
			## Maki-ro doesn't need head tilt for visual scanning
			headTiltBaseBehavior.start(self, enable_ht=False)
			selectiveAttention.__is_scanning = True
			try:
				thread.start_new_thread( selectiveAttention.macroVisualScan, (self, ))
			except:
				rospy.logerr("Unable to start new thread for selectiveAttention.macroVisualScan()")
		elif msg.data.startswith( "visualScan stop" ):
			if msg.data.endswith( "disable_ht=False" ):
				selectiveAttention.stopVisualScan( self, disable_ht=False )
			else:
				selectiveAttention.stopVisualScan( self, disable_ht=True )

		elif msg.data == "reset selectiveAttention":
			selectiveAttention.setVisualScanTargetPose( self, EP_FRONT, ET_MIDDLE, ipt=300)

		else:
			pass

		return

if __name__ == '__main__':
        print "__main__: BEGIN"
	SA = selectiveAttention( True, None )

	rospy.Subscriber( "/maki_macro", String, SA.parse_maki_macro )
        rospy.logdebug( "now subscribed to /maki_macro" )

	rospy.spin()   ## keeps python from exiting until this node is stopped

        print "__main__: END"


