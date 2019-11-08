#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
import os

import math
import string
import random
import thread

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from base_behavior import * ## baseBehavior, headTiltBaseBehavior, eyelidBaseBehavior
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt


########################
## Blink behavior
##
## SPONTANEOUS BLINKING
##	Definition: From en.wikipedia.org/wiki/Blinking, "spontaneous
##	blinking is done without external stimuli and internal effort.
##	This type of blinking is conducted in the pre-motor brain stem
##	and happens without conscious effort... Infants blink at an
##	average rate of 1 or 2 times in a minute. Throughout
##	childhood, the blink rate increases, and by adolescence, it is
##	usually equivalent to adults... Generally, between each blink is
##	an interval of 2 to 10 seconds (for adults); actual rates vary
##	by individual averaging about 10 blinks per minute." 
## 	Description: Maki-ro will periodically close its eyelids and re-open
##
## ENDOGENOUS EYE BLINKS
##	Definition: "When the eyes are focused on an object for an extended period
##	of time, such as when reading, the rate of blinking decreases
##	to about 3 or 4 times per minute," according to
##	en.wikipedia.org/wiki/Blinking. David L. Neumann and Ottmar V.
##	Lipp ("Spontaneous and reflexive eye activity measures of
##	mental workload") note that "people tend to blink more when the
##	eyes move from instrument to instrument because blinks tend to
##	punctuate the end of an episode of visual information intake."
##	Description: Maki-ro will periodically close its eyelids and re-open 
##	at a reduced rate to contrast the above spontaneous blinking.
##	Endogenous eye blinks should be used as Maki-ro begins to turn its
##	head to look at something, finishes watching something, etc.
##	Description: TODO and relevant functions
##
## REFLEXIVE BLINKING
##	Definition: According to Neumann and Lipp, "reflexive blinks
##	occur in response to the physical properties of a task-
##	irrelevant stimulus. The blink reflex can be elicited by a sudden
##	and moderately intense stimulus from a range of input modalities,
##	such as a loud noise, flash of light, tap of the forehead..." 
########################
class blinking( eyelidBaseBehavior ):
	## variables private to this class
	## all instances of this class share the same value
	__is_spontaneous_blinking = None


	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		eyelidBaseBehavior.__init__( self, verbose_debug, ros_pub )
		## add anything else needed by an instance of this subclass

		if self.makiPP == None:
			self.makiPP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )

		if blinking.__is_spontaneous_blinking == None:
			blinking.__is_spontaneous_blinking = False

		## variables relating to spontaneous blinking
		## 10 times per minute == 0.17 times per second == 0.17 Hz
		self.spontaneous_blink_rate_max = 10	
		## 2 times per minute ==  0.033 times per second == 0.033 Hz
		self.spontaneous_blink_rate_min = 3	#2	 

		## variables relating to endogenous eye blinks
		## 4 times per minute == 0.067 times per second == 0.067 Hz
		self.endogenous_blink_rate_max = 4	
		## 3 times per minute == 0.05 times per second == 0.05 Hz
		self.endogenous_blink_rate_min = 3
		self.endogenous_blink_repetitions = 2
		
		## variables relating to the duration of 1 eye blink
		## length of time for eyelid to close and re-open
		self.blink_action_duration = 350	#300	#250	#200	## milliseconds
		self.last_blink_time = None
		self.next_blink_time = None

		self.next_blink_pub = rospy.Publisher("blinking/next_blink", Int16, queue_size = 10)

		self.ALIVE = True
		return

	#############################
	##	To run, publish to /maki_macro:
	##		spontaneousBlink start
	##
	##	To stop, publish to /maki_macro:
	##		spontaneousBlink stop
	#############################
	def macroSpontaneousBlink( self ):
		rospy.logdebug("macroSpontaneousBlink: BEGIN")

		blinking.requestFeedback( self, SC_GET_PP )
		blinking.setEyelidNeutralPose( self, self.makiPP["LL"])
		## set ranges for spontaneous blinking
		blinking.setEyelidRange( self, self.makiPP["LL"], ll_close=LL_CLOSE_HALF)

		self.count_movements = 0
		_duration_to_closed = float( self.blink_action_duration ) * 0.5
		_duration = 0
		_start_time = rospy.get_time()
		self.next_blink_time = _start_time
		## this is a nested while loop
		while (blinking.__is_spontaneous_blinking and 
			self.ALIVE and not rospy.is_shutdown()):

			if self.mTT_INTERRUPT:	
				rospy.logdebug("mTT_INTERRUPT=" + str(self.mTT_INTERRUPT))
				break

			## debugging
			#rospy.logdebug("blinking.__is_spontaneous_blinking=" + str(blinking.__is_spontaneous_blinking))
			if not blinking.__is_spontaneous_blinking:
				rospy.logdebug("blinking.__is_spontaneous_blinking=" + str(blinking.__is_spontaneous_blinking))
				break

			_remaining_duration = self.next_blink_time - rospy.get_time()
			if (_remaining_duration > 0.01):	## 100 ms
				rospy.logdebug( str(_remaining_duration) + " seconds until next blink...")
				self.SWW_WI.sleepWhileWaiting( 0.1, increment=0.05, end_early=False )
				continue	## skip to the start of the while loop
			else:
				### FPPZ request published takes 100ms to propogate
				blinking.requestFeedback( self, SC_GET_PP )

			## Calculate goal speed
			_distance_to_closed = abs( self.makiPP["LL"] - self.ll_close )
			_gs_ll = self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_closed, _duration_to_closed ) 
			blinking.pubTo_maki_command( self, "LLGS" + str(_gs_ll) + str(TERM_CHAR_SEND))

			rospy.logdebug("SPONTANEOUS BLINK... eyelid close, GO!")
			_spontaneous_blink_start_time = rospy.get_time()
			## NOTE: cmd_prop must be set to True in eyelidClose
			## otherwise subsequent eyelidOpen won't open corectly
			## almost 50% of the time
			blinking.eyelidClose( self, cmd_prop=True )	
			rospy.logdebug("SPONTANEOUS BLINK... eyelid close, DONE! eyelid open, GO!")
			try:
				blinking.eyelidOpen( self, monitor=True )
			except rospy.exceptions.ROSException as e:
				rospy.logwarn("[WARNING] eyelids stalled when opening from blink...")
				blinking.eyelidOpen( self, monitor=False, cmd_prop=True )
			_spontaneous_blink_end_time = rospy.get_time()
			rospy.logdebug("SPONTANEOUS BLINK... eyelid open, DONE!")
			self.last_blink_time = rospy.get_time()
			rospy.loginfo("spontaneous blink duration: " + str( abs(_spontaneous_blink_end_time - _spontaneous_blink_start_time) ) + " seconds" )
			self.count_movements = self.count_movements +1

			_seconds_until_next_blink = random.uniform(self.spontaneous_blink_rate_min, self.spontaneous_blink_rate_max)
			self.next_blink_pub.publish( int(_seconds_until_next_blink) )
			self.next_blink_time = self.last_blink_time + _seconds_until_next_blink
			rospy.logdebug("Next spontaneous blink in " + str(_seconds_until_next_blink) + " seconds... at time " + str(self.next_blink_time))

			_duration = abs(rospy.get_time() - _start_time)
		#end	while self.ALIVE and not rospy.is_shutdown():

		self.next_blink_time = None
		self.next_blink_pub.publish( 0 )
		self.seconds_until_next_blink = None

		rospy.loginfo( "NUMBER OF SPONTANEOUS BLINKS: " + str(self.count_movements) )
		rospy.loginfo( "Duration: " + str(_duration) + " seconds" )

		rospy.logdebug("macroSpontaneousBlink: END")
		return

	def startSpontaneousBlink( self ):
		if blinking.__is_spontaneous_blinking:
			rospy.logwarn("[WARNING] already performing spontaneous blinking... do NOT start another")
			return
		else:
			rospy.sleep(0.25)

		rospy.logdebug("startSpontaneousBlink(): BEGIN")
		## call base class' start function
		eyelidBaseBehavior.start(self)
		blinking.__is_spontaneous_blinking = True
		rospy.logdebug("startSpontaneousBlink(): After eyelidBaseBehavior.start()")
		self.next_blink_time = rospy.get_time()
		rospy.logdebug("startSpontaneousBlink(): next_blink_time=" + str(self.next_blink_time))
		self.macroSpontaneousBlink()
		rospy.logdebug("startSpontaneousBlink(): END")
		return

	def stopSpontaneousBlink( self, auto_reset_eyelid=True ):
		rospy.logdebug("stopSpontaneousBlink(): BEGIN")
		blinking.__is_spontaneous_blinking = False

		## call base class' stop function
		eyelidBaseBehavior.stop(self)

		self.next_blink_time = None

		if auto_reset_eyelid:
			### set eyelid  
			#_pub_cmd = "LLGP" + str(self.origin_ll) + str(TERM_CHAR_SEND) 
			#blinking.monitorMoveToGP( self, _pub_cmd, ll_gp=self.origin_ll )
			blinking.setEyelidNeutralPose( self, self.origin_ll, monitor=True )

		rospy.logdebug("stopSpontaneousBlink(): END")
		return

	def reset( self ):
		blinking.setEyelidNeutralPose( self, LL_OPEN_DEFAULT, monitor=True )
		return

	def parse_maki_macro( self, msg ):
		#rospy.logdebug( msg.data )
		print msg.data

		if msg.data == "reset eyelids":
			blinking.reset( self )

		elif msg.data == "spontaneousBlink start":
			## This call is blocking
			#blinking.startSpontaneousBlink( self )
			try:
				thread.start_new_thread( blinking.startSpontaneousBlink, (self, ))
			except:
				rospy.logerr("Unable to start new thread for blinking.startSpontaneousBlink()")

		elif msg.data == "spontaneousBlink stop auto_reset_eyelid":
			blinking.stopSpontaneousBlink( self, auto_reset_eyelid=True )

		elif msg.data == "spontaneousBlink stop":
			blinking.stopSpontaneousBlink( self, auto_reset_eyelid=False )

		else:
			pass

		return

if __name__ == '__main__':
        print "__main__: BEGIN"
	blink = blinking( True, None )

	rospy.Subscriber( "/maki_macro", String, blink.parse_maki_macro )
        rospy.logdebug( "now subscribed to /maki_macro" )

	rospy.spin()   ## keeps python from exiting until this node is stopped

        print "__main__: END"


