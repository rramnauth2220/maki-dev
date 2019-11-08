#! /usr/bin/env python

import rospy

import math

## NOTE: Changed to using rospy.Time to make compatible with Clock service for live and simulated time
#from time import sleep
#from timeit import default_timer as timer	## wall clock. Unix 1/100 second granularity
## NOTE: rospy.get_time() reports the same as timer() when running live

from maki_robot_common import *


class ROS_sleepWhileWaiting_withInterrupt:
	def __init__( self, verbose_debug=False ):
		self.PIC_INTERRUPT = True
		self.VERBOSE_DEBUG = verbose_debug
		self.step_resolution = 0.001	## nanosecond resolution	## previous values: 1.0, 0.25, 0.01 seconds

	def abort( self ):
		self.PIC_INTERRUPT = True


	#############################
	## augmented version of sleep in milliseconds
	##
	## Typical usage: called as sleepWhileWaiting( 150, 0.05 ), which will sleep 
	## slightly less than 150 milliseconds. Check every 0.05 seconds (50 ms)
	## (default increment is 0.25 s, or 250 ms) to see if more than 
	## 100 milliseconds have elapsed. end_early flag is True by default, so 
	## 150 milliseconds is decreased by 1 increment value (150 - 50).
	##
	## Options: 
	##	* specify increment (default is 0.25 second, 250 ms)
	## 	* specify if sleep should end 1 increment early (default is True)
	#############################
	def sleepWhileWaitingMS( self, ms_sleep_time, increment=0.25, end_early=True):
		## convert from milliseconds to sleep in seconds
		## increment is still in seconds since calls sleepWhileWaiting

		_new_ms_sleep_time = float(ms_sleep_time)/1000.0 
		#if end_early:
		#	_new_ms_sleep_time = _new_ms_sleep_time - float(increment)
	
		#print "ms_sleep_time = " + str( ms_sleep_time )
		#print "increment = " + str( increment )
		#print "new_ns_sleep_time = " + str( _new_ms_sleep_time )
		ROS_sleepWhileWaiting_withInterrupt.sleepWhileWaiting( self, _new_ms_sleep_time, increment, end_early )


	#############################
	## augmented version of sleep in seconds
	##
	## Typical usage: called as sleepWhileWaiting( 2 ), which will sleep 2 seconds
	## check every 1 second (default increment) to see if more than 2 seconds
	## have elapsed
	##
	## sleep_time can be int or float
	##
	## Options: 
	##	* specify increment in seconds as int or float (default is 1 second)
	## 	* specify if sleep should end 1 increment early (default is False)
	#############################
	def sleepWhileWaiting( self, sleep_time, increment=1, end_early=False ):
		self.PIC_INTERRUPT = False	## reset

		if self.VERBOSE_DEBUG: rospy.logdebug( "BEGIN: sleepWhileWaiting for " + str(sleep_time) + " seconds" )

		## ---- PREP, ERROR CHECKING, and ADJUSTMENT ----
		## exit if sleep_time = 0
		if (sleep_time == 0):
			rospy.logdebug( "DONE: sleepWhileWaiting for " + str(sleep_time) + " seconds" )
			return

		## auto-adjust increment value
		_orig_increment = increment
		_adjust_start_time = rospy.get_time()
		if (sleep_time < increment):
			increment = sleep_time
			rospy.logdebug("sleepWhileWaiting: **** sleep_time is less than increment; increment adjusted from " + str(_orig_increment) + " seconds to " + str(increment))
		if end_early:
			sleep_time = sleep_time - increment

			## exit if sleep_time = 0
			if (sleep_time == 0):
				rospy.logdebug( "DONE: sleepWhileWaiting for adjusted " + str(sleep_time) + " seconds; end_early=" + str(end_early) )
				return

		increment = max(self.step_resolution, increment)
		## check if sleep_time is evenly divisible by increment
		while (not rospy.is_shutdown()):
			_old_increment = increment

			## perform division
			_st_i = float(sleep_time)/float(increment)  
			if ( float( int(_st_i) ) != _st_i ):
				increment = float(_old_increment) * 0.5
				#rospy.logdebug("**** NEED TO ADJUST INCREMENT **** now increment = " + str(increment))

				## prevent from going beyond nanosecond resolution
				if (increment < 2*self.step_resolution):
					increment = self.step_resolution
					rospy.logdebug("sleepWhileWaiting: **** ADJUSTMENT CUTOFF: increment auto-adjusted from " + str(_orig_increment) + " seconds to " + str(increment) )
					break	## exit while loop
			else:
				if (increment != _orig_increment):
					rospy.logdebug("sleepWhileWaiting: **** increment auto-adjusted from " + str(_orig_increment) + " seconds to " + str(increment) )
				break	## exit while loop
		_adjust_duration = abs(rospy.get_time() - _adjust_start_time)
		if self.VERBOSE_DEBUG:	rospy.logdebug( "duration of adjustment: " + str( _adjust_duration ) + " seconds")
		## now adjust sleep_time to remove adjustment calculation time
		sleep_time = sleep_time - _adjust_duration

		## exit if sleep_time <= 0
		if (sleep_time <= 0):
			rospy.logdebug( "DONE: sleepWhileWaiting for adjusted " + str(sleep_time) + " seconds" )
			return


		### ---- ACTUAL SLEEP LOOP ----
		_start_sleep_time = rospy.get_time()
		#rospy.sleep(increment)	# emulate a do-while
		while (not rospy.is_shutdown() and
			not self.PIC_INTERRUPT and
			(abs(rospy.get_time() - _start_sleep_time) < sleep_time) ):
			rospy.sleep(increment)
			#if self.VERBOSE_DEBUG:	rospy.logdebug(".")

		if self.VERBOSE_DEBUG: rospy.logdebug( "DONE: sleepWhileWaiting for " + str(sleep_time) + " seconds" )

