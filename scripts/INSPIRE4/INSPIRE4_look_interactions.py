#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string
import random

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from base_behavior import * 	## classes baseBehavior and headTiltBaseBehavior
from lookAt import *
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt


########################
## Maki-ro's "lookAt" locations during the INSPIRE4 interaction
##
## Description of INSPIRE4 interaction:
##	- Maki-ro starts face center
##	- Maki-ro performs ONE startle behavior
##	- If the infant fixates on Maki-ro, then Maki-ro will turn towards
##		the designated left or right screen. (1 second)
##	- Maki-ro will watch the stimuli for 8 seconds
##	- Maki-ro will return to face center (1 second)
########################
class lookINSPIRE4Interaction( lookAt ):
	## variables private to this class
	## all instances of this class share the same value
	HP_LEFT_SCREEN = None
	HT_LEFT_SCREEN = None
	EP_LEFT_SCREEN_SACCADE = None
	
	HP_RIGHT_SCREEN = None
	HT_RIGHT_SCREEN = None
	EP_RIGHT_SCREEN_SACCADE = None

	HP_FACE_INFANT = None
	HT_FACE_INFANT = None
	EP_FACE_INFANT_FROM_LEFT_SCREEN = None
	EP_FACE_INFANT_FROM_RIGHT_SCREEN = None

	FACING_LEFT_SCREEN = None
	FACING_RIGHT_SCREEN = None
	FACING_INFANT = None

	def __init__(self, verbose_debug, ros_pub):
## KATE
		## call base class' __init__
		lookAt.__init__( self, verbose_debug, ros_pub )
		#headTiltBaseBehavior.__init__( self, verbose_debug, ros_pub )
		#headPanBaseBehavior.__init__( self, verbose_debug, self.ros_pub )
		## add anything else needed by an instance of this subclass
		self.DC_helper = dynamixelConversions()

		if lookINSPIRE4Interaction.HP_LEFT_SCREEN == None:	
			lookINSPIRE4Interaction.HP_LEFT_SCREEN = HP_LEFT_SCREEN	#650	#620
		if lookINSPIRE4Interaction.HT_LEFT_SCREEN == None:	
			lookINSPIRE4Interaction.HT_LEFT_SCREEN = HT_LEFT_SCREEN
		if lookINSPIRE4Interaction.EP_LEFT_SCREEN_SACCADE == None:
			lookINSPIRE4Interaction.EP_LEFT_SCREEN_SACCADE = EP_LEFT_SCREEN_SACCADE	#EP_RIGHT

		if lookINSPIRE4Interaction.HP_RIGHT_SCREEN == None:
			lookINSPIRE4Interaction.HP_RIGHT_SCREEN = HP_RIGHT_SCREEN	#404
		if lookINSPIRE4Interaction.HT_RIGHT_SCREEN == None:
			lookINSPIRE4Interaction.HT_RIGHT_SCREEN = lookINSPIRE4Interaction.HT_LEFT_SCREEN
		if lookINSPIRE4Interaction.EP_RIGHT_SCREEN_SACCADE == None:
			lookINSPIRE4Interaction.EP_RIGHT_SCREEN_SACCADE = EP_RIGHT_SCREEN_SACCADE	#EP_LEFT

		if lookINSPIRE4Interaction.HP_FACE_INFANT == None:
			lookINSPIRE4Interaction.HP_FACE_INFANT = HP_FRONT
		if lookINSPIRE4Interaction.HT_FACE_INFANT == None:
			lookINSPIRE4Interaction.HT_FACE_INFANT = HT_MIDDLE	
		if lookINSPIRE4Interaction.EP_FACE_INFANT_FROM_LEFT_SCREEN == None:
			lookINSPIRE4Interaction.EP_FACE_INFANT_FROM_LEFT_SCREEN = EP_LEFT
		if lookINSPIRE4Interaction.EP_FACE_INFANT_FROM_RIGHT_SCREEN == None:
			lookINSPIRE4Interaction.EP_FACE_INFANT_FROM_RIGHT_SCREEN = EP_RIGHT

		if lookINSPIRE4Interaction.FACING_LEFT_SCREEN == None:
			lookINSPIRE4Interaction.FACING_LEFT_SCREEN = "leftScreen"
		if lookINSPIRE4Interaction.FACING_RIGHT_SCREEN == None:
			lookINSPIRE4Interaction.FACING_RIGHT_SCREEN = "rightScreen"
		if lookINSPIRE4Interaction.FACING_INFANT == None:
			lookINSPIRE4Interaction.FACING_INFANT = "infant"

		self.facing = None

		if self.makiPP == None:
			self.makiPP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )

		self.ipt_turn = 1000	## ms
		#self.delta_ht = 10	## ticks
		#self.ht_rand_min = lookINSPIRE4Interaction.HT_LEFT_SCREEN - self.delta_ht
		#self.ht_rand_max = lookINSPIRE4Interaction.HT_LEFT_SCREEN +- self.delta_ht
		self.delta_ht = 10	## ticks
		self.ht_rand_min = lookINSPIRE4Interaction.HT_LEFT_SCREEN - self.delta_ht
		self.delta_ht = 0	## ticks
		self.ht_rand_max = lookINSPIRE4Interaction.HT_LEFT_SCREEN + self.delta_ht

		self.__use_shift_gaze = True	## CHANGE TO FALSE TO REVERT

		self.ALIVE = True
		return

## KATE
	### override base class
	#def pubTo_maki_command( self, commandOut, fixed_gaze=True, cmd_prop=True, time_ms=100, time_inc=0.5):
	#	rospy.logdebug("lookINSPIRE4Interaction.pubTo_maki_command(): BEGIN")
	#	## call base class' pubTo_maki_command
	#	headPanBaseBehavior.pubTo_maki_command( self, commandOut, fixed_gaze=fixed_gaze, cmd_prop=cmd_prop, time_ms=time_ms, time_inc=time_inc )
	#	rospy.logdebug("lookINSPIRE4Interaction.pubTo_maki_command(): END")
	#	return

	## override base class
	def start( self, enable_ht=True, auto_face_infant=True ):
		## call base class' start
		lookAt.start( self, enable_ht=enable_ht )
		#headTiltBaseBehavior.start( self, enable_ht=enable_ht )

## KATE 15:25
## NOTE: self.facing isn't well used in new turn behaviors
##	We could save time here...
		## CHECK TO SEE WHICH HEAD PAN POSITION IS CLOSEST
		lookINSPIRE4Interaction.requestFeedback( self, SC_GET_PP )
		_hp_pp = self.makiPP["HP"]	
		_delta_pp = DELTA_PP
		if (abs(_hp_pp - lookINSPIRE4Interaction.HP_FACE_INFANT) < _delta_pp):
			self.facing = lookINSPIRE4Interaction.FACING_INFANT
			rospy.logdebug("lookINSPIRE4Interaction.start(): Maki-ro already facing infant")

		elif (abs(_hp_pp - lookINSPIRE4Interaction.HP_RIGHT_SCREEN) < _delta_pp):
			self.facing = lookINSPIRE4Interaction.FACING_RIGHT_SCREEN
			rospy.logdebug("lookINSPIRE4Interaction.start(): Maki-ro already facing rightScreen")

		elif (abs(_hp_pp - lookINSPIRE4Interaction.HP_LEFT_SCREEN) < _delta_pp):
			self.facing = lookINSPIRE4Interaction.FACING_LEFT_SCREEN
			rospy.logdebug("lookINSPIRE4Interaction.start(): Maki-ro already facing leftScreen")

		else:
			rospy.logwarn("lookINSPIRE4Interaction.start(): Maki-ro not facing rightScreen, leftScreen, or infant")
			if auto_face_infant:
				rospy.logwarn("lookINSPIRE4Interaction.start(): resetting Maki-ro to neutral (facing infant)")
				lookINSPIRE4Interaction.turnToInfant( self )

		return

## KATE
	### override base class
	#def stop( self, disable_ht=True ):
	#	## call base class' stop
	#	return headTiltBaseBehavior.stop( self, disable_ht=disable_ht )

	###########################################
	##
	##	To run, publish to /maki_macro:
	##		turnToScreen right
	##		turnToScreen left
	##
	## Default is right_screen == True, Maki-ro faces to rightScreen; 
	##	otherwise if False, Maki-ro faces to leftScreen
	###########################################
	def turnToScreen( self, right_screen=True ):
		if not isinstance(right_screen, bool):
			rospy.logerr("turnToScreen(): INVALID INPUT: right_screen should be boolean")
			return

		if self.__use_shift_gaze:
			lookINSPIRE4Interaction.turnToScreen_new( self, right_screen=right_screen )
		else:
			lookINSPIRE4Interaction.turnToScreen_old( self, right_screen=right_screen )
		return

## KATE
	def turnToScreen_new( self, right_screen=True ):
		## 2016-08-02, ktsui: head tilt motor is very noisy when trying to hold this
		## 	pose variation
		_pub_ht = False #True

		## Use lookAt.shiftGazeVelocity()
		if right_screen:
			_hp_gp = lookINSPIRE4Interaction.HP_RIGHT_SCREEN
			_ep_gp_shift = lookINSPIRE4Interaction.EP_RIGHT_SCREEN_SACCADE
		else:
			_hp_gp = lookINSPIRE4Interaction.HP_LEFT_SCREEN
			_ep_gp_shift = lookINSPIRE4Interaction.EP_LEFT_SCREEN_SACCADE

		## remain looking at the screen
		_ep_gp_fixed = _ep_gp_shift
		if _pub_ht:	_ht_gp = random.randint(self.ht_rand_min, self.ht_rand_max)
		_duration_s = float(self.ipt_turn) / 1000.0	## seconds

		## apparently requires a little extra time to EP to far side
		if _pub_ht:
			lookINSPIRE4Interaction.shiftGazeVelocity( self, hp_gp=_hp_gp, ht_gp=_ht_gp, ep_gp_shift=_ep_gp_shift, ep_gp_fixed=_ep_gp_fixed, duration_s=_duration_s, padding=-0.1 )
		else:
			lookINSPIRE4Interaction.shiftGazeVelocity( self, hp_gp=_hp_gp, ep_gp_shift=_ep_gp_shift, ep_gp_fixed=_ep_gp_fixed, duration_s=_duration_s, padding=-0.1 )

		return

	def turnToScreen_old( self, right_screen=True ):
		rospy.logdebug("turnToScreen(): BEGIN")

		if not isinstance(right_screen, bool):
			rospy.logerr("turnToScreen(): INVALID INPUT: right_screen should be boolean")
			return

		_pub_hp = True
		_pub_ep = True
		## 2016-08-02, ktsui: head tilt motor is very noisy when trying to hold this
		## 	pose variation
		_pub_ht = True
		_pub_ipt = False
		_ipt_turn = self.ipt_turn	## 1000 ms

		_counter_rotate = False

		if right_screen and (self.facing == lookINSPIRE4Interaction.FACING_RIGHT_SCREEN):
			rospy.logwarn("turnToScreen(): WARNING: Maki-ro is reported as already facing " + lookINSPIRE4Interaction.FACING_RIGHT_SCREEN)

		elif (not right_screen) and (self.facing == lookINSPIRE4Interaction.FACING_LEFT_SCREEN):
			rospy.logwarn("turnToScreen(): WARNING: Maki-ro is reported as already facing " + lookINSPIRE4Interaction.FACING_LEFT_SCREEN)
		
		elif ((right_screen and (self.facing == lookINSPIRE4Interaction.FACING_LEFT_SCREEN)) or
			((not right_screen) and (self.facing == lookINSPIRE4Interaction.FACING_RIGHT_SCREEN))):
			rospy.logwarn("turnToScreen(): WARNING: Maki-ro is NOT intended to move from looking at one screen to the other")
			rospy.logwarn("turnToScreen(): _ipt_turn adjusted to 2x")
			_ipt_turn = 2 * _ipt_turn

		else:
			pass

		### from infant perspective <==> from robot perspective 
		#EP_RIGHT_SCREEN = EP_LEFT
		#EP_LEFT_SCREEN = EP_RIGHT

		_pub_cmd = ""

		_start_time = rospy.get_time()
		if (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):
			_loop_count = 0

			if _pub_ep:	_ep_gp = EP_FRONT	## default
			if _pub_hp:	_hp_gp = HP_FRONT	## default
			if _pub_ht:	_ht_gp = HT_MIDDLE	## default

			if right_screen:	
				## TURN TO RIGHT SCREEN
				##lookINSPIRE4Interaction.EP_RIGHT_SCREEN_SACCADE = 460	#EP_LEFT
				if _pub_ep:	_ep_gp = lookINSPIRE4Interaction.EP_RIGHT_SCREEN_SACCADE
				if _pub_hp:	_hp_gp = lookINSPIRE4Interaction.HP_RIGHT_SCREEN
			else:
				## TURN TO LEFT SCREEN
				#lookINSPIRE4Interaction.EP_LEFT_SCREEN_SACCADE = 578	#EP_RIGHT
				if _pub_ep:	_ep_gp = lookINSPIRE4Interaction.EP_LEFT_SCREEN_SACCADE
				if _pub_hp:	_hp_gp = lookINSPIRE4Interaction.HP_LEFT_SCREEN

			if _pub_ht:	_ht_gp = random.randint(self.ht_rand_min, self.ht_rand_max)

			## GET THE DIFFERENCE
			lookINSPIRE4Interaction.requestFeedback( self, SC_GET_PP )
			if _pub_ep:	_delta_ep_pp = abs(self.makiPP["EP"] - _ep_gp)
			if _pub_hp:	_delta_hp_pp = abs(self.makiPP["HP"] - _hp_gp)
			if _pub_ht:	_delta_ht_pp = abs(self.makiPP["HT"] - _ht_gp)

			## COMPUTE THE GOAL SPEEDS
			if _pub_ep:	_gs_ep = self.DC_helper.getGoalSpeed_ticks_durationMS( _delta_ep_pp, _ipt_turn )
			if _pub_hp:	_gs_hp = self.DC_helper.getGoalSpeed_ticks_durationMS( _delta_hp_pp, _ipt_turn )
			if _pub_ht:	_gs_ht = self.DC_helper.getGoalSpeed_ticks_durationMS( _delta_ht_pp, _ipt_turn )

			## THIS ONE IS TECHNICALLY RIGHT
			#_gs_sequence = ((72,146,200), (72,146,200), (72,146,200), (72,73,200), (72,73,200))
			## USE THIS SEQUENCE FOR turnToScreen
			_gs_sequence = ((72,146,200), (72,146,200), (72,146,200), (72,73,200), (72,73,200))
			_counter_rotate_loop = 2	## after third	## at 600ms

			_first_pass = True
			_loop_count = 0
			_start_time = rospy.get_time()
			## IGNORE _gs_hp_old
			for _gs_hp_old, _gs_ep,_step_timeMS in _gs_sequence:
				#rospy.logdebug( str(_loop_count) + ") _gs_hp=" + str(_gs_hp) + "_gs_ep=" + str(_gs_ep) + ", _step_timeMS=" + str(_step_timeMS))

				if _first_pass:
					_pub_cmd = ""
					if _pub_hp:	_pub_cmd += "HPGS" + str(_gs_hp) 
					if _pub_ht:	_pub_cmd += "HTGS" + str(_gs_ht) 
					if _pub_ep:	_pub_cmd += "EPGS" + str(_gs_ep)
					_pub_cmd += TERM_CHAR_SEND
					rospy.loginfo( _pub_cmd )
					lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd )

					rospy.loginfo("SACCADE!")
					_pub_cmd = ""
					if _pub_hp:	_pub_cmd += "HPGP" + str(_hp_gp) 
					if _pub_ht:	_pub_cmd += "HTGP" + str(_ht_gp) 
					if _pub_ep:	_pub_cmd += "EPGP" + str(_ep_gp) 
					if _pub_ipt:	_pub_cmd += SC_SET_IPT + str(_ipt_turn)
					_pub_cmd += TERM_CHAR_SEND
					rospy.loginfo( _pub_cmd )
					lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd )
					_first_pass = False

				elif (_loop_count == _counter_rotate_loop) and _counter_rotate:
					rospy.loginfo("TRACKING")
					_pub_cmd = ""
					if _pub_ep:	_pub_cmd += "EPGS" + str(_gs_ep)
					if _pub_ep:	_pub_cmd += "EPGP" + str(EP_FRONT) 
					_pub_cmd += TERM_CHAR_SEND
					rospy.loginfo( _pub_cmd )
					lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd )

				else:
					_pub_cmd = ""
					if _pub_hp:	_pub_cmd += "HPGS" + str(_gs_hp) 
					if _pub_ht:	_pub_cmd += "HTGS" + str(_gs_ht) 
					if _pub_ep:	_pub_cmd += "EPGS" + str(_gs_ep)
					_pub_cmd += TERM_CHAR_SEND
					rospy.loginfo( _pub_cmd )
					lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd )

				_sleep_timestep = abs(100 - _step_timeMS)
				if _sleep_timestep > 0:	self.SWW_WI.sleepWhileWaitingMS( _sleep_timestep, end_early=False )
				_loop_count = _loop_count +1
			#end	for _gs_hp_old, _gs_ep,_step_timeMS in _gs_sequence:

			if right_screen:
				self.facing = lookINSPIRE4Interaction.FACING_RIGHT_SCREEN
			else:
				self.facing = lookINSPIRE4Interaction.FACING_LEFT_SCREEN

		else:
			rospy.logwarn("Cannot turnToScreen. Publish 'interaction start' first")
			return
		#end	if (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):

		_duration = abs(rospy.get_time() - _start_time)
		rospy.loginfo( "NUMBER OF TIMESTEPS: " + str(_loop_count) )
		rospy.loginfo( "Duration: " + str(_duration) + " seconds" )

		rospy.logdebug("turnToScreen(): END")
		return

	###########################################
	##
	##	To run, publish to /maki_macro:
	##		turnToInfant
	##		
	## Maki-ro turns back to facing the infant from previously looking at DIRECTION screen
	###########################################
	def turnToInfant( self ):
		if (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):
			if (self.facing == lookINSPIRE4Interaction.FACING_INFANT):
				rospy.logwarn("turnToInfant(): WARNING: Maki-ro is reported as already facing " + lookINSPIRE4Interaction.FACING_INFANT)

			if self.__use_shift_gaze:
				lookINSPIRE4Interaction.turnToInfant_new( self )
			else:
				lookINSPIRE4Interaction.turnToInfant_old( self )
		else:
			rospy.logwarn("Cannot turnToInfant. Publish 'interaction start' first")
			return
		return

## KATE
	def turnToInfant_new( self ):
		## Use lookAt.shiftGazeVelocity()
		_hp_gp = lookINSPIRE4Interaction.HP_FACE_INFANT
		_ht_gp = lookINSPIRE4Interaction.HT_FACE_INFANT
		_duration_s = float(self.ipt_turn) / 1000.0	## seconds

		## apparently requires a little extra time to EP to far side
		lookINSPIRE4Interaction.shiftGazeVelocity( self, hp_gp=_hp_gp, ht_gp=_ht_gp, duration_s=_duration_s, padding=-0.1 )
		return

	def turnToInfant_old( self ):
		rospy.logdebug("turnToInfant(): BEGIN")

		if (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):
		#	if (self.facing == lookINSPIRE4Interaction.FACING_INFANT):
		#		rospy.logwarn("turnToInfant(): WARNING: Maki-ro is reported as already facing " + lookINSPIRE4Interaction.FACING_INFANT)

			_pub_cmd = ""

			_pub_hp = True
			_pub_ep = True 
			_pub_ht = True
			_pub_ipt = False
			_ipt_turn = self.ipt_turn	## 1000 ms
			_counter_rotate = True

			if _pub_hp:	_hp_gp = lookINSPIRE4Interaction.HP_FACE_INFANT
			if _pub_ht:	_ht_gp = lookINSPIRE4Interaction.HT_FACE_INFANT
			if _pub_ep:
				_ep_gp = EP_FRONT
				if _counter_rotate and (self.facing == lookINSPIRE4Interaction.FACING_LEFT_SCREEN):
					_ep_gp = 460	## EP min value
				elif _counter_rotate and (self.facing == lookINSPIRE4Interaction.FACING_RIGHT_SCREEN):
					_ep_gp = 578	## EP max value
				else:
					pass

			## GET THE DIFFERENCE
			lookINSPIRE4Interaction.requestFeedback( self, SC_GET_PP )
			if _pub_ep:	_delta_ep_pp = abs(self.makiPP["EP"] - _ep_gp)
			if _pub_hp:	_delta_hp_pp = abs(self.makiPP["HP"] - _hp_gp)
			if _pub_ht:	_delta_ht_pp = abs(self.makiPP["HT"] - _ht_gp)

			## COMPUTE THE GOAL SPEEDS
			if _pub_ep:	_gs_ep = self.DC_helper.getGoalSpeed_ticks_durationMS( _delta_ep_pp, _ipt_turn )
			if _pub_hp:	_gs_hp = self.DC_helper.getGoalSpeed_ticks_durationMS( _delta_hp_pp, _ipt_turn )
			if _pub_ht:	_gs_ht = self.DC_helper.getGoalSpeed_ticks_durationMS( _delta_ht_pp, _ipt_turn )


			## USE THIS SEQUNCE FOR turnToInfant
			_gs_sequence = ((72,173,100), (72,173,100), (72,45,100), (72,45,100), (72,45,100), (72,45,100), (72,45,100), (72,45,100), (72,45,100), (72,45,100))
			## THIS IS TECHNICALLY CORRECT BY COMPUTATION for 1000 ms, but eye pan doesn't finish in time
			#_gs_sequence = ((72,173,100), (72,173,100), (72,33,100), (72,33,100), (72,33,100), (72,33,100), (72,33,100), (72,33,100), (72,33,100), (72,33,100))
			_counter_rotate_loop = 2	## after second


			_first_pass = True
			_loop_count = 0
			_start_time = rospy.get_time()
			## IGNORE _gs_hp_old
			for _gs_hp_old, _gs_ep,_step_timeMS in _gs_sequence:

				#rospy.logdebug( str(_loop_count) + ") _gs_hp=" + str(_gs_hp) + "_gs_ep=" + str(_gs_ep) + ", _step_timeMS=" + str(_step_timeMS))

				if _first_pass:
					## SEND THE GOAL SPEEDS *BEFORE* THE GOAL POSITIONS
					_pub_cmd = ""
					if _pub_hp:	_pub_cmd += "HPGS" + str(_gs_hp) 
					if _pub_ht:	_pub_cmd += "HTGS" + str(_gs_ht) 
					if _pub_ep:	_pub_cmd += "EPGS" + str(_gs_ep)
					_pub_cmd += TERM_CHAR_SEND
					rospy.loginfo( _pub_cmd )
					lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd )

					rospy.loginfo("SACCADE!")
					_pub_cmd = ""
					if _pub_hp:	_pub_cmd += "HPGP" + str(_hp_gp) 
					if _pub_ht:	_pub_cmd += "HTGP" + str(_ht_gp) 
					if _pub_ep:	_pub_cmd += "EPGP" + str(_ep_gp) 
					if _pub_ipt:	_pub_cmd += SC_SET_IPT + str(_ipt_turn)
					_pub_cmd += TERM_CHAR_SEND
					rospy.loginfo( _pub_cmd )
					lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd )
					_first_pass = False

				elif (_loop_count >= _counter_rotate_loop) and _counter_rotate:
					rospy.loginfo("TRACKING")
					_pub_cmd = ""
					if _pub_ep:	_pub_cmd += "EPGS" + str(_gs_ep)
					if _pub_ep:	_pub_cmd += "EPGP" + str(EP_FRONT) 
					_pub_cmd += TERM_CHAR_SEND
					rospy.loginfo( _pub_cmd )
					lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd )

				else:
					_pub_cmd = ""
					if _pub_hp:	_pub_cmd += "HPGS" + str(_gs_hp) 
					if _pub_ht:	_pub_cmd += "HTGS" + str(_gs_ht) 
					if _pub_ep:	_pub_cmd += "EPGS" + str(_gs_ep)
					_pub_cmd += TERM_CHAR_SEND
					rospy.loginfo( _pub_cmd )
					lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd )

				## make sure to sleep if needed, so that this loop occurs every 100ms
				_sleep_timestep = abs(100 - _step_timeMS)
				if _sleep_timestep > 0:	self.SWW_WI.sleepWhileWaitingMS( _sleep_timestep, end_early=False )
				_loop_count = _loop_count +1
			#end	for _gs_hp_old, _gs_ep,_step_timeMS in _gs_sequence:


			## NOTE: THIS ISN'T CHECKED via self.makiPP
			self.facing = lookINSPIRE4Interaction.FACING_INFANT

		else:
			rospy.logwarn("Cannot turnToInfant. Publish 'interaction start' first")
			return
		#end	if (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):


		_duration = abs(rospy.get_time() - _start_time)
		rospy.loginfo( "NUMBER OF TIMESTEPS: " + str(_loop_count) )
		rospy.loginfo( "Duration: " + str(_duration) + " seconds" )

		rospy.logdebug("turnToInfant(): END")
		return


	def parse_maki_macro( self, msg ):
		print msg.data

		if msg.data == "interaction start":
			## try to nicely startup without jerking MAKI's head tilt servo
			lookINSPIRE4Interaction.start( self )

		elif msg.data.startswith( "interaction stop" ):
			if msg.data.endswith( "disable_ht=False" ):
				lookINSPIRE4Interaction.stop( self, disable_ht=False )
			else:
				lookINSPIRE4Interaction.stop( self )

		elif msg.data.startswith( "turnToScreen left" ):
			lookINSPIRE4Interaction.turnToScreen( self, right_screen=False )

		elif msg.data.startswith( "turnToScreen right" ):
			lookINSPIRE4Interaction.turnToScreen( self, right_screen=True )

		elif msg.data == "turnToInfant":
			lookINSPIRE4Interaction.turnToInfant( self )

		else:
			pass

		return

if __name__ == '__main__':
        print "__main__: BEGIN"
	lookStimuli = lookINSPIRE4Interaction( True, None )

	rospy.Subscriber( "/maki_macro", String, lookStimuli.parse_maki_macro )
        rospy.logdebug( "now subscribed to /maki_macro" )

	rospy.spin()   ## keeps python from exiting until this node is stopped

        print "__main__: END"

