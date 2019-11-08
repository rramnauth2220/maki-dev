#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string
import random

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from base_behavior import *     ## classes baseBehavior, headTiltBaseBehavior, headPanBaseBehavior
from lookAt import *
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt


########################
## Maki-ro's "lookAt" locations during the INSPIRE4 intro 
##
## Description of INSPIRE4 intro:
##  - START: Maki-ro is asleep
##  - Experimenter wakes Maki-ro up by touching shoulder
##  - Maki-ro wakes up
##  [*] Maki-ro looks at experimenter
##  - Experimenter waves hello
##  - Maki-ro bows its head in return
##  - Experimenter covers own eyes and plays peek-a-boo once
##  - Maki-ro responds with startle
##  - Maki-ro relaxes from startle
##  - Experimenter show Maki-ro strobing LED ball
##  - Maki-ro performs small, quick nod
##  - Experimenter moves ball to upper right calibration point
##  - Experimenter shakes the ball
##  [*] Maki-ro looks to (ball in) upper right calibration point
##  - Experimenter moves ball to lower right calibration point
##  - Experimenter shakes the ball
##  [*] Maki-ro looks to (ball in) lower right calibration point
##  - Experimenter moves ball in front of infant
##  - Experimenter shakes the ball
##  [*] Maki-ro looks to (ball in) front of infant
##  - Experimenter removes ball and leaves
##  - Maki-ro looks at infant, as if studying infant's face
########################
#class lookINSPIRE4Intro( eyelidHeadTiltBaseBehavior, headPanBaseBehavior ):
class lookINSPIRE4Intro( eyelidHeadTiltBaseBehavior, lookAt ):
    ## variables private to this class
    ## all instances of this class share the same value
    __is_intro_running = None
    __is_startled = None

    HP_EXPERIMENTER = None
    HT_EXPERIMENTER = None

    HP_BALL_LOWER_RIGHT = None
    HT_BALL_LOWER_RIGHT = None

    HP_BALL_UPPER_RIGHT = None
    HT_BALL_UPPER_RIGHT = None

    HP_ASLEEP = None
    HT_ASLEEP = None
    LL_ASLEEP = None

    HP_FACE_INFANT = None
    HT_FACE_INFANT = None

    FACING_EXPERIMENTER = None
    FACING_BALL_LOWER_RIGHT = None
    FACING_BALL_UPPER_RIGHT = None
    FACING_ASLEEP = None
    FACING_INFANT = None

    def __init__(self, verbose_debug, ros_pub):
        ## call base class' __init__
        eyelidHeadTiltBaseBehavior.__init__( self, verbose_debug, ros_pub )
        #headPanBaseBehavior.__init__( self, verbose_debug, self.ros_pub )
        lookAt.__init__( self, verbose_debug, self.ros_pub )
        ## add anything else needed by an instance of this subclass
        self.DC_helper = dynamixelConversions()

        if self.makiPP == None:
            self.makiPP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )

        if lookINSPIRE4Intro.__is_intro_running == None:
            lookINSPIRE4Intro.__is_intro_running = False

        ## These values should match asleep_awake.py
        ##  HP_AWAKE_EXP and HT_AWAKE_EXP
        if lookINSPIRE4Intro.HP_EXPERIMENTER == None:   
            lookINSPIRE4Intro.HP_EXPERIMENTER = 620     ## ticks
        if lookINSPIRE4Intro.HT_EXPERIMENTER == None:   
            lookINSPIRE4Intro.HT_EXPERIMENTER = 570     ## ticks

        if lookINSPIRE4Intro.HP_BALL_UPPER_RIGHT == None:
            lookINSPIRE4Intro.HP_BALL_UPPER_RIGHT = 404     ## ticks
        if lookINSPIRE4Intro.HT_BALL_UPPER_RIGHT == None:
            lookINSPIRE4Intro.HT_BALL_UPPER_RIGHT = lookINSPIRE4Intro.HT_EXPERIMENTER

        if lookINSPIRE4Intro.HP_BALL_LOWER_RIGHT == None:
            lookINSPIRE4Intro.HP_BALL_LOWER_RIGHT = lookINSPIRE4Intro.HP_BALL_UPPER_RIGHT
        if lookINSPIRE4Intro.HT_BALL_LOWER_RIGHT == None:
            lookINSPIRE4Intro.HT_BALL_LOWER_RIGHT = 480     ## ticks

        if lookINSPIRE4Intro.HP_ASLEEP == None:
            lookINSPIRE4Intro.HP_ASLEEP = HP_LEFT
        if lookINSPIRE4Intro.HT_ASLEEP == None:
            lookINSPIRE4Intro.HT_ASLEEP = HT_DOWN
        if lookINSPIRE4Intro.LL_ASLEEP == None:
            lookINSPIRE4Intro.LL_ASLEEP = LL_CLOSE_MAX

        ## NOTE: This should be the same value as lookINSPIRE4Interaction
        if lookINSPIRE4Intro.HP_FACE_INFANT == None:
            lookINSPIRE4Intro.HP_FACE_INFANT = HP_FRONT
        if lookINSPIRE4Intro.HT_FACE_INFANT == None:
            lookINSPIRE4Intro.HT_FACE_INFANT = HT_MIDDLE

        if lookINSPIRE4Intro.FACING_EXPERIMENTER == None:
            lookINSPIRE4Intro.FACING_EXPERIMENTER = "experimenter"
        if lookINSPIRE4Intro.FACING_BALL_UPPER_RIGHT == None:
            lookINSPIRE4Intro.FACING_BALL_UPPER_RIGHT = "ballUpperRight"
        if lookINSPIRE4Intro.FACING_BALL_LOWER_RIGHT == None:
            lookINSPIRE4Intro.FACING_BALL_LOWER_RIGHT = "ballLowerRight"
        if lookINSPIRE4Intro.FACING_ASLEEP == None:
            lookINSPIRE4Intro.FACING_ASLEP = "asleep"
        if lookINSPIRE4Intro.FACING_INFANT == None:
            lookINSPIRE4Intro.FACING_INFANT = "infant"

        self.facing = None

        self.ipt_turn = 1000    ## ms
        self.ipt_turn_s = float(self.ipt_turn / 1000.0)     ## s
        #self.delta_ht = 10     ## ticks
        #self.ht_rand_min = lookINSPIRE4Intro.HT_EXPERIMENTER - self.delta_ht
        #self.ht_rand_max = lookINSPIRE4Intro.HT_EXPERIMENTER +- self.delta_ht

        ## from lookINSPIRE4Intro
        if lookINSPIRE4Intro.__is_startled == None:
            lookINSPIRE4Intro.__is_startled = False
        self.HT_STARTLE = HT_STARTLE   #530    #525

        self.HT_NEUTRAL = HT_MIDDLE
        self.HT_GS_DEFAULT = 15         ## as set in Arbotix-M driver
        self.HT_GS_MAX = 75     #60     #50
        self.HT_GS_MIN = 12     ##10    ## too slow, stall out when moving too slow

        self.LL_STARTLE = LL_OPEN_MAX
        self.LL_NEUTRAL = LL_OPEN_DEFAULT
        self.LL_GS_DEFAULT = 100    ## as set in Arbotix-M driver
        self.LL_GS_MIN = 10

        ### Game variables
        #if lookINSPIRE4Intro.__is_game_running == None:
        #   lookINSPIRE4Intro.__is_game_running = False
        #if lookINSPIRE4Intro.__is_game_exit == None:
        #   lookINSPIRE4Intro.__is_game_exit = False
        #self.ll_startle = LL_OPEN_MAX
        #
        #self.last_startle_time = None
        #self.next_startle_time = None
        #
        #self.game_state = None
        #
        #self.repetitions = 6   ## do 6 rounds of infant engagement behavior max
        #
        #self.duration_between_startle_min = 2.0    ## seconds
        #self.duration_between_startle_max = 5.0    ## seconds

        lookINSPIRE4Intro.calculatePose( self )

        self.__use_shift_gaze = True    ## CHANGE TO FALSE TO REVERT

        self.ALIVE = True
        return

    ## override base class
    def pubTo_maki_command( self, commandOut, fixed_gaze=True, cmd_prop=True, time_ms=100, time_inc=0.5):
        rospy.logdebug("lookINSPIRE4Intro.pubTo_maki_command(): BEGIN")
        ### call base class' pubTo_maki_command
        #headPanBaseBehavior.pubTo_maki_command( self, commandOut, fixed_gaze=fixed_gaze, cmd_prop=cmd_prop, time_ms=time_ms, time_inc=time_inc )
        lookAt.pubTo_maki_command( self, commandOut, fixed_gaze=fixed_gaze, cmd_prop=cmd_prop, time_ms=time_ms, time_inc=time_inc )
        rospy.logdebug("lookINSPIRE4Intro.pubTo_maki_command(): END")
        return

    ## override base class
    def start( self, enable_ht=True ):
        ## call base class' start
        eyelidHeadTiltBaseBehavior.start( self, enable_ht=enable_ht )

        ## CHECK TO SEE WHICH POSITION IS CLOSEST
        lookINSPIRE4Intro.requestFeedback( self, SC_GET_PP )
        _hp_pp = self.makiPP["HP"]  
        _ht_pp = self.makiPP["HT"]
        _ll_pp = self.makiPP["LL"]
        _delta_pp = DELTA_PP
        if (abs(_hp_pp - lookINSPIRE4Intro.HP_FACE_INFANT) < _delta_pp):
            self.facing = lookINSPIRE4Intro.FACING_INFANT
            rospy.logdebug("lookINSPIRE4Intro.start(): Maki-ro already facing infant")

        elif (abs(_hp_pp - lookINSPIRE4Intro.HP_BALL_UPPER_RIGHT) < _delta_pp):
            if (abs(_ht_pp - lookINSPIRE4Intro.HT_BALL_UPPER_RIGHT) < _delta_pp):
                self.facing = lookINSPIRE4Intro.FACING_BALL_UPPER_RIGHT
                rospy.logdebug("lookINSPIRE4Intro.start(): Maki-ro already facing upper right calibration point")
            elif ((abs(_ht_pp - lookINSPIRE4Intro.HT_ASLEEP) < _delta_pp) and
                (abs(_ll_pp - lookINSPIRE4Intro.LL_ASLEEP) < _delta_pp)):
                self.facing = lookINSPIRE4Intro.FACING_ASLEEP
                rospy.logdebug("lookINSPIRE4Intro.start(): Maki-ro already facing asleep")
            elif (abs(_ht_pp - lookINSPIRE4Intro.HT_BALL_LOWER_RIGHT) < _delta_pp):
                self.facing = lookINSPIRE4Intro.FACING_BALL_LOWER_RIGHT
                rospy.logdebug("lookINSPIRE4Intro.start(): Maki-ro already facing lower right calibration point")
            else:
                rospy.logdebug("lookINSPIRE4Intro.start(): Maki-ro already facing toward rightScreen")

        elif (abs(_hp_pp - lookINSPIRE4Intro.HP_EXPERIMENTER) < _delta_pp):
            if (abs(_ht_pp - lookINSPIRE4Intro.HT_EXPERIMENTER) < _delta_pp):
                self.facing = lookINSPIRE4Intro.FACING_EXPERIMENTER
                rospy.logdebug("lookINSPIRE4Intro.start(): Maki-ro already facing Experimenter")
            else:
                rospy.logdebug("lookINSPIRE4Intro.start(): Maki-ro already facing leftScreen")

        else:
            pass


        if self.facing == None:
            rospy.logwarn("lookINSPIRE4Intro.start(): Maki-ro not asleep or facing experimenter, upper right calibration point, lower right calibration point, or infant")
            ## TODO: Update with something relevant for the intro script, 
            ##  likely put Maki-ro to sleep
            #rospy.logwarn("lookINSPIRE4Intro.start(): resetting Maki-ro to neutral (facing infant)")
            #lookINSPIRE4Intro.turnToInfant( self )

        return

    ## override base class
    def stop( self, disable_ht=True ):
        ## call base class' stop
        return eyelidHeadTiltBaseBehavior.stop( self, disable_ht=disable_ht )


    def calculatePose( self ):
        _pub_cmd = ""

        ## from facing infant, look to experimenter
        _ipt = self.ipt_turn
        _pub_cmd = ""
        _pub_cmd += "HP" + SC_SET_GP + str(lookINSPIRE4Intro.HP_EXPERIMENTER)
        _pub_cmd += "HT" + SC_SET_GP + str(lookINSPIRE4Intro.HT_EXPERIMENTER)
        _pub_cmd += SC_SET_IPT + str(_ipt)
        _pub_cmd += TERM_CHAR_SEND
        self.pub_cmd_look_fromInfant_toExperimenter = _pub_cmd

        ## from asleep, look to experimenter
        _ipt = int( 2 * self.ipt_turn )
        _pub_cmd = ""
        _pub_cmd += "HP" + SC_SET_GP + str(lookINSPIRE4Intro.HP_EXPERIMENTER)
        _pub_cmd += "HT" + SC_SET_GP + str(lookINSPIRE4Intro.HT_EXPERIMENTER)
        _pub_cmd += SC_SET_IPT + str(_ipt)
        _pub_cmd += TERM_CHAR_SEND
        self.pub_cmd_look_fromAsleep_toExperimenter = _pub_cmd

        ## from looking at experimenter, look to upper right calibration point
        _ipt = int( 2 * self.ipt_turn)
        _pub_cmd = ""
        _pub_cmd += "HP" + SC_SET_GP + str(lookINSPIRE4Intro.HP_BALL_UPPER_RIGHT)
        _pub_cmd += "HT" + SC_SET_GP + str(lookINSPIRE4Intro.HT_BALL_UPPER_RIGHT)
        _pub_cmd += SC_SET_IPT + str(_ipt)
        _pub_cmd += TERM_CHAR_SEND
        self.pub_cmd_look_fromExperimenter_toBallUpperRight = _pub_cmd

        ## TODO: manipulate LL... raise???
        ## from looking at upper right calibration point, look to lower right calibration point
        _ipt = self.ipt_turn
        _pub_cmd = ""
        _pub_cmd += "HP" + SC_SET_GP + str(lookINSPIRE4Intro.HP_BALL_LOWER_RIGHT)
        _pub_cmd += "HT" + SC_SET_GP + str(lookINSPIRE4Intro.HT_BALL_LOWER_RIGHT)
        _pub_cmd += SC_SET_IPT + str(_ipt)
        _pub_cmd += TERM_CHAR_SEND
        self.pub_cmd_look_fromBallUpperRight_toBallLowerRight = _pub_cmd

        ## from looking at lower right calibration point, look to infant
        _ipt = self.ipt_turn
        _pub_cmd = ""
        _pub_cmd += "HP" + SC_SET_GP + str(lookINSPIRE4Intro.HP_FACE_INFANT)
        _pub_cmd += "HT" + SC_SET_GP + str(lookINSPIRE4Intro.HT_FACE_INFANT)
        _pub_cmd += SC_SET_IPT + str(_ipt)
        _pub_cmd += TERM_CHAR_SEND
        self.pub_cmd_look_fromBallLowerRight_toInfant = _pub_cmd

        return

    ###########################
    ##
    ##  To run, publish to /maki_macro
    ##      intro lookAtExperimenter
    ##
    ###########################
    def macroLookAtExperimenter( self ):
        rospy.logdebug("macrolookAtExperimenter(): BEGIN")

        if self.ALIVE:
            if not rospy.is_shutdown():
                rospy.logdebug("NOT rospy.is_shutdown()")
                pass
            else:
                return

            if self.mTT_INTERRUPT:  
                rospy.logdebug("mTT_INTERRUPT=" + str(mTT_INTERRUPT))
                return
            else:
                rospy.loginfo("-----------------")

## KATE
                if self.__use_shift_gaze:
                    ## Update to class lookAt.shiftGazeVelocity( hp_gp, ep_gp_shift, ep_gp_fixed, hp_pp, ep_pp, duration_s)
                    lookINSPIRE4Intro.shiftGazeVelocity( self, hp_gp=lookINSPIRE4Intro.HP_EXPERIMENTER, ht_gp=lookINSPIRE4Intro.HT_EXPERIMENTER, duration_s=self.ipt_turn_s, padding=-0.1)
                else:   ## REVERT
                    lookINSPIRE4Intro.lookAt_jointHeadAndEyePan( self, self.pub_cmd_look_fromInfant_toExperimenter )
                    ##baseBehavior.pubTo_maki_command( self, str(self.look_at) )
                    ##self.sww_wi.sleepWhileWaitingMS( 2000 )

                rospy.loginfo("-----------------")

            #end    if not self.mTT_INTERRUPT:
        #end    if self.ALIVE:

        rospy.logdebug("macroLookAtExperimenter(): END")
        return

    ###########################
    ##
    ##  To run, publish to /maki_macro
    ##      intro lookAtBallLocationRight lower
    ##      intro lookAtBallLocationRight upper
    ##
    ###########################
    #def macroLookAtBallLocationRight( self, lower=False, upper=False, shift=True ):
    def macroLookAtBallLocationRight( self, lower=False, upper=False ):
        rospy.logdebug("macroLookAtBallLocationRight(): BEGIN")

        ## check inputs
        if not ((isinstance( lower, bool )) and 
            (isinstance( upper, bool )) and
            (lower or upper)):
            rospy.logwarn("macroLookAtBallLocationRight(): INVALID INPUT: expected lower and upper to be of type bool, with at least one True")
            return

        if self.ALIVE:
            if not rospy.is_shutdown():
                rospy.logdebug("NOT rospy.is_shutdown()")
                pass
            else:
                return

            if self.mTT_INTERRUPT:  
                rospy.logdebug("mTT_INTERRUPT=" + str(mTT_INTERRUPT))
                return
            else:
                rospy.loginfo("-----------------")
                #baseBehavior.pubTo_maki_command( self, str(self.look_ball_loc_lower_right) )
                #self.sww_wi.sleepWhileWaitingMS( 2000 )

                ## From facing the Friend, Maki-ro looks at UPPER RIGHT first
                if upper:   
## KATE
                    if self.__use_shift_gaze:
                        #lookINSPIRE4Intro.shiftGazeVelocity( self, hp_gp=lookINSPIRE4Intro.HP_BALL_UPPER_RIGHT, duration_s=2.0*self.ipt_turn_s )
                        lookINSPIRE4Intro.shiftGazeVelocity( self, hp_gp=lookINSPIRE4Intro.HP_BALL_UPPER_RIGHT, ep_gp_shift=lookAt.EP_MIN_LEFT, ht_gp=lookINSPIRE4Intro.HT_BALL_UPPER_RIGHT, duration_s=float(2.0*self.ipt_turn_s), padding=-0.1 )
                    else:   ## REVERT
                        lookINSPIRE4Intro.lookAt_jointHeadAndEyePan( self, self.pub_cmd_look_fromExperimenter_toBallUpperRight )

                ## Then from facing upper right, Maki-ro looks at LOWER RIGHT
                if lower:   
                    if self.__use_shift_gaze:
                        _ipt = self.ipt_turn
                        _pub_cmd = ""
                        _pub_cmd += "EP" + SC_SET_GP + str(EP_FRONT)
                        _pub_cmd += "HP" + SC_SET_GP + str(lookINSPIRE4Intro.HP_BALL_LOWER_RIGHT)
                        _pub_cmd += "HT" + SC_SET_GP + str(lookINSPIRE4Intro.HT_BALL_LOWER_RIGHT)
                        _pub_cmd += SC_SET_IPT + str(_ipt)
                        _pub_cmd += TERM_CHAR_SEND
                        lookINSPIRE4Intro.lookAt_jointHeadAndEyePan( self, _pub_cmd, monitor=True)
                        pass
                    else:   ## REVERT
                        lookINSPIRE4Intro.lookAt_jointHeadAndEyePan( self, self.pub_cmd_look_fromBallUpperRight_toBallLowerRight )

                rospy.loginfo("-----------------")

            #end    if not self.mTT_INTERRUPT:
        #end    if self.ALIVE:

        rospy.logdebug("macroLookAtBallLocationRight(): END")
        return

    ###########################
    ##
    ##  To run, publish to /maki_macro
    ##      intro lookAtInfant
    ##
    ###########################
    def macroLookAtInfant( self ):
        rospy.logdebug("macroAtInfant(): BEGIN")

        if self.ALIVE:
            if not rospy.is_shutdown():
                rospy.logdebug("NOT rospy.is_shutdown()")
                pass
            else:
                return

            if self.mTT_INTERRUPT:  
                rospy.logdebug("mTT_INTERRUPT=" + str(mTT_INTERRUPT))
                return
            else:
                rospy.loginfo("-----------------")
                #baseBehavior.pubTo_maki_command( self, str(self.look_neutral) )
                #self.sww_wi.sleepWhileWaitingMS( 2000 )

                ## Maki-ro should have been previously looking at the lower right
                ##  calibration point
## KATE
                if self.__use_shift_gaze:
                    lookINSPIRE4Intro.shiftGazeVelocity( self, hp_gp=lookINSPIRE4Intro.HP_FACE_INFANT, ht_gp=lookINSPIRE4Intro.HT_FACE_INFANT, padding=-0.1 )
## 2016-06-16, KATE
                    ### HACK... it looks weird for the robot to finally adjust its eye tilt when exiting the whole experiment... and then fall asleep...
                    #lookINSPIRE4Intro.pubTo_maki_command( self, "reset" )
                    ## 2016-06-17, ktsui: This still didn't quite look right, so commented out

                else:   ## REVERT
                    lookINSPIRE4Intro.lookAt_jointHeadAndEyePan( self, self.pub_cmd_look_fromBallLowerRight_toInfant )
                    ## TODO: Maki-ro doesn't quite return to HT_MIDDLE
                    lookINSPIRE4Intro.pubTo_maki_command( self, "reset" )

                rospy.loginfo("-----------------")

            #end    if not self.mTT_INTERRUPT:
        #end    if self.ALIVE:

        rospy.logdebug("macroLookAtInfant(): END")
        return


    ## TODO: lookAt should shift to new ground state
    ## This is a work in progress
    def lookAt_jointHeadAndEyePan( self, commandOut, monitor=True, hp_gp=None, ht_gp=None ):
        rospy.logdebug("lookAt(): BEGIN")

        if (monitor and
            ((hp_gp != None) and isinstance(hp_gp, int)) and
            ((ht_gp != None) and isinstance(ht_gp, int))):
            pass
        else:
            rospy.logwarn("lookAt(): INVALID INPUTS: monitor must be True AND hp_gp and ht_gp must be valid integers; set monitor to False")
            monitor=False

        ## TODO: Check validity of changing from pose to another... INTENDED vs. not
        #if (self.facing == lookINSPIRE4Intro.FACING_INFANT):
        #   rospy.logwarn("turnToScreen(): WARNING: Maki-ro is reported as already facing " + lookINSPIRE4Intro.FACING_INFANT)

        _pub_cmd = ""

        _start_time = rospy.get_time()
        if (lookINSPIRE4Intro.__is_intro_running and (self.ALIVE) and 
            (not self.mTT_INTERRUPT) and (not rospy.is_shutdown())):
            _loop_count = 0


            ## NOTE: unnecssary to specify eye pan since monitorMoveToGP() and
            ## pubTo_maki_command() are inherited from headPanBaseBehavior
            ## Will automatically adjust eye pan based on head pan, if not specified
            _pub_cmd = commandOut
            rospy.logwarn( _pub_cmd )
            if monitor:
                try:
                    lookINSPIRE4Intro.monitorMoveToGP( self, _pub_cmd, hp_gp=hp_gp, ht_gp=ht_gp )
                except rospy.exceptions.ROSException as _e:
                    rospy.logerr("lookAt(): " + str(_e))
                    _tmp = re.search( "IPT([0-9]+)", _pub_cmd )
                    lookINSPIRE4Intro.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
                    if _tmp != None:
                        self.SWW_WI.sleepWhileWaitingMS( int(_tmp.group(1))-100, end_early=False )
        
            else:
                _tmp = re.search( "IPT([0-9]+)", _pub_cmd )
                lookINSPIRE4Intro.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
                if _tmp != None:
                    self.SWW_WI.sleepWhileWaitingMS( int(_tmp.group(1))-100, end_early=False )
        
        else:
            rospy.logwarn("Cannot lookAt. Publish 'intro start' first")
            return
        #end    if (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):

        _duration = abs(rospy.get_time() - _start_time)
        rospy.loginfo( "NUMBER OF TIMESTEPS: " + str(_loop_count) )
        rospy.loginfo( "Duration: " + str(_duration) + " seconds" )

        rospy.logdebug("lookAt(): END")
        return

    ## NOTE: during intro, we expect Maki-ro's head to be tilted up to 
    ##  face the experimenter
    def macroGreeting( self, nod_angle=15.0, duration=600, repetitions=1 ):
        rospy.logdebug("macroGreeting(): BEGIN")

        _start_time = rospy.get_time()
        ## check the inputs
        if isinstance(nod_angle, float) or isinstance(nod_angle, int):
            pass
        else:
            rospy.logwarn("macroGreeting(): INVALID VALUE: nod_angle=" + str(nod_angle) + "; updated to 15 degrees")
            nod_angle = 15.0    ## degrees
        if isinstance(duration, float) or isinstance(duration, int):
            pass
        else:
            rospy.logwarn("macroGreeting(): INVALID VALUE: duration=" + str(duration) + "; updated to 600 milliseconds")
            duration = 600  ## milliseconds
        if isinstance(repetitions, int) and (repetitions > 0):
            pass
        else:
            rospy.logwarn("macroGreeting(): INVALID VALUE: repetitions=" + str(repetitions) + "; updated to 1")
            repetitions = 1

        _pub_ipt = False #True
        _monitor = True     #False
        _duration_nod = duration

        _my_ticks_ht = self.DC_helper.convertToTicks_degrees( nod_angle )
        rospy.logdebug("_my_ticks_ht: " + str(nod_angle) + " degrees is " + str(_my_ticks_ht) + " ticks")
        _gs_ht = abs( self.DC_helper.getGoalSpeed_ticks_durationMS( _my_ticks_ht, _duration_nod) )
        _my_ticks_ht = int( float(_my_ticks_ht * 0.5) + 0.5 )

        _my_ticks_ll = self.DC_helper.convertToTicks_degrees( nod_angle * 0.36 )    ## same constant Todorovic 2009
        rospy.logdebug("_my_ticks_ll: " + str(nod_angle) + " degrees is " + str(_my_ticks_ll) + " ticks")
        _gs_ll = abs( self.DC_helper.getGoalSpeed_ticks_durationMS( _my_ticks_ll, _duration_nod) )
        _my_ticks_ll = int( float(_my_ticks_ll * 0.5) + 0.5 )
        rospy.logdebug("DIVIDED IN HALF: _my_ticks_ht=" + str(_my_ticks_ht) + "; _my_ticks_ll=" + str(_my_ticks_ll))

        ## Store initial pose
        lookINSPIRE4Intro.requestFeedback( self, SC_GET_PP )
        self.previous_ll = self.makiPP["LL"]
        self.previous_ht = self.makiPP["HT"]

        _my_head_nod_up_ht = self.previous_ht + int( float(_my_ticks_ht * 0.5) + 0.5 )
        _my_head_nod_down_ht = self.previous_ht - _my_ticks_ht
        _my_head_nod_up_ll = self.previous_ll - _my_ticks_ll    ## head up, eyelids down
        _my_head_nod_down_ll = self.previous_ll + _my_ticks_ll  ## head down, eyelids up

        ## generate servo control command to set goal positions
        ## NOTE: on the Arbotix-M side, a sync_write function is used
        ## to simultaneously broadcast the updated goal positions
        _duration_nod_up = float(_duration_nod) * 0.25   #0.25
        _head_nod_up_gp_cmd = ""
        _head_nod_up_gp_cmd += "LL" + SC_SET_GP + str(_my_head_nod_up_ll)
        _head_nod_up_gp_cmd += "HT" + SC_SET_GP + str(_my_head_nod_up_ht)
        if _pub_ipt:    _head_nod_up_gp_cmd += SC_SET_IPT + str( int(_duration_nod_up + 0.5) )
        _head_nod_up_gp_cmd += TERM_CHAR_SEND

        _duration_nod_down = float(_duration_nod) * 0.6     #0.5
        _head_nod_down_gp_cmd = ""
        _head_nod_down_gp_cmd += "LL" + SC_SET_GP + str(_my_head_nod_down_ll)
        _head_nod_down_gp_cmd += "HT" + SC_SET_GP + str(_my_head_nod_down_ht)
        if _pub_ipt:    _head_nod_down_gp_cmd += SC_SET_IPT + str( int(_duration_nod_down + 0.5) )
        _head_nod_down_gp_cmd += TERM_CHAR_SEND

        _duration_nod_center = float(_duration_nod) * 0.3   #0.2    #0.25
        _head_nod_center_gp_cmd = ""
        _head_nod_center_gp_cmd += "LL" + SC_SET_GP + str(self.previous_ll)
        _head_nod_center_gp_cmd += "HT" + SC_SET_GP + str(self.previous_ht)
        if _pub_ipt:    _head_nod_center_gp_cmd += SC_SET_IPT + str( int(_duration_nod_center + 0.5) )
        _head_nod_center_gp_cmd += TERM_CHAR_SEND

        _my_tuple = (("NOD UP", _duration_nod_up, _my_head_nod_up_ll, _my_head_nod_up_ht, _head_nod_up_gp_cmd), ("NOD DOWN", _duration_nod_down, _my_head_nod_down_ll, _my_head_nod_down_ht, _head_nod_down_gp_cmd), ("NOD CENTER", _duration_nod_center, self.previous_ll, self.previous_ht, _head_nod_center_gp_cmd))
        _pub_cmd = ""

        ## preset the desired goal speeds BEFORE sending the goal positions
        _pub_cmd = ""
        _pub_cmd += "LL" + SC_SET_GS + str(_gs_ll)
        _pub_cmd += "HT" + SC_SET_GS + str(_gs_ht)
        ## NOTE: pubTo_maki_command will automatically add TERM_CHAR_SEND postfix
        ## publish and give time for the command to propogate to the servo motors
        lookINSPIRE4Intro.pubTo_maki_command( self, str(_pub_cmd), cmd_prop=True )

        _duration = abs(rospy.get_time() -_start_time)
        rospy.loginfo("OVERHEAD SETUP TIME: " + str(_duration) + " seconds")

        _loop_count = 0
        _start_time = rospy.get_time()
        while (_loop_count < repetitions) and (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):
            rospy.logdebug("-------------------")

            for _print, _duration_nod, _my_ll, _my_ht, _head_nod_gp_cmd in _my_tuple:
                rospy.loginfo("====> " + str(_print))
                #lookINSPIRE4Intro.requestFeedback( self, SC_GET_PP )
                #rospy.logdebug( str(self.makiPP) )

                #if not _pub_ipt:
                #   ## Calculate goal speed base on distance and duration (in milliseconds)
                #   _distance_to_head_nod_ll = abs( self.makiPP["LL"] - _my_ll )
                #   rospy.logdebug("_distance_to_head_nod_ll=" + str(_distance_to_head_nod_ll))
                #   if (_distance_to_head_nod_ll > 0):
                #       _gs_ll = abs( self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_head_nod_ll, _duration_nod) )
                #       rospy.loginfo("_gs_ll=" + str(_gs_ll))
                #
                #   _distance_to_head_nod_ht = abs( self.makiPP["HT"] - _my_ht )
                #   rospy.logdebug("_distance_head_nod_ht=" + str(_distance_to_head_nod_ht))
                #   if (_distance_to_head_nod_ht > 0):
                #       _gs_ht = abs( self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_head_nod_ht, _duration_nod) )
                #       rospy.loginfo("_gs_ht=" + str(_gs_ht))
                #       _gs_ht = min(_gs_ht, self.HT_GS_MAX)
                #       rospy.loginfo("adjusted _gs_ht=" + str(_gs_ht))
                #
                #   ## preset the desired goal speeds BEFORE sending the goal positions
                #   _pub_cmd = ""
                #   if (_distance_to_head_nod_ll>0):    _pub_cmd += "LL" + SC_SET_GS + str(_gs_ll)
                #   if (_distance_to_head_nod_ht>0):    _pub_cmd += "HT" + SC_SET_GS + str(_gs_ht)
                #   ## NOTE: pubTo_maki_command will automatically add TERM_CHAR_SEND postfix
                #
                #   ## publish and give time for the command to propogate to the servo motors
                #   lookINSPIRE4Intro.pubTo_maki_command( self, str(_pub_cmd), cmd_prop=True )

                ## set servo control command to set goal positions
                _pub_cmd = _head_nod_gp_cmd

                _start_time_head_nod = rospy.get_time()
                try:
                    if _monitor and (_duration_nod >= 0):
                        ## NOTE: publish and give time for the command to propogate to the servo motors,
                        ## but DO NOT MONITOR (excess overhead of minimum 200ms, which is greater
                        ## than some _duration_nod and will cause delay)
                        lookINSPIRE4Intro.monitorMoveToGP( self, _pub_cmd, ll_gp=_my_ll, ht_gp=_my_ht )
                    else:
                        lookINSPIRE4Intro.pubTo_maki_command( self, _pub_cmd )  ## default 100ms to propogate
                        if _duration_nod > 100:
                            self.SWW_WI.sleepWhileWaitingMS( _duration_nod - 100, end_early=False)
                except rospy.exceptions.ROSException as e1:
                    rospy.logerr( str(e1) )
                _duration = abs(_start_time_head_nod - rospy.get_time())
                rospy.loginfo( str(_print) + "\tDURATION: " + str(_duration) + " seconds; EXPECTED " + str(_duration_nod))
            #end    for _print, _duration_nod, _my_ll, _my_ht, _head_nod_gp_cmd in _my_tuple:

            _loop_count = _loop_count +1

            ## debugging
            #rospy.loginfo(".............P A U S E ...")
            #self.SWW_WI.sleepWhileWaiting( 1 )     ## 1 second
        # end   while not rospy.is_shutdown():

        _duration = abs(rospy.get_time() - _start_time)
        rospy.logdebug( "NUMBER OF GREETING MOVMENTS: " + str(_loop_count) )
        rospy.logdebug( "Duration: " + str(_duration) + " seconds" )
        return


    def startStartle( self, relax=False ):
        rospy.logdebug("startStartle(): BEGIN")
        ### call base class' start function
        #eyelidHeadTiltBaseBehavior.start(self)
        #rospy.logdebug("startStartle(): After eyelidHeadTiltBaseBehavior.start()")
        lookINSPIRE4Intro.macroStartleRelax( self, startle=True, relax=relax )
        rospy.logdebug("startStartle(): END")


    ## similar to the engagment game but uses current HT and LL values
    ## startle and relax are relative to the current HT and LL values
    def macroStartleRelax( self, startle=True, relax=True, repetitions=1 ):
        rospy.logdebug("macroStartleRelax(): BEGIN")
        _start_time = rospy.get_time()
        _safe_distance_check = True

        ## check the inputs
        if (isinstance(startle, bool) and (not startle) and 
            isinstance(relax, bool) and (not relax)):   
            return

        if isinstance(repetitions, int) and (repetitions > 0):
            pass
        else:
            rospy.logwarn("macroStartleRelax(): INVALID VALUE: repetitions=" + str(repetitions) + "; updated to 1")
            repetitions = 1

        _expected_delta_ll = self.LL_STARTLE - self.LL_NEUTRAL  ## 535 - 500 = 35 ticks
        _expected_delta_ht = self.HT_STARTLE - self.HT_NEUTRAL  ## 525 - 505 = 20 ticks
        rospy.logdebug("_expected_delta_ll=" + str(_expected_delta_ll) + " ticks, _expected_delta_ht=" + str(_expected_delta_ht) + " ticks")

        if startle:
            ## Store initial pose
            lookINSPIRE4Intro.requestFeedback( self, SC_GET_PP )
            self.previous_ll = self.makiPP["LL"]
            self.previous_ht = self.makiPP["HT"] - 5    ## slightly exaggerate the relax behavior
        rospy.logdebug("self.previous_ll = " + str(self.previous_ll) + ", self.previous_ht = " + str(self.previous_ht) )

        ## generate servo control command to set goal positions
        ## NOTE: on the Arbotix-M side, a sync_write function is used
        ## to simultaneously broadcast the updated goal positions
        _my_startle_ht = self.HT_STARTLE
        _my_startle_ll = self.LL_STARTLE
        _startle_gp_cmd = ""
        if startle:
            #_startle_gp_cmd += "LL" + SC_SET_GP + str(self.LL_STARTLE)
            #_startle_gp_cmd += "HT" + SC_SET_GP + str(self.HT_STARTLE)

            ## Make adjustments for if Maki-ro's eyelid is not neutral pose
            if (self.previous_ll >= _my_startle_ll):
                _my_startle_ll = self.previous_ll + _expected_delta_ll
                rospy.logdebug("adjusted _my_startle_ll to " + str(_my_startle_ll) + " ticks")
            ## Make adjustments for if Maki-ro's head tilt is not neutral pose
            if (self.previous_ht >= _my_startle_ht):
                _my_startle_ht = self.previous_ht + _expected_delta_ht
                rospy.logdebug("adjusted _my_startle_ht to " + str(_my_startle_ht) + " ticks")
            _startle_gp_cmd += "LL" + SC_SET_GP + str(_my_startle_ll)
            _startle_gp_cmd += "HT" + SC_SET_GP + str(_my_startle_ht)
            _startle_gp_cmd += TERM_CHAR_SEND
        _relax_gp_cmd  = ""
        if relax:
            #_relax_gp_cmd += "LL" + SC_SET_GP + str(self.LL_NEUTRAL)
            #_relax_gp_cmd += "HT" + SC_SET_GP + str(self.HT_NEUTRAL)
            _relax_gp_cmd += "LL" + SC_SET_GP + str(self.previous_ll)
            _relax_gp_cmd += "HT" + SC_SET_GP + str(self.previous_ht)
            _relax_gp_cmd += TERM_CHAR_SEND
        _pub_cmd = ""

        ## NOTE: during intro, we expect Maki-ro's head to be tilted up to 
        ##  face the experimenter
        ## Realigning to neutral pose doesn't make sense here
        #
        #if relax and (not startle):
        #   rospy.loginfo("relax ONLY... skip alignment")
        #   pass
        #else:
        #   ## Move to neutral eyelid and head tilt pose
        #   rospy.loginfo("BEFORE startle, adjust LL and HT to NeutralPose")
        #   _pub_cmd = ""
        #   if (abs(self.makiPP["LL"] - self.LL_NEUTRAL) > DELTA_PP):
        #       _pub_cmd += "LLGP" + str(self.LL_NEUTRAL)
        #   if (abs(self.makiPP["HT"] - self.HT_NEUTRAL) > DELTA_PP):
        #       _pub_cmd += "HTGP" + str(self.HT_NEUTRAL) 
        #   if ( len(_pub_cmd) > 0 ):
        #       _pub_cmd += TERM_CHAR_SEND
        #       try:
        #           lookINSPIRE4Intro.monitorMoveToGP( self, _pub_cmd, ll_gp=self.LL_NEUTRAL, ht_gp=self.HT_NEUTRAL )
        #       except rospy.exceptions.ROSException as _e:
        #           rospy.logerr( str(_e) )
        #       #self.SWW_WI.sleepWhileWaiting(1)   ## 1 second     ## debugging

        _duration = abs(rospy.get_time() -_start_time)
        rospy.loginfo("OVERHEAD SETUP TIME: " + str(_duration) + " seconds")

        _loop_count = 0
        _start_time = rospy.get_time()
        while ((_loop_count < repetitions) and (self.ALIVE) and
             (not self.mTT_INTERRUPT) and (not rospy.is_shutdown())):
            rospy.logdebug("-------------------")

            ## NOTE: The startle behavior is performed in one pass,
            ##  however, the relax behavior contains an internal
            ##  while loop and requiring potentially multiple
            ##  iterations there before completing ONE pass through
            ##  this outer while loop
            ## These local variables need to be set for every
            ##  iteration through this outer level while loop

            ## TODO: unify duration_startle
            #_duration_startle = 100        ## millisecond
            _duration_relax = 1000      ## milliseconds
            _duration_relax_wait = 250
            _tmp_scale = 1.0


            ## Does not contain any inner loops
            if startle:
                rospy.loginfo("====> STARTLE")

                ## NOTE: The logic below is initially informed by the
                ##  present positions of the motors, their goal positions, 
                ##  and the duration in which to accomplish the desired
                ##  change.
                ## However, the _duration_startle is so small (100 to 150 
                ##  milliseconds). The time delay in the communications of
                ##  requesting feedback from the motors, receiving feedback 
                ##  from the motors, sending the goal 
                ##  position/speed, requesting feedback to verify that
                ##      the desired motor values have been propogated and 
                ##  updated, and receiving the feedback is greater
                ##  than the _duration_startle. Closed loop execution of the
                ##  startle behavior is too costly to use monitorMoveToGP().
                ## Instead, using pubTo_maki_command(), the servo  
                ##  commands are broadcast and given a small period of time
                ##  for the commands to be propogated to the motors. It is
                ##  assumed that the servo commands will be executed.

                ## STEP 1: get feedback of present positions
                lookINSPIRE4Intro.requestFeedback( self, SC_GET_PP )
                #rospy.logdebug( str(self.makiPP) )     ## debugging

                ## STEP 2: Calculate goal speed base on distance and duration (in milliseconds)
                ## STEP 2A: EYELID calculations
                _duration_startle = 100         ## millisecond
                _distance_to_startle_ll = abs( self.makiPP["LL"] - _my_startle_ll )
                rospy.logdebug("_distance_startle_ll=" + str(_distance_to_startle_ll))
                _tmp_duration_startle = _duration_startle

                ## If the difference is more than the allow 5 tick threshold,
                ##  calculate the proportional scaling factor from the ratio
                ##  of actual distance to the expected distance.
                ## Apply this distance based scale to the duration of the behavior
                if (abs(_distance_to_startle_ll - _expected_delta_ll) > DELTA_PP):
                    _tmp_scale = float(_distance_to_startle_ll) / float(_expected_delta_ll) 
                    _tmp_duration_startle = _tmp_scale * _tmp_duration_startle
                rospy.logdebug("_tmp_scale=" + str(_tmp_scale) + "; _tmp_duration_startle=" + str(_tmp_duration_startle))
                
                ## If there exists any distance to close,
                ##  calculate the goal speed based on the distance
                ##  (in ticks) and scaled duration (in milliseconds)
                if (_distance_to_startle_ll > 0):
                    _gs_ll = abs( self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_startle_ll, _tmp_duration_startle) )
                    rospy.loginfo("_gs_ll=" + str(_gs_ll))


                ## STEP 2B: Repeat for HEAD TILT
                _duration_startle = 150     #200    #250        ## millisecond
                _distance_to_startle_ht = abs( self.makiPP["HT"] - _my_startle_ht )
                rospy.logdebug("_distance_startle_ht=" + str(_distance_to_startle_ht))
                ## Update the duration to suit the head tilt motor
                _tmp_duration_startle = _duration_startle

                if (abs(_distance_to_startle_ht - _expected_delta_ht) > DELTA_PP):
                    _tmp_scale = float(_distance_to_startle_ht) / float(_expected_delta_ht) 
                    _tmp_duration_startle = _tmp_scale * _tmp_duration_startle
                rospy.logdebug("_tmp_scale=" + str(_tmp_scale) + "; _tmp_duration_startle=" + str(_tmp_duration_startle))

                if (_distance_to_startle_ht > 0):
                    ## Compute the goal speed for the head tilt motor
                    _gs_ht = abs( self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_startle_ht, _tmp_duration_startle) )
                    rospy.loginfo("_gs_ht=" + str(_gs_ht))
                    ## Adjust the speed to limit the head tilt motor
                    ##  from moving at higher than desirable velocity
                    _gs_ht = min(_gs_ht, self.HT_GS_MAX)
                    rospy.loginfo("adjusted _gs_ht=" + str(_gs_ht))


                ## STEP 3: Preset the desired goal speeds BEFORE sending the goal positions
                _pub_cmd = ""
                if (_distance_to_startle_ll>0):     _pub_cmd += "LL" + SC_SET_GS + str(_gs_ll)
                if (_distance_to_startle_ht>0):     _pub_cmd += "HT" + SC_SET_GS + str(_gs_ht)
                ## NOTE: pubTo_maki_command will automatically add TERM_CHAR_SEND postfix

                ## Publish and give time for the command to propogate to the servo motors
                ##  Default command propogation time is 100 ms
                lookINSPIRE4Intro.pubTo_maki_command( self, str(_pub_cmd), cmd_prop=True )

                ## STEP 4: Set servo control command to set goal positions
                ##  and send the message, calling pubTo_maki_command
                _pub_cmd = _startle_gp_cmd

                _start_time_startle = rospy.get_time()
                try:
                    ## BEHAVIOR DESCRIPTION:
                    ##      Maki-ro open eyes wide
                    ##      and INTENTIONALLY "jerks" head back
                    lookINSPIRE4Intro.pubTo_maki_command( self, _pub_cmd )  ## default 100ms to propogate

                    ## NOTE: Publish and give time for the command to propogate to the servo motors,
                    ##      but DO NOT MONITOR (excess overhead of minimum 200ms, which is greater
                    ##      than _duration_startle and will cause delay, thus blocking
                    ##  any other behaviors)
                    #lookINSPIRE4Intro.monitorMoveToGP( self, _pub_cmd, ll_gp=self.LL_STARTLE, ht_gp=self.HT_STARTLE)

                    ## STEP 5: Wait while the behavior is performed.
                    ##      Since the startle behavior is not monitored, it is
                    ##  necessary to provide sufficient time to execute.
                    ##  Subsequent servo commands published would override
                    ##  by replacing the value set on the motors themselves

                    ## If the scaled duration is greater than the command
                    ##  propogation delay (100ms), sleep for the difference
                    if _tmp_duration_startle > 100:
                        self.SWW_WI.sleepWhileWaitingMS( _tmp_duration_startle - 100, end_early=False)
                except rospy.exceptions.ROSException as e1:
                    rospy.logerr( str(e1) )

                ## STEP 6: Calculate the elapsed duration of the behavior
                _duration = abs(_start_time_startle - rospy.get_time())
                rospy.logwarn( "Startle duration: Expected: " + str(_tmp_duration_startle) + " milliseconds, Actual: " + str(_duration) + " seconds" )

                ## STEP 7: Set __is_startled state to True
                ##  WITHOUT VERIFYING THE ROBOT'S RESULTING POSITION
                lookINSPIRE4Intro.__is_startled = True

                rospy.loginfo("Done: STARTLE ====")
            #end    if startle:



            ## NOTE: Contains its own while loop within
            ##  and used in a manner similar to
            ##  monitorMoveToGP()
            ## The while loop is used to iteratively update
            ##  and set the goal speeds for the eyelid
            ##  and head tilt motors based on duration
            ##  remaining and distance remaining until
            ##  either1)  the goal position has been obtained 
            ##  or 2) the duration is exceeded
            if relax:
                rospy.loginfo("====> RELAX")
                #rospy.loginfo( str(self.makiPP) )  ## debugging
                _first_pass = True
                _start_time_relax = rospy.get_time()

                ## NOTE: This is our custom version of monitorMoveToGP
                ##      adjusts speed based on difference
                ##      between current position and goal
                ##      position to stay within _duration_relax
                ## Like monitorMoveToGP, this nested while loop is BLOCKING
                _relax_loop_count = 0
                while relax and (not rospy.is_shutdown()):
                    ##  This while loop's comparators are such that
                    ##  it cant only be terminated by 1) SHUTTING
                    ##  DOWN the rosnode, or 2) encountering a
                    ##  condition whose body includes a BREAK
                    ##  statement

                    _relax_loop_count += 1

                    ## STEP 1: get present positions for 
                    ##  executing this CLOSED LOOP behavior
                    lookINSPIRE4Intro.requestFeedback( self, SC_GET_PP )
                    rospy.loginfo( str(self.makiPP) )

                    ## STEP 2: Computer difference between current and goal positions
                    ## TODO: do this calculation using map
                    #_distance_to_relax_ll = abs( self.makiPP["LL"] - self.LL_NEUTRAL )
                    #rospy.loginfo("_distance_to_relax_ll=" + str(_distance_to_relax_ll))
                    #_distance_to_relax_ht = abs( self.makiPP["HT"] - self.HT_NEUTRAL )
                    #rospy.loginfo("_distance_to_relax_ht=" + str(_distance_to_relax_ht))
                    _distance_to_relax_ll = abs( self.makiPP["LL"] - self.previous_ll )
                    rospy.loginfo("_distance_to_relax_ll=" + str(_distance_to_relax_ll))
                    _distance_to_relax_ht = abs( self.makiPP["HT"] - self.previous_ht )
                    rospy.loginfo("_distance_to_relax_ht=" + str(_distance_to_relax_ht))

                    ## Add some error checking
                    ## if this is first time going through the inner relax while loop
                    ##  AND the present positions for BOTH the eyelid motor and
                    ##  heat tilt motors are the same as the previous neutral position,
                    ##  it is VERY LIKELY that the motors have not achieved the
                    ##  startle pose and are still moving
                    ## So... continue
                    if (_first_pass and (_distance_to_relax_ll == 0) and (_distance_to_relax_ht == 0)): 
                        _relax_loop_count = 0
                        rospy.logdebug("(startle) Relax while loop: False start... need updated present position feedback. Continue...")
                        continue        ## jump back to the beginning of this inner while loop
                    
                    rospy.loginfo("Relax: Pass #" + str(_relax_loop_count) + " through while loop.... _distance_to_relax_ll=" + str(_distance_to_relax_ll) +", _distance_to_relax_ht=" + str(_distance_to_relax_ht) + " ticks")

                    #_tmp_duration_startle = _duration_startle
                    #if (abs(_distance_to_startle_ht - _expected_delta_ht) > DELTA_PP):
                    #   _tmp_scale = float( _distance_to_startle_ht / _expected_delta_ht )
                    #   _tmp_duration_startle = _tmp_scale * _tmp_duration_startle
                    #rospy.logdebug("_tmp_scale=" + str(_tmp_scale) + "_tmp_duration_startle=" + str(_tmp_duration_startle))

                    ## STEP 3: ADJUST DURATION (elapsed) TO STAY WITHIN _duration_relax
                    ## ALSO MONITOR REMAINING DISTANCE
                    if _first_pass:
                        rospy.logwarn(">>>>>>>>>>>>>>> _distance_to_relax_ht=" + str(_distance_to_relax_ht))
                        if startle:     rospy.logwarn(">>>>>>>>>>>>>>> _distance_to_startle_ht=" + str(_distance_to_startle_ht))
                        rospy.logdebug("duration_relax = " + str(_duration_relax))
                        #_first_pass=False
                        pass

                    elif (_distance_to_relax_ll > DELTA_PP) or (_distance_to_relax_ht > DELTA_PP):
                        ## If either goal position is greater than the threshold tolerance,
                        ##  decrement the duration by _duration_relax_wait
                        ##  milliseconds unit
                        _duration_relax = _duration_relax - _duration_relax_wait
                        rospy.logdebug("duration_relax = " + str(_duration_relax))

                    else:
                        #rospy.logdebug("close enough...done relax while loop")
                        rospy.logwarn(">>>>>>>>>>>>> BREAK!! Positioned close enough... Remaining _distance_to_relax_ll=" + str(_distance_to_relax_ll) + ", _distance_to_relax_ht=" + str(_distance_to_relax_ht) + " ticks... remaining _duration_relax is " + str(_duration_relax) +" seconds ...done relax while loop")
                        break

                    ## SEEPARATELY CHECK TO SEE IF _duration_relax HAS BEEN EXCEEDED
                    if (_duration_relax <= 0):
                        #rospy.logdebug("negative time...done relax while loop")
                        rospy.logwarn(">>>>>>>>>>> BREAK!!! Exceeded _duration_relax; negative time ( " + str(_duration_relax) + " seconds )... Remaining _distance_to_relax_ll=" + str(_distance_to_relax_ll) + ", _distance_to_relax_ht=" + str(_distance_to_relax_ht) + " ticks ...done relax while loop")
                        break

                    ## STEP 4: Calculate new goal speeds
                    ## STEP 4A: EYELID
                    if (_distance_to_relax_ll > 0):
                        _gs_ll = self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_relax_ll, _duration_relax)
                        rospy.loginfo("_gs_ll=" + str(_gs_ll))
                        _gs_ll = max(_gs_ll, self.LL_GS_MIN)
                        rospy.loginfo("adjusted _gs_ll=" + str(_gs_ll))

                    elif _safe_distance_check:
                        _gs_ll = self.LL_GS_MIN

                    else:
                        ## BE CAREFUL!!! _gs_ll could be 0
                        pass
                    #end    if (_distance_to_relax_ll > 0):

                    ## STEP 4B: HEAD TILT
                    if (_distance_to_relax_ht > 0):
                        _gs_ht = self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_relax_ht, _duration_relax)
                        rospy.loginfo("_gs_ht=" + str(_gs_ht))
                        if _first_pass:
                            if (_gs_ht < self.HT_GS_MIN):
                                _duration_relax_min_gs = self.DC_helper.getTurnDurationMS_ticks_goalSpeed( _distance_to_relax_ht, self.HT_GS_MIN )
                                rospy.logwarn( "(startle) Relax behavior is LIKELY to finish before the duration specified ( " + str(_duration_relax) + " ms )... Calculated _gs_ht ( " + str(_gs_ht) + " ) BELOW the minimum limit ( " + str(self.HT_GS_MIN) + " )... Recommendation for DECREASING THE DURATION to\t" + str(_duration_relax_min_gs) + " milliseconds" )
                            if (_gs_ht > self.HT_GS_MAX):
                                _duration_relax_max_gs = self.DC_helper.getTurnDurationMS_ticks_goalSpeed( _distance_to_relax_ht, self.HT_GS_MAX )
                                rospy.logwarn( "(startle) Relax behavior is UNLIKELY to finish in the duration specified ( " + str(_duration_relax) + " ms )... Calculated _gs_ht ( " + str(_gs_ht) + " ) EXCEEDS the maximum limit ( " + str(self.HT_GS_MAX) + " )... Recommendation for INCREASING THE DURATION to\t" + str(_duration_relax_max_gs) + " milliseconds" ) 

                        ## Adjust the speed to limit the head tilt motor
                        ##  from moving at higher than desirable velocity
                        ##  since too HIGH TORQUE
                        _gs_ht = min(_gs_ht, self.HT_GS_MAX)
                        rospy.loginfo("adjusted _gs_ht=" + str(_gs_ht))
                        ## Similarly adjust from moving at slower than desirable velocity
                        ##  since likely to STALLL
                        _gs_ht = max(_gs_ht, self.HT_GS_MIN)
                        rospy.loginfo("adjusted _gs_ht=" + str(_gs_ht))
                        #_duration_relax_wait = self.DC_helper.getTurnDurationMS_ticks_goalSpeed( _distance_to_relax, _gs_ll )
                        #rospy.loginfo("waitMS = " + str(_duration_relax_wait))

                    elif _safe_distance_check:
                        _gs_ht = self.HT_GS_MIN

                    else:
                        ## BE CAREFUL!!! _gs_ht could be 0
                        pass
                    #end    if (_distance_to_relax_ht > 0):
            

                    ## STEP 5: Generate servo control command to set new goal speeds
                    ##  BEFORE sending goal positions
                    ## At the very least, set new goal speeds of self.LL_GS_MIN and self.HT_GS_MIN
                    _pub_cmd = ""
                    if _safe_distance_check:
                        ## Only build _pub_cmd if the distance is more that 0
                        ##      and if _gs_* is greater than 0 (unlimited speed)
                        if (_distance_to_relax_ll>0):   _pub_cmd += "LL" + SC_SET_GS + str(_gs_ll)
                        if (_distance_to_relax_ht>0):   _pub_cmd += "HT" + SC_SET_GS + str(_gs_ht)
                    else:
                        _pub_cmd += "LL" + SC_SET_GS + str(_gs_ll)
                        _pub_cmd += "HT" + SC_SET_GS + str(_gs_ht)
                        _pub_cmd += TERM_CHAR_SEND
                    lookINSPIRE4Intro.pubTo_maki_command( self, str(_pub_cmd) )     ## 100 ms command propogation delay

                    ## On the first time through the whole relax loop, do
                    ##  a couple of housekeeping things: 1) start a timer,
                    ##  2) publish the new goal speeds again
                    if _first_pass:     
                        #lookINSPIRE4Intro.pubTo_maki_command( self, str(_pub_cmd), time_ms=50 )    
                        _start_time_relax = rospy.get_time()
                        
                        ## Formulate special mixed goal speed and goal position servo 
                        ##  command. goal speeds should be set as they are parsed,
                        ##  and goal positions are queued until TERM_CHAR_SEND
                        ##  is encountered
                        _pub_cmd = ""
                        _pub_cmd += "LL" + SC_SET_GS + str(_gs_ll)
                        _pub_cmd += "HT" + SC_SET_GS + str(_gs_ht)
                        _pub_cmd += _relax_gp_cmd
                        rospy.logwarn("Secret sauce is... " + _pub_cmd )
                        lookINSPIRE4Intro.pubTo_maki_command( self, str(_pub_cmd) )     

                        _first_pass = False


                    ## STEP 6: Publish servo control command to set goal positions
                    ##  The goal positions will be published EVERY pass through
                    ##  the inner relax while loop
                    _pub_cmd = _relax_gp_cmd
        
                    try:
                        ## BEHAVIOR DESCRIPTION:
                        ##      Maki-ro relaxes wide open eyes
                        ##      and head comes back forward to neutral
                        lookINSPIRE4Intro.pubTo_maki_command( self, _pub_cmd )  ## 100 ms command propogation delay

                        ## sleep for wait increment
                        self.SWW_WI.sleepWhileWaitingMS( _duration_relax_wait, end_early=False)
                    except rospy.exceptions.ROSException as e2:
                        rospy.logerr( "macroStartleRelax(): relax: ERROR: " + str(e2) )
                #end    while relax and (not rospy.is_shutdown()):

                _duration = abs(_start_time_relax - rospy.get_time())
                #rospy.logwarn( "Relax duration: " + str(_duration) + " seconds" )
                rospy.logwarn( ">>>>>>>>>>>> (startle) Relax duration: Expected: " + str( _relax_loop_count * _duration_relax_wait ) + " milliseconds, Actual: " + str(_duration) + " seconds ( " + str(_relax_loop_count) + " passes )" )
                rospy.loginfo( str(self.makiPP) )

                ## STEP 7: unset __is_startled state
                ##  Regardless of how the relax inner while
                ##  loop was broken, Maki-ro is no longer
                ##  positioned in the startle expression.
                ##  Either Maki-ro 1) successfully performed
                ##  the relax behavior (present and goal
                ##  positions are within tolerance), or
                ##  2) began and completed a portion of the 
                ##  relax behavior but exceeded the
                ##  specified duration
                lookINSPIRE4Intro.__is_startled = False

                ## STEP 8: Insurance to make Maki-ro's eyelids reset to neutral
                lookINSPIRE4Intro.setEyelidNeutralPose( self, self.previous_ll, cmd_prop=True )
                rospy.loginfo("Done: RELAX ====")
            #end    if relax:

            _loop_count = _loop_count +1

            ## debugging
            #rospy.loginfo(".............P A U S E ...")
            #self.SWW_WI.sleepWhileWaiting( 1 )     ## 1 second
        # end   while not rospy.is_shutdown():

        _duration = abs(rospy.get_time() - _start_time)
        rospy.logdebug( "NUMBER OF STARTLE/RELAX MOVMENTS: " + str(_loop_count) )
        rospy.logdebug( "Total Duration: " + str(_duration) + " seconds" )
        return


    def startStartle( self, relax=False ):
        rospy.logdebug("startStartle(): BEGIN")
        ### call base class' start function
        #eyelidHeadTiltBaseBehavior.start(self)
        #rospy.logdebug("startStartle(): After eyelidHeadTiltBaseBehavior.start()")
        lookINSPIRE4Intro.macroStartleRelax( self, startle=True, relax=relax )
        rospy.logdebug("startStartle(): END")
        return

    def stopStartle( self, disable_ht=True ):
        ## shift into eyelid and headtilt neutral
        lookINSPIRE4Intro.macroStartleRelax( self, startle=False, relax=True )

        lookINSPIRE4Intro.stop(self, disable_ht=disable_ht )
        return

    def introStart( self, enable_ht=True ):
        ## try to nicely startup without jerking MAKI's head tilt servo
        lookINSPIRE4Intro.start(self, enable_ht=enable_ht)
        lookINSPIRE4Intro.__is_intro_running = True
        return

    def introStop( self, disable_ht=True ):
        rospy.logdebug("introStop(): BEGIN")
        lookINSPIRE4Intro.__is_intro_running = False
        lookINSPIRE4Intro.stop(self, disable_ht=disable_ht)
        rospy.logdebug("introStop(): END")
        return


    def parse_maki_macro( self, msg ):
        print msg.data

        if msg.data == "intro start":
            lookINSPIRE4Intro.introStart( self )

        elif msg.data.startswith( "intro stop"):
            if msg.data.endswith( "disable_ht=False" ):
                lookINSPIRE4Intro.introStop( self, disable_ht=False )
                ## TODO: May need to change to False when whole INSPIRE4
                ##  script and control program are in place
            else:
                lookINSPIRE4Intro.introStop( self, disable_ht=True )

        elif msg.data == "intro greet":
            lookINSPIRE4Intro.macroGreeting( self )

        elif msg.data == "intro startle":
            ## perform one startle then immediately relax
            lookINSPIRE4Intro.startStartle( self, relax=True )

        elif msg.data == "intro startle start":
            ## perform one startle and hold
            lookINSPIRE4Intro.startStartle( self, relax=False )

        elif msg.data.startswith( "intro startle stop" ):
            ## from startle, immediately relax
            if msg.data.endswith( "disable_ht=False" ):
                lookINSPIRE4Intro.stopStartle( self, disable_ht=False )
            else:
                lookINSPIRE4Intro.stopStartle( self, disable_ht=True )

        elif msg.data == "intro lookAtExperimenter":
            lookINSPIRE4Intro.lookAt_jointHeadAndEyePan( self, self.pub_cmd_look_fromInfant_toExperimenter )
            #lookINSPIRE4Intro.macroLookAtExperimenter( self )

        elif msg.data == "intro lookAtBallLocationUpperRight":
            lookINSPIRE4Intro.lookAt_jointHeadAndEyePan( self, self.pub_cmd_look_fromExperimenter_toBallUpperRight )

        elif msg.data == "intro lookAtBallLocationLowerRight":
            lookINSPIRE4Intro.lookAt_jointHeadAndEyePan( self, self.pub_cmd_look_fromBallUpperRight_toBallLowerRight )

        elif msg.data == "intro lookAtInfant":
            lookINSPIRE4Intro.lookAt_jointHeadAndEyePan( self, self.pub_cmd_look_fromBallLowerRight_toInfant )
            ## TODO: Maki-ro doesn't quite return to HT_MIDDLE
            lookINSPIRE4Intro.pubTo_maki_command( self, "reset" )

        ## TODO: Remove since no longer exists
        elif msg.data == "lookNeutral":
            #self.macroLookNeutral()
            pass

        else:
            pass

        return

if __name__ == '__main__':
        print "__main__: BEGIN"
        lookIntro = lookINSPIRE4Intro( True, None )

        rospy.Subscriber( "/maki_macro", String, lookIntro.parse_maki_macro )
        rospy.logdebug( "now subscribed to /maki_macro" )

        rospy.spin()   ## keeps python from exiting until this node is stopped

        print "__main__: END"





