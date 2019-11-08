#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from base_behavior import *     ## classes baseBehavior and headTiltBaseBehavior


########################
## Maki-ro's "lookAt" head / eye pan coordination
##
## Description:
##  Maki-ro's eye pan will move rapidly towards "gaze target"
##  Then as head pan moves and comes into alignment with "gaze
##  target", eye pan will counter rotate
##
## TODO: Add corresponding head / eye tilt
##
## Thanks much to Ale!
########################
class lookAt( headTiltBaseBehavior, headPanBaseBehavior ):
    ## variables private to this class
    ## all instances of this class share the same value
    EP_MIN_LEFT = 460
    EP_MAX_RIGHT = 578

    def __init__(self, verbose_debug, ros_pub):
        ## call base class' __init__
        headTiltBaseBehavior.__init__( self, verbose_debug, ros_pub )
        headPanBaseBehavior.__init__( self, verbose_debug, self.ros_pub )
        ## add anything else needed by an instance of this subclass
        self.DC_helper = dynamixelConversions()

        ## DEFAULT MOTOR SPEEDS
        self.LL_GS_DEFAULT = 100
        self.EP_GS_DEFAULT = 100
        self.ET_GS_DEFAULT = 200
        self.HP_GS_DEFAULT = 15
        self.HT_GS_DEFAULT = 51

        self.ALIVE = True
        return

    ## override base class
    def pubTo_maki_command( self, commandOut, fixed_gaze=True, cmd_prop=True, time_ms=100, time_inc=0.5):
        rospy.logdebug("lookINSPIRE4Interaction.pubTo_maki_command(): BEGIN")
        ## call base class' pubTo_maki_command
        headPanBaseBehavior.pubTo_maki_command( self, commandOut, fixed_gaze=fixed_gaze, cmd_prop=cmd_prop, time_ms=time_ms, time_inc=time_inc )
        rospy.logdebug("lookINSPIRE4Interaction.pubTo_maki_command(): END")
        return

    ## override base class
    def start( self, enable_ht=True ):
        ## call base class' start
        headTiltBaseBehavior.start( self, enable_ht=enable_ht )
        return

    ## override base class
    def stop( self, disable_ht=True ):
        ## call base class' stop
        return headTiltBaseBehavior.stop( self, disable_ht=disable_ht )

    ## NOTE: Currently any head tilt command is addressed like head pan:
    ##  head tilt single movement from present position to goal postion
    def shiftGazeVelocity( self, hp_gp=None, ep_gp_shift=None, ep_gp_fixed=None, ht_gp=None, hp_pp=None, ep_pp=None, ht_pp=None, duration_s=1.0, padding=0.1 ):
        rospy.loginfo("shiftGazeVelocity(): INITIAL INPUT: hp_gp=" + str(hp_gp) + ", ep_gp_shift=" + str(ep_gp_shift) + ", ep_gp_fixed=" + str(ep_gp_fixed) + ", ht_gp=" + str(ht_gp) + ", hp_pp=" + str(hp_pp) + ", ep_pp=" + str(ep_pp) + ", ht_pp=" + str(ht_pp) + ", duration_s=" + str(duration_s))

        ## check validity of inputs
        if ((hp_gp == None) or (not isinstance(hp_gp, int))):
            rospy.logerr("shiftGazeVelocity(): INVALID INPUT: hp_gp = " + str(hp_gp) + "; expected int")
            return None
        if (ep_gp_fixed == None):
            rospy.logwarn("shiftGazeVelocity(): INPUT NOT SPECIFIED: ep_gp_fixed = " + str(ep_gp_fixed) + "; auto correcting to neutral")
            ep_gp_fixed = EP_FRONT
        if (ht_gp == None):
            rospy.logwarn("shiftGazeVelocity(): INPUT NOT SPECIFIED: ht_gp = " + str(ht_gp) + "; auto correcting to neutral")
            ht_gp = HT_MIDDLE
            
        if ((hp_pp == None) or (not isinstance(hp_pp, int)) or
            (ep_pp == None) or (not isinstance(ep_pp, int)) or
            (ht_pp == None) or (not isinstance(ht_pp, int))):
            ## get most recent values
            lookAt.requestFeedback( self, SC_GET_PP )
        ## if present position is not specified, use the feedback values
        if ((hp_pp == None) or (not isinstance(hp_pp, int))):   hp_pp = self.makiPP["HP"]
        if ((ep_pp == None) or (not isinstance(ep_pp, int))):   ep_pp = self.makiPP["EP"]
        if ((ht_pp == None) or (not isinstance(ht_pp, int))):   ht_pp = self.makiPP["HT"]

        if ((ep_gp_shift == None) or (not isinstance(ep_gp_shift, int))):
            rospy.logwarn("shiftGazeVelocity(): INPUT NOT SPECIFIED: ep_gp_shift = " + str(ep_gp_shift) + "; inferring from head pan present and goal positions")
            if (hp_gp < hp_pp):     ## turning to LEFT
                _ep_gp_shift = lookAt.EP_MIN_LEFT
            else:   ## turning to RIGHT
                _ep_gp_shift = lookAt.EP_MAX_RIGHT
        else:
            _ep_gp_shift = ep_gp_shift

        rospy.loginfo("shiftGazeVelocity(): ADJUSTED INPUT: hp_gp=" + str(hp_gp) + ", ep_gp_shift=" + str(_ep_gp_shift) + ", ep_gp_fixed=" + str(ep_gp_fixed) + ", ht_gp=" + str(ht_gp) + ", hp_pp=" + str(hp_pp) + ", ep_pp=" + str(ep_pp) + ", ht_pp=" + str(ht_pp) + ", duration_s=" + str(duration_s))

        ## STEP 0: calculate _delta_hp_pp
        _delta_hp_pp = abs( hp_pp - hp_gp )
        ##  calculate _delta_ht_pp
        _delta_ht_pp = abs( ht_pp - ht_gp )         
        rospy.loginfo("STEP 0: _delta_hp_pp=" + str(_delta_hp_pp) + ", _delta_ht_pp=" + str(_delta_ht_pp))

        ## STEP 1: calculate HP degrees
        _hp_degrees = self.DC_helper.convertToDegrees_ticks( _delta_hp_pp )
        rospy.loginfo("STEP 1: _hp_degrees=" + str(_hp_degrees))

        ## STEP 2: calculate EP degrees
        _delta_ep_pp_shift = abs( ep_pp - _ep_gp_shift )
        #print _delta_ep_pp_shift
        _ep_shift_degrees = self.DC_helper.convertToDegrees_ticks( _delta_ep_pp_shift )
        rospy.loginfo("STEP 2: _delta_ep_pp_shift=" + str(_delta_ep_pp_shift) + ", _ep_shift_degrees=" + str(_ep_shift_degrees))

        ## STEP 3: subtract EP degrees
        _counter_angle = abs( _hp_degrees - _ep_shift_degrees )
        rospy.loginfo("STEP 3: _counter_angle=" + str(_counter_angle))

        ## STEP 4: convert to duration 
        ##  AMOUNT OF TIME TO ELAPSE BEFORE COUNTERING EYE PAN
        _counter_ticks = self.DC_helper.convertToTicks_degrees( _counter_angle )
        if (_delta_hp_pp > 20):
                _counter_ratio = float(_counter_ticks) / float(_delta_hp_pp)
        else:
                rospy.logerr("_delta_hp_pp == 0; cannot divide by it!")
                _counter_ratio = .4 # a pretty neutral ratio if you think about it
        _counter_duration = _counter_ratio * duration_s     ## in seconds
        rospy.loginfo("STEP 4: _counter_ticks=" + str(_counter_ticks) + ", _counter_ratio=" + str(_counter_ratio) + ",  _counter_duration=" + str(_counter_duration))

        ## STEP 5A: calculate HP goal speed
## KATE
        if (_delta_hp_pp > 10):
            _hp_gs = self.DC_helper.getGoalSpeed_ticks_duration( _delta_hp_pp, duration_s )
            if _hp_gs < 10.0:
                _hp_gs = self.HP_GS_DEFAULT
        else:
            ## JAKE CHANGE. NOT SURE WHAT THIS DOES
            _hp_gs = self.HP_GS_DEFAULT
            _hp_gp = hp_pp
        rospy.loginfo( "STEP 5A: _delta_hp_pp = " + str(_delta_hp_pp) + "; _hp_gs = " + str(_hp_gs))

        ## STEP 5B: calculate HT goal speed
        if (_delta_ht_pp > 0):
            _ht_gs = self.DC_helper.getGoalSpeed_ticks_duration( _delta_ht_pp, duration_s )
        else:
            _ht_gs = self.HT_GS_DEFAULT = 51
        rospy.loginfo( "STEP 5B: _delta_ht_pp = " + str(_delta_ht_pp) + "; _ht_gs = " + str(_ht_gs))

        ## STEP 6: calculate EP goal speed for counter rotation
        ##  2*_ep_gs is a good heuristic for shift movement
        _ep_gs = self.DC_helper.getGoalSpeed_ticks_duration( abs(ep_gp_fixed - _ep_gp_shift), (duration_s - _counter_duration) )
        #_ep_gs_shift = 2 * _ep_gs
        _ep_gs_shift = self.DC_helper.getGoalSpeed_ticks_duration( _delta_ep_pp_shift, _counter_duration )
        rospy.loginfo("STEP 6: _ep_gs=" + str(_ep_gs) + ", _ep_gs_shift=" + str(_ep_gs_shift))


        ## STEP 7: print
        rospy.loginfo("t(0): hp_gp=" + str(hp_gp) + ", _hp_gs=" + str(_hp_gs) + "; ep_gp=" + str(_ep_gp_shift) + ", _ep_gs=" + str( _ep_gs_shift ))
        rospy.loginfo("t(" + str(_counter_duration) + "): _ep_gp=" + str(ep_gp_fixed) + ", _ep_gs=" + str(_ep_gs))
        rospy.loginfo("t(" + str(duration_s) + "): DONE")
    

        ## NOW ACTUATE THE MOVEMENT
        ## send the goal speeds first
        #lookAt.pubTo_maki_command( "HPGS" + str(_hp_gs) + "EPGS" + str(_ep_gs_shift) + TERM_CHAR_SEND, fixed_gaze=False )
        _pub_cmd = ""
        if (_delta_hp_pp > 0):  _pub_cmd += "HPGS" + str(_hp_gs) + "EPGS" + str(_ep_gs_shift) 
        if (_delta_ht_pp > 0):  _pub_cmd += "HTGS" + str(_ht_gs)
        _pub_cmd += TERM_CHAR_SEND 
        rospy.loginfo("shiftGazeVelocity(): send initial goal speeds: " + str(_pub_cmd))
        lookAt.pubTo_maki_command( self, _pub_cmd, fixed_gaze=False )
        _start_time = rospy.get_time()

        ## next send the goal positions
        #lookAt.pubTo_maki_command( "HPGP" + str(hp_gp) + "EPGP" + str(_ep_gp_shift) + TERM_CHAR_SEND, fixed_gaze=False )
        _pub_cmd = ""
        if (_delta_hp_pp > 0):  _pub_cmd += "HPGP" + str(hp_gp) + "EPGP" + str(_ep_gp_shift) 
        if (_delta_ht_pp > 0):  _pub_cmd += "HTGP" + str(ht_gp)
        _pub_cmd += TERM_CHAR_SEND
        rospy.loginfo("shiftGazeVelocity(): send initial goal positions: " + str(_pub_cmd))
        lookAt.pubTo_maki_command( self, _pub_cmd, fixed_gaze=False )

        ## wait until counter duration
        #_sleep_duration = _counter_duration - 0.1      ## compensate for the command propogation delay         ## doesn't make EP to far side
## KATE
        #_sleep_duration = _counter_duration    ## need to sleep for full value of _counter_duration    ## doesn't stop with head pan
        #_sleep_duration = _counter_duration - 0.05
        _sleep_duration = _counter_duration - padding
        rospy.sleep( _sleep_duration )
        rospy.loginfo("shiftGazeVelocity(): counter!!! _sleep_duration=" + str(_sleep_duration))
        #lookAt.pubTo_maki_command( self, "EPGS" + str(_ep_gs) + "EPGP" + str(ep_gp_fixed) + TERM_CHAR_SEND, fixed_gaze=False )
        _pub_cmd = ""
        _pub_cmd += "EPGS" + str(_ep_gs) + "EPGP" + str(ep_gp_fixed) 
        _pub_cmd += TERM_CHAR_SEND
        rospy.loginfo("shiftGazeVelocity(): send counter message: " + str(_pub_cmd))
        lookAt.pubTo_maki_command( self, _pub_cmd, fixed_gaze=False )
        ## wait until duration_s
        _sleep_duration = duration_s - _sleep_duration - 0.1    ## compensate for the command propogation delay
        if (_sleep_duration > 0):   
            rospy.sleep( _sleep_duration )
        rospy.loginfo("shiftGazeVelocity(): finish!! _sleep_duration=" + str(_sleep_duration))

        _duration = abs(rospy.get_time() - _start_time)
        rospy.loginfo( "Duration: Expected = " + str(duration_s) + " seconds; elapsed = " + str(_duration) + " seconds" )
        return



