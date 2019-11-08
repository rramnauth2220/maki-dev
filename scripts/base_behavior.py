#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import sys
import string
import thread
import re       # see http://stackoverflow.com/questions/5749195/how-can-i-split-and-parse-a-string-in-python


from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions

from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt


########################
## All behavior macros will use this as base class
########################
class baseBehavior(object):
    ## all instances of this class share the same value
    ## variables private to this class
    __maki_cmd_msg_format = None
    __maki_feedback_format = None


    def __init__(self, verbose_debug, ros_pub):
        self.count_movements = 0
        self.ALIVE = True
        #self.mTT_INTERRUPT = True
## 2016-06-16, KATE
        self.mTT_INTERRUPT = False
        self.VERBOSE_DEBUG = verbose_debug	## default is False
        self.SWW_WI = ROS_sleepWhileWaiting_withInterrupt()
        self.DC_helper = dynamixelConversions()

        ## Does Maki-ro remain in end position (shift=True)
        ## or revert to ground position (shift=False)
        self.shift = False  ## default is False

        #print ros_pub
        if ros_pub == None:
            self.initROS( self )
        else:
            self.ros_pub = ros_pub		## can we pass a ros publisher??? Apparently so!
        if baseBehavior.__maki_cmd_msg_format == None:
            baseBehavior.initPubMAKIFormat( self )
            #rospy.logdebug( str(baseBehavior.__maki_cmd_msg_format) )
        if baseBehavior.__maki_feedback_format == None:
            baseBehavior.initSubMAKIFormat( self )
        self.initPubMAKI()
        self.initSubMAKIFeedback()
        self.makiPP = None
        self.maki_feedback_values = {}  ## empty dictionary

    def start(self, makiPP=None):
        _invalid_entry = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )

        ## request a feedback message
        baseBehavior.requestFeedback( self, str(SC_GET_PP) )

        ## check to see if there is an entry with key "PP"
        #while (not rospy.is_shutdown() and self.mTT_INTERRUPT):
## 1026-06-12, KATE
        ## while ros is running and no interruptions have occurred
        while (not rospy.is_shutdown() and not self.mTT_INTERRUPT):
            ## LEGACY CHECK
            ## if we were passed a valid makiPP 
            if (makiPP != None) and (makiPP != _invalid_entry):
                self.makiPP = makiPP
                break   ## break the while loop

            ## if we got a message on /maki_feedback_pres_pos
            if (self.makiPP != None) and (self.makiPP != _invalid_entry):
                break   ## break the while loop
            #else:
            #   rospy.loginfo("Waiting for a message on /maki_feedback_pres_pos...")
            #   ## request a feedback message
            #   baseBehavior.requestFeedback( self, str(SC_GET_PP) )

            #self.SWW_WI.sleepWhileWaitingMS( 250, end_early=False )    ## 0.25 second

            rospy.loginfo("Waiting for a message on /maki_feedback_pres_pos...")
            baseBehavior.requestFeedback( self, str(SC_GET_PP), time_ms=500 )
        #end    while (not rospy.is_shutdown() and self.mTT_INTERRUPT):

## 2016-06-16, KATE
        #self.ALIVE = True
        self.mTT_INTERRUPT = False
        return

    #######################
    # stop at a planned break point
    #######################
    def stop(self):
        self.mTT_INTERRUPT = True

    #######################
    # stop immediately
    #######################
    def abort(self):
        self.ALIVE = False
        self.mTT_INTERRUPT = True

    def update( self, makiPP ):
        self.makiPP = makiPP

    def requestFeedback( self, feedback_type, cmd_prop=True, time_ms=100, time_inc=0.01 ):
        ## check /maki_feedback_*
        ## request a feedback message
        #baseBehavior.pubTo_maki_command( self, str(SC_FEEDBACK) + str(feedback_type) + str(TERM_CHAR_SEND), cmd_prop, time_ms, time_inc )
        baseBehavior.pubTo_maki_command( self, str(SC_FEEDBACK) + str(feedback_type) + str(TERM_CHAR_SEND), cmd_prop=cmd_prop, time_ms=time_ms, time_inc=time_inc )
        return

    #####################
    ## THESE ARE COMMON FOR ALL BEHAVIORS
    #####################
    def pubTo_maki_command( self, commandOut, cmd_prop=True, time_ms=100, time_inc=0.01 ):
        _pub_flag = False

        ## make sure that commandOut ends in only one TERM_CHAR_SEND
        _tmp = re.search( baseBehavior.__maki_cmd_msg_format, commandOut )
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
            # if self.VERBOSE_DEBUG:       rospy.logdebug( str(commandOut) + " added TERM_CHAR_SEND" )
            rospy.logdebug( str(commandOut) + " added TERM_CHAR_SEND" )
        else:
            rospy.logerr( "Incorrect message format" + str(commandOut) )

        # if self.VERBOSE_DEBUG: rospy.logdebug( str(commandOut) )
        rospy.logdebug( str(commandOut) )

        if _pub_flag and not rospy.is_shutdown():
            self.ros_pub.publish( commandOut )
            if (cmd_prop):  self.SWW_WI.sleepWhileWaitingMS( time_ms, time_inc, end_early=False )   ## make sure command propogates
        return


    #####################
    ## Initialize ROS node 
    #####################
    def initROS( self, nodename="anon" ):
        ## get function name for logging purposes
        _fname = sys._getframe().f_code.co_name        ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
        print str(_fname) + ": BEGIN"   ## THIS IS BEFORE ROSNODE INIT

        _anon_rosnode = False
        if nodename == "anon":
            _anon_rosnode = True
        else:
            ## Parse nodename
            ## Examples passed: <__main__.headNod object at 0x7f24b3e07150>
            ## <__main__.lookAlissa object at 0xb6c91bac>
            #print str(nodename)
            _tmp_nodename = str(nodename)
            _object_name_format = "^\<\_\_main\_\_\.(\w+)"
            #print _object_name_format
            _tmp = re.search( _object_name_format, _tmp_nodename )
            if _tmp != None:
                nodename = _tmp.group(1)
                #print nodename
            else:
                ## Actually... this is already an initialized rosnode, so quick, exit!
                self.ros_pub = nodename
                #print nodename
                return

            # see http://wiki.ros.org/rospy/Overview/Logging
            # if self.VERBOSE_DEBUG:
            #   self.ros_pub = rospy.init_node(str(nodename), anonymous=_anon_rosnode, log_level=rospy.DEBUG)
            #   rospy.logdebug("log_level=rospy.DEBUG")
            # else:
            self.ros_pub = rospy.init_node(nodename, anonymous=_anon_rosnode)       ## defaults to log_level=rospy.INFO
            
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

        rospy.logdebug( str(_fname) + ": END")
        return

    #####################
    ## Set up regex format for publishing on /maki_command
    #####################
    def initPubMAKIFormat(self):
        ## get function name for logging purposes
        _fname = sys._getframe().f_code.co_name        ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
        rospy.logdebug( str(_fname) + ": BEGIN")

        ## make sure that commandOut ends in only one TERM_CHAR_SEND
        baseBehavior.__maki_cmd_msg_format = "\A[a-yA-Y]+[a-yA-Y0-9]*"
        baseBehavior.__maki_cmd_msg_format += str(TERM_CHAR_SEND)
        baseBehavior.__maki_cmd_msg_format += "{1}$"

        rospy.loginfo( str(_fname) + ": END")
        return

    def initSubMAKIFormat( self ):
        ## Setup regex template for expected feedback syntax
        _feedback_msg_format = "\A([A-Z]{2})"   ## 2 alphabetic char prefix
        _feedback_msg_format += "(([0-9]+" + DELIMITER_RECV + "){" + str(SERVOCOUNT-1) + "}[0-9]+)" 
        _feedback_msg_format += TERM_CHAR_RECV + "\Z"
        #print _feedback_msg_format
        baseBehavior.__maki_feedback_format = re.compile(_feedback_msg_format)
        return baseBehavior.__maki_feedback_format

    def initROSSub( self, feedback ):
        #print feedback

        # Setup subscribers
        for _prefix, _sub_params in feedback.iteritems():
            _topic = _sub_params[0]
            _type = _sub_params[1]
            _callback = _sub_params[2]
            rospy.Subscriber( _topic, _type, _callback )
            rospy.logdebug( "now subscribed to " + str(_topic) )
        return

    def initSubMAKIFeedback( self ):
        _maki_feedback_sub = {}		## init as empty dictionary
        _maki_feedback_sub[ str(SC_GET_PP) ] = ("maki_feedback_pres_pos", String, self.parseMAKIFeedbackMsg)
        _maki_feedback_sub[ str(SC_GET_ER) ] = ("maki_feedback_error", String, self.parseMAKIFeedbackMsg)
        baseBehavior.initROSSub( self, _maki_feedback_sub )
        return

    def parseMAKIFeedbackMsg ( self, recv_msg ):
        #if self.VERBOSE_DEBUG:
        #   #rospy.logdebug( "parseMAKIFeedbackMsg: BEGIN" )
        #   rospy.logdebug( "Received: " + str(recv_msg.data) )

        _tmp = baseBehavior.__maki_feedback_format.search( recv_msg.data )
        if _tmp != None:
            _prefix = _tmp.group(1)
            _feedback_values = _tmp.group(2)
            #print "Validated: prefix='" + _prefix + "' and feedback_values='" + _feedback_values + "'"
        else:
            rospy.logerr( "Received with ERROR! Invalid message format: " + str(recv_msg) )
            return  ## return without an expression argument returns None. Falling off the end of a function also returns None

        _values = re.findall("([0-9]+)", _feedback_values)  ## this is a list of strings
        ## need to conver to int (see http://stackoverflow.com/questions/22672598/converting-lists-of-digits-stored-as-strings-into-integers-python-2-7) 
        _tmp_dict = dict( zip(F_VAL_SEQ, map(int, _values)) )

        if (len(self.maki_feedback_values) == 0) or not ( str(_prefix) in self.maki_feedback_values ):
            ## if no _prefix entry exists in the dictionary, add new
            self.maki_feedback_values[ str(_prefix) ] = _tmp_dict
            rospy.logdebug( "New entry added to self.maki_feedback_values" + str(recv_msg) )

        elif not (_tmp_dict == self.maki_feedback_values[ str(_prefix) ]):
            ## if _prefix entry exists, update
            self.maki_feedback_values[ str(_prefix) ].update( _tmp_dict )
            #rospy.loginfo( "Updated entry in self.maki_feedback_values" + str(recv_msg) )
            rospy.logdebug( "Updated entry in self.maki_feedback_values" + str(recv_msg) )
        else:
            pass

        ## update
        if str(_prefix) == str(SC_GET_PP):
            self.makiPP = _tmp_dict

        #print "parseMAKIFeedbackMsg: END"
        return

    #####################
    ##
    ## This is blocking
    ##
    ## Raises rospy.exceptions.ROSException when breaks on stall
    ##
    ## TODO:
    ## *  Estimate duration based on the largest difference
    ##  between current and goal positions
    #####################
    def monitorMoveToGP( self, gp_cmd, hp_gp=None, ht_gp=None, ll_gp=None, lr_gp=None, ep_gp=None, et_gp=None, delta_pp=DELTA_PP, cmd_prop=True, msg_id=None):
        ### SEND THE COMMAND
        baseBehavior.pubTo_maki_command( self, gp_cmd, cmd_prop=cmd_prop )
        # This publishes the status of the behavior. We will send this to the VH
        #pub_behavior = rospy.Publisher("behavior_status",String,queue_size=10)

        rospy.logdebug("monitorMoveToGP(): delta_pp=" + str(delta_pp) + "; gp_cmd=" + gp_cmd)

        _moving_flag = dict( zip(F_VAL_SEQ, [None]*len(F_VAL_SEQ)) )
        _count_moving_flags = 0
        if (hp_gp != None) and (hp_gp != INVALID_INT):  
            _moving_flag["HP"] = True
            _count_moving_flags = _count_moving_flags +1
        if (ht_gp != None) and (ht_gp != INVALID_INT):  
            _moving_flag["HT"] = True
            _count_moving_flags = _count_moving_flags +1
        if (ll_gp != None) and (ll_gp != INVALID_INT):  
            _moving_flag["LL"] = True
            _count_moving_flags = _count_moving_flags +1
        if (lr_gp != None) and (lr_gp != INVALID_INT):  
            _moving_flag["LR"] = True
            _count_moving_flags = _count_moving_flags +1
        if (ep_gp != None) and (ep_gp != INVALID_INT):  
            _moving_flag["EP"] = True
            _count_moving_flags = _count_moving_flags +1
        if (et_gp != None) and (et_gp != INVALID_INT):  
            _moving_flag["ET"] = True
            _count_moving_flags = _count_moving_flags +1

        if _count_moving_flags == 0:    return

        ## NOTE: COMMENTED OUT... Let the behaviors themselves increment
        ##  Otherwise, duplicative increment is likely
        #self.count_movements = self.count_movements +1

        _stall_count = 0
        _loop_count = 0
        _old_makiPP = self.makiPP
        _start_time = rospy.get_time()
        ### REPEAT SENDING THE COMMAND
        baseBehavior.pubTo_maki_command( self, gp_cmd, cmd_prop=False )
        #while not rospy.is_shutdown():
## 2016-06-16, KATE
        while ((not rospy.is_shutdown()) and (not self.mTT_INTERRUPT)):
            _loop_count += 1

            ## There is an implicit sleep in requestFeedback of 100ms (default)
            baseBehavior.requestFeedback( self, SC_GET_PP ) 

            ### TODO: THIS WOULD BE A NICE OPTIMIZATION
            ### Check to see if the goal position command has propogated
            ###     If so, there should be changes in PP
            ###     Otherwise, catch a breath... continue
            ###     instead of inflating _stall_count
            #if (_old_makiPP == self.makiPP):
            #   rospy.logwarn("baseBehavior.monitorMoveToGP(): CONTINUE!!!")
            #   continue    ## jump back to the top of the loop
            

            if _moving_flag["HP"] and (abs(self.makiPP["HP"] - hp_gp) <= delta_pp):
                rospy.logdebug("HP done moving")
                _moving_flag["HP"] = False
                _count_moving_flags = _count_moving_flags -1

            if _moving_flag["HT"] and (abs(self.makiPP["HT"] - ht_gp) <= delta_pp):
                rospy.logdebug("HT done moving")
                _moving_flag["HT"] = False
                _count_moving_flags = _count_moving_flags -1

            if _moving_flag["EP"] and (abs(self.makiPP["EP"] - ep_gp) <= delta_pp):
                rospy.logdebug("EP done moving")
                _moving_flag["EP"] = False
                _count_moving_flags = _count_moving_flags -1

            if _moving_flag["ET"] and (abs(self.makiPP["ET"] - et_gp) <= delta_pp):
                rospy.logdebug("ET done moving")
                _moving_flag["ET"] = False
                _count_moving_flags = _count_moving_flags -1

            if _moving_flag["LL"] and (abs(self.makiPP["LL"] - ll_gp) <= delta_pp):
                rospy.logdebug("LL done moving")
                _moving_flag["LL"] = False
                _count_moving_flags = _count_moving_flags -1

            if _moving_flag["LR"] and (abs(self.makiPP["LR"] - lr_gp) <= delta_pp):
                rospy.logdebug("LR done moving")
                _moving_flag["LR"] = False
                _count_moving_flags = _count_moving_flags -1

            if _count_moving_flags == 0:    break

            if (_old_makiPP == self.makiPP):
                _stall_count += 1
                rospy.logdebug("... _stall_count = " + str(_stall_count) )
                ## 2016-07-21 ktsui: comment out
                #baseBehavior.requestFeedback( self, SC_GET_PP ) 
                #baseBehavior.requestFeedback( self, SC_GET_PP, time_ms=250 ) 
## KATE
                if _stall_count != _loop_count:
                    rospy.logdebug("potential STALL RECOVERY?? _loop_count=" + str(_loop_count) + ", _stall_count=" + str(_stall_count))


            if (_stall_count == 10):    
                #rospy.logerr("STALLED!!!")
                raise rospy.exceptions.ROSException("STALLED!!!! self.makiPP hasn't changed... is the motor at its limit?")
         #       pub_behavior.publish("{} STALLED".format(msg_id))
                break
            _old_makiPP = self.makiPP
        _duration = abs(rospy.get_time() - _start_time)
        rospy.logdebug("monitorMoveToGP() **** DONE! movement took " + str(_duration) + " seconds")
        #pub_behavior.publish("{} COMPLETED".format(msg_id))
        return

    ## Funtion:     Verifes if the current pose matches the specified pose (within tolerance delta_pp)
    ## Returns:     True or False
    def verifyPose( self, hp=None, ht=None, ll=None, lr=None, ep=None, et=None, delta_pp=DELTA_PP ):
        _ret = False

        _request_verify_flag = dict( zip(F_VAL_SEQ, [None]*len(F_VAL_SEQ)) )
        _count_request_verify_flags = 0
        if (hp != None) and (hp != INVALID_INT):    
            _request_verify_flag["HP"] = True
            _count_request_verify_flags = _count_request_verify_flags +1
        if (ht != None) and (ht != INVALID_INT):    
            _request_verify_flag["HT"] = True
            _count_request_verify_flags = _count_request_verify_flags +1
        if (ll != None) and (ll != INVALID_INT):    
            _request_verify_flag["LL"] = True
            _count_request_verify_flags = _count_request_verify_flags +1
        if (lr != None) and (lr != INVALID_INT):    
            _request_verify_flag["LR"] = True
            _count_request_verify_flags = _count_request_verify_flags +1
        if (ep != None) and (ep != INVALID_INT):    
            _request_verify_flag["EP"] = True
            _count_request_verify_flags = _count_request_verify_flags +1
        if (et != None) and (et != INVALID_INT):    
            _request_verify_flag["ET"] = True
            _count_request_verify_flags = _count_request_verify_flags +1

        if _count_request_verify_flags == 0:    
            rospy.logdebug("verifyPose(): INVALID goal poses")
            return _ret     #False

        ## request feedback of present positions
        baseBehavior.requestFeedback( self, SC_GET_PP )
        _makiPP = self.makiPP

        ## build dictionary to verify
        _verify_dict = {}
        _verify_dict["HP"] = hp
        _verify_dict["HT"] = ht
        _verify_dict["EP"] = ep
        _verify_dict["ET"] = et
        _verify_dict["LL"] = ll
        _verify_dict["LR"] = lr

        for _motor in F_VAL_SEQ:
            if _request_verify_flag[_motor]:
                if (abs(_makiPP[_motor] - _verify_dict[_motor]) < delta_pp):
                    rospy.logdebug( _motor + " present position matches goal position")
                    _request_verify_flag[_motor] = False
                    _count_request_verify_flags = _count_request_verify_flags -1
                else:
                    _ret = False
                    if self.VERBOSE_DEBUG:
                        rospy.logwarn("verifyPose(): " + _motor + " present position (" + str(_makiPP[_motor]) + ") DOES NOT matches goal position (" + str(_verify_dict[_motor]) + ")" )
                    else:
                        break	## break the for loop; no sense in performing unnecessary calculations
        #end	for _motor in F_VAL_SEQ:

        if _count_request_verify_flags == 0:	_ret = True

        return _ret

    # blocks until movement is stable for 200ms or until 4 seconds have passed.
    def waitForMovementToComplete(self):
        lastPP = self.makiPP
        equalTimes = 0
        totalTimes = 0
        maxTimes = 80
        
        while not rospy.is_shutdown() and equalTimes < 4 and totalTimes < maxTimes:
            # sleeps 50ms as well as requests.
            baseBehavior.requestFeedback( self, SC_GET_PP, time_ms=50 )
            if lastPP == self.makiPP:
                equalTimes += 1
                totalTimes += 1
            else:
                equalTimes = 0
            lastPP = self.makiPP
            
        if totalTimes >= maxTimes:
            rospy.logerror('Waiting for movement to complete for more than ' + str(maxTimes * 50) + 'ms. Is something wrong?')
        
########################
## All behavior macros involving head pan (HP) will use this as base class
########################
class headPanBaseBehavior(baseBehavior):
    def __init__(self, verbose_debug, ros_pub):
        ## call base class' __init__
        baseBehavior.__init__( self, verbose_debug, ros_pub )
        ## now custom things
        self.prefix_hpgp = "HP" + str(SC_SET_GP)
        self.prefix_epgp = "EP" + str(SC_SET_GP)
        self.hpgp_regex = self.prefix_hpgp + "([0-9]+)"
        self.aCFG_coeff = 0.36      ## average of 0.21 and 0.53 from Todorvic 2009

    ## Automatically fill in commandOut to adjust eye pan position based on head pan position
    ##  if eye pan position is missing
    def pubTo_maki_command( self, commandOut, fixed_gaze=True, cmd_prop=True, time_ms=100, time_inc=0.01 ):
        rospy.logdebug("headPanBaseBehavior.pubTo_maki_command(): BEGIN")
        if fixed_gaze:  commandOut = headPanBaseBehavior.amend_maki_command( self, commandOut, fixed_gaze=fixed_gaze )
        return baseBehavior.pubTo_maki_command( self, commandOut, cmd_prop=cmd_prop, time_ms=time_ms, time_inc=time_inc )

    ## Amend to adjust for fixed gaze
    def monitorMoveToGP( self, gp_cmd, fixed_gaze=True, hp_gp=None, ht_gp=None, ll_gp=None, lr_gp=None, ep_gp=None, et_gp=None, delta_pp=DELTA_PP, cmd_prop=True ):
        rospy.logdebug("headPanBaseBehavior.monitorMoveToGP(): BEGIN")
        if fixed_gaze:  gp_cmd = headPanBaseBehavior.amend_maki_command( self, gp_cmd, fixed_gaze=fixed_gaze )
        return baseBehavior.monitorMoveToGP( self, gp_cmd, hp_gp=hp_gp, ht_gp=ht_gp, ll_gp=ll_gp, lr_gp=lr_gp, ep_gp=ep_gp, et_gp=et_gp, delta_pp=delta_pp, cmd_prop=cmd_prop )


    def amend_maki_command( self, commandOut, fixed_gaze=True ):
        if fixed_gaze and (self.prefix_hpgp in commandOut) and not (self.prefix_epgp in commandOut):
            rospy.logdebug("Need to adjust commandOut...")
            ## get HPGP value
            _tmp = re.search( self.hpgp_regex, commandOut )
            if _tmp != None:
                _hp_gp = int( _tmp.group(1) )
            else:
                return commandOut

            ## calculate EPGP value
            _ep_gp = EP_FRONT   ## default
            _ep_gp = headPanBaseBehavior.autoCalculateFixedGazeFromHPGP( self, _hp_gp )

            ## update commandOut
            commandOut = self.prefix_epgp + str(_ep_gp) + commandOut    
            rospy.logdebug("headPanBaseBehavior.pubTo_maki_command(): Update commandOut to: " + str(commandOut))
        return commandOut

    ## Todorovic 2009: "In order to maintain the perceptions of fixed gaze, every 1% shift of
    ##  the facial features from centered, corresponded to an iris shift in the lid
    ##  apertures of 0.21-0.53% depending on testing method"
    def autoCalculateFixedGazeFromHPGP( self, hp_gp ):
        rospy.logdebug("autoCalculateFixedGazeFromHPGP(): BEGIN")

        _hp_gp_degrees = self.DC_helper.convertToDegrees_ticks( abs(hp_gp - HP_FRONT) ) 
        _ep_gp_degrees = _hp_gp_degrees * self.aCFG_coeff
        ret = self.DC_helper.convertToTicks_degrees( _ep_gp_degrees )
        if (hp_gp < HP_FRONT):
            ret = EP_FRONT - ret
        else:
            ret = EP_FRONT + ret
        rospy.logdebug("given hp_gp=" + str(hp_gp) + ", computed ep_gp=" + str(ret))
        rospy.logdebug("autoCalculateFixedGazeFromHPGP(): END")
        return ret

########################
## All behavior macros involving head tilt (HT) will use this as base class
########################
class headTiltBaseBehavior(baseBehavior):
    ## all instances of this class share the same value
    ## variables private to this class
    __ht_enabled = None
    __ht_enable_cmd = None
    __ht_disable_cmd = None

    __maki_feedback_format = None

    def __init__(self, verbose_debug, ros_pub):
        ## call base class' __init__
        baseBehavior.__init__( self, verbose_debug, ros_pub )
        if (headTiltBaseBehavior.__maki_feedback_format == None):
            headTiltBaseBehavior.__maki_feedback_format = baseBehavior.initSubMAKIFormat( self )

        self.htgp_regex = "HT" + str(SC_SET_GP) + "([0-9]+)"

        ## subscribe to rostopic maki_feedback_torque_limit and maki_feedback_goal_pos
        _maki_feedback_sub = {}         ## init as empty dictionary
        _maki_feedback_sub[ str(SC_GET_TL) ] = ("maki_feedback_torque_limit", String, self.parseMAKIFeedbackMsg)
        _maki_feedback_sub[ str(SC_GET_PT) ] = ("maki_feedback_pres_temp", String, self.parseMAKIFeedbackMsg)
        _maki_feedback_sub[ str(SC_GET_GP) ] = ("maki_feedback_goal_pos", String, self.parseMAKIFeedbackMsg)
        baseBehavior.initROSSub( self, _maki_feedback_sub )

        if headTiltBaseBehavior.__ht_enabled == None:
            headTiltBaseBehavior.__ht_enabled = False
        if headTiltBaseBehavior.__ht_enable_cmd == None:
            headTiltBaseBehavior.__ht_enable_cmd = "HT" + str(SC_SET_TL) + str(ht_tl_enable) + str(TERM_CHAR_SEND)
        if headTiltBaseBehavior.__ht_disable_cmd == None:
            headTiltBaseBehavior.__ht_disable_cmd = "HT" + str(SC_SET_TL) + str(ht_tl_disable) + str(TERM_CHAR_SEND)
        try:
            thread.start_new_thread( headTiltBaseBehavior.monitorMotorTemperature, ( self, ) )
        except Exception as _e:
            rospy.logwarn("Unable to start a new thread for headTiltBaseBehavior.monitorMotorTemperature(): " + str(_e))

        return


    def monitorMotorTemperature( self ):
        rospy.logdebug("Start a new thread to monitor the motors' present temperature...")
        _start_time = rospy.get_time()

        ## init as empty dictionaries
        _start_pt = {}
        _current_pt = {}
        _previous_pt = {}
        _running_sum_pt = {}
        _delta_pt = {}
        _average_pt = {}    

        headTiltBaseBehavior.requestFeedback( self, SC_GET_PT )
        ## check to see if there is an entry with key "PT"
        #while (not rospy.is_shutdown()):
## 2016-06-16, KATE
        while (not rospy.is_shutdown()) and self.ALIVE:
            ## if we got a message on /maki_feedback_pres_temp
            if ( str(SC_GET_PT) in self.maki_feedback_values ):
                break   ## break the while loop
            else:
                rospy.logdebug("Waiting for a message on /maki_feedback_pres_temp...")
                ## request a feedback message
                headTiltBaseBehavior.requestFeedback( self, str(SC_GET_PT) )
                rospy.sleep(30)     ## wait for 30 seconds

        ## save our initial state
        _start_pt.update( self.maki_feedback_values[ SC_GET_PT ] )
        _current_pt.update( self.maki_feedback_values[ SC_GET_PT ] )
        _previous_pt.update( self.maki_feedback_values[ SC_GET_PT ] )
        _running_sum_pt.update( self.maki_feedback_values[ SC_GET_PT ] )
        _running_average_pt = dict( zip(F_VAL_SEQ, [ 0 ] * len(F_VAL_SEQ) ) )
        _delta_pt.update( headTiltBaseBehavior.computeTemperature( self, _current_pt, _previous_pt, "delta" ) )
        _average_pt.update( headTiltBaseBehavior.computeTemperature( self, _current_pt, _previous_pt, "average" ) )
        ## TODO
        _average_delta_pt = _running_average_pt
        ## TODO: max
        ## TODO: min
        ## TODO: median

        _loop_count = 1
        _ht_pt = _current_pt["HT"]
        #while (not rospy.is_shutdown()):
## 2016-06-16, KATE
        while (not rospy.is_shutdown()) and self.ALIVE:
            _loop_count += 1
            rospy.sleep(60.0 * 2.5)     ## sample motors' present temperature every 2.5 minutes
            #rospy.sleep(30.0)  ## faster for debugging
            headTiltBaseBehavior.requestFeedback( self, SC_GET_PT )
            _current_pt.update( self.maki_feedback_values[ SC_GET_PT ] )

            ## Calculate some statistics
            _delta_pt = headTiltBaseBehavior.computeTemperature( self, _current_pt, _previous_pt, "delta" )
            _average_pt = headTiltBaseBehavior.computeTemperature( self, _current_pt, _previous_pt, "average" )
            _running_sum_pt = headTiltBaseBehavior.computeTemperature( self, _running_sum_pt, _current_pt, "add" )
            _running_average_pt = headTiltBaseBehavior.computeTemperature( self, _running_sum_pt, _loop_count, "running_average" )


            if not (_previous_pt == _current_pt):
                rospy.logdebug("monitorMotorTemperature(): -------- Temperature change detected --------")
                rospy.logdebug("Previous temp (Celsius): " + str(_previous_pt))
                rospy.logdebug("Current temp (Celsius): " + str(_current_pt))
                rospy.logdebug("Change of 2 most recent readings: " + str(_delta_pt))
                rospy.logdebug("Average of 2 most recent readings: " + str(_average_pt))
                rospy.logdebug("Running average (n=" + str(_loop_count) + "): " + str(_running_average_pt))

            ## Post information about heat tilt motor's temperature
            _previous_ht_pt = _ht_pt
            _ht_pt = _current_pt["HT"]
            if _ht_pt >= 70:
                rospy.logerr("!!!!!!!! ERROR: Head tilt motor is OVERHEATED. Turn off Maki-ro IMMEDIATELY and leave off for 20 minutes !!!!!!!")
            elif _ht_pt > 65:
                rospy.logwarn("**** DANGER: Head tilt motor will soon OVERHEAT ( " + str(_ht_pt) + " C ) ****")
            elif _ht_pt > 55:
                if (_ht_pt != _previous_ht_pt):
                    rospy.logwarn("**** WARNING: Head tilt motor is HEATING UP ( " + str(_ht_pt) + " C ) ****")
            elif _ht_pt >= 50:
                if (_ht_pt != _previous_ht_pt):
                    rospy.logwarn("**** NOTIFICATION: Head tilt motor is getting WARM ( " + str(_ht_pt) + " C ) ****")
            else:
                pass

            ## update
            _previous_pt.update( _current_pt )
        #end    while (not rospy.is_shutdown()):
        return

    def computeTemperature( self, pt_1, pt_2, op ):
        ## check the validity of the inputs
        if ((not isinstance(op, basestring)) or
            ((op != "delta") and
            (op != "add") and
            (op != "average") and
            (op != "running_average")) ):
            rospy.logwarn("computeTemperature(): INVALID INPUT: op given as " + str(op))
            return None

        _ret_pt = dict( zip(F_VAL_SEQ, [ 0 ] * len(F_VAL_SEQ) ) )
    
        for _servo in F_VAL_SEQ:
            if _servo == "LR":  
                _ret_pt[ _servo ] = pt_1[ _servo ]
                continue

            if op == "delta":
                _ret_pt[ _servo ] = float(pt_1[ _servo ]) - float(pt_2[ _servo ])
            elif op == "add":
                _ret_pt[ _servo ] = float(pt_1[ _servo ]) + float(pt_2[ _servo ])
            elif op == "average":
                _ret_pt[ _servo ] = float( pt_1[ _servo ] + pt_2[ _servo ]) * 0.5
            elif op == "running_average":
                _ret_pt[ _servo ] = float( pt_1[ _servo ] / float(pt_2) )
            else:
                rospy.logwarn("INVALID OP: " + str(op))
                return None

        return _ret_pt


    ## override base class
    def start( self, makiPP=None, enable_ht=True ):
        ## need to call base class' start function first!!!!
        baseBehavior.start( self, makiPP )

        ## check to see if there is an entry with key "TL"
        #while (not rospy.is_shutdown()):
## 2016-06-16, KATE
        while (not rospy.is_shutdown()) and self.ALIVE:
            ## if we got a message on /maki_feedback_torque_limit
            if ( str(SC_GET_TL) in self.maki_feedback_values ):
                break   ## break the while loop
            else:
                rospy.loginfo("Waiting for a message on /maki_feedback_torque_limit...")
                ## request a feedback message
                headTiltBaseBehavior.requestFeedback( self, str(SC_GET_TL) )

            #self.SWW_WI.sleepWhileWaiting( 1 )     ## 1 second

        if enable_ht:   self.enableHT()


    ## override base class
    def stop( self, disable_ht=True ):
        rospy.logdebug( "headTiltBaseBehavior: stop()" )
        ## call base behavior first
        baseBehavior.stop( self )
        if disable_ht:  self.disableHT()
        rospy.logdebug( "headTiltBaseBehavior: stop() -- END" )


    def isHTEnabled( self ):
        return headTiltBaseBehavior.__ht_enabled


    def enableHT( self ):
        if headTiltBaseBehavior.__ht_enabled == True:
            ## already enabled
            ## keep enabled; head tilt servo monitor might be running, so reset timer
            headTiltBaseBehavior.pubTo_maki_command( self, str(headTiltBaseBehavior.__ht_enable_cmd), cmd_prop=False )
            return

        ## typical result
        ## enableHT duration: 0.589332818985 seconds
        ## MIN: 0.532744884491 seconds
        _enableHT_start_time = rospy.get_time()

        ## re-init values
        self.makiPP = None

        ## wait here until feedback is published to rostopics
        _loop_count = 0
        while (((self.makiPP == None) or 
            (not (SC_GET_PP in self.maki_feedback_values)) or
            (not (SC_GET_TL in self.maki_feedback_values))) and
            (not self.mTT_INTERRUPT) and        ## 2016-06-16, KATE
            (not rospy.is_shutdown())):
            if (_loop_count == 0):
                ## request current servo motor values
                headTiltBaseBehavior.requestFeedback( self, str(SC_GET_TL) )
            elif (_loop_count == 3):
                headTiltBaseBehavior.requestFeedback( self, str(SC_GET_PP) )
            else:
                self.SWW_WI.sleepWhileWaitingMS( 100, 0.01 )    ## make sure command propogates
            _loop_count = (1 + _loop_count) % 20
            rospy.logdebug( "enable, 1st while: " + str(_loop_count) )

        if (self.maki_feedback_values[ str(SC_GET_TL) ]["HT"] == ht_tl_enable):
            ## keep enabled; head tilt servo monitor might be running, so reset timer
            headTiltBaseBehavior.pubTo_maki_command( self, str(headTiltBaseBehavior.__ht_enable_cmd), cmd_prop=False )
            ## set flag to reflect servo status
            headTiltBaseBehavior.__ht_enabled = True
            return

        ## Otherwise, HT TL is 0%
        ## Set the goal position as the present position
        if (self.makiPP["HT"] <= HT_MAX and self.makiPP["HT"] >= HT_MIN):
            _pub_cmd_GP = "HT" + str(SC_SET_GP) + str(self.makiPP["HT"]) + str(TERM_CHAR_SEND)
            ## bypass headTiltBaseBehavior.pubTo_maki_command() since in this
            ##  one case, we want to set the goal position first
            ##      also, we are manually below setting TL (torque load)
            baseBehavior.pubTo_maki_command( self, str(_pub_cmd_GP) )
## 100ms
        
        ## wait here until feedback is published to rostopics
        _loop_count = 0
        #while (self.maki_feedback_values[ str(SC_GET_TL) ]["HT"] != ht_tl_enable):
        while (not rospy.is_shutdown()) and (not headTiltBaseBehavior.__ht_enabled):    ## value set in parseMAKIFeedbackMsg
            if (_loop_count == 0):
                headTiltBaseBehavior.pubTo_maki_command( self, str(headTiltBaseBehavior.__ht_enable_cmd) )
## 100ms

            ## every pass through,
            ## request current servo motor values
            headTiltBaseBehavior.requestFeedback( self, str(SC_GET_TL) )
## 100ms

            _loop_count = (1 + _loop_count) % 10
            rospy.logdebug( "enable, 2nd while: " + str(_loop_count) )

        ## we only get here if the head tilt enable command has been sent, executed,
        ## and status reflected in the servo motor status
        headTiltBaseBehavior.__ht_enabled = True

        rospy.logdebug( "enableHT duration: " + str( rospy.get_time() - _enableHT_start_time ) + " seconds" )
        return


    def disableHT( self ):
        if headTiltBaseBehavior.__ht_enabled == False:
            ## already disabled
            return

        ## typical result:
        ## disableHT duration: 0.151942968369 s
        ## MAX: 0.203392982483 s
        _disableHT_start_time = rospy.get_time()

        _loop_count = 0
        #while (self.maki_feedback_values[ str(SC_GET_TL) ]["HT"] != ht_tl_disable):
        while (not rospy.is_shutdown()) and headTiltBaseBehavior.__ht_enabled:  ## value set in parseMAKIFeedbackMsg
            if (_loop_count % 10) == 0:
                headTiltBaseBehavior.pubTo_maki_command( self, str(headTiltBaseBehavior.__ht_disable_cmd) )
                headTiltBaseBehavior.requestFeedback( self, str(SC_GET_TL) )
            else:
                self.SWW_WI.sleepWhileWaitingMS( 100, 0.01 )    ## make sure command propogates
            _loop_count = 1 + _loop_count

        headTiltBaseBehavior.__ht_enabled = False

        rospy.logdebug(" disableHT duration: " + str( rospy.get_time() - _disableHT_start_time ) + " seconds" )
        return


    ### TODO: bug fix -- need to replicate bug first!
    #def reset( self ):
    #
    #   if (self.makiPP["HT"] <= HT_UP and self.makiPP["HT"] >= HT_DOWN):
    #       _pub_cmd_GP = "HT" + str(SC_SET_GP) + str(self.makiPP["HT"]) + str(TERM_CHAR_SEND)
    #       baseBehavior.pubTo_maki_command( self, str(_pub_cmd_GP) )
    #   
    #   ## send enable command
    #   headTiltBaseBehavior.pubTo_maki_command( self, str(headTiltBaseBehavior.__ht_enable_cmd) )
    #   headTiltBaseBehavior.__ht_enabled = True
    #
    #   ## publish "reset" to /maki_command
    #   headTiltBaseBehavior.pubTo_maki_command( self, "reset" )


    ## override base class
    def parseMAKIFeedbackMsg ( self, recv_msg ):
        _tmp = headTiltBaseBehavior.__maki_feedback_format.search( recv_msg.data )
        if _tmp != None:
            _prefix = _tmp.group(1)
            if str(_prefix) == str(SC_GET_TL):

                _feedback_values = _tmp.group(2)
                #print "Validated: prefix='" + _prefix + "' and feedback_values='" + _feedback_values + "'"

                _values = re.findall("([0-9]+)", _feedback_values)  ## this is a list of strings
                ## need to conver to int (see http://stackoverflow.com/questions/22672598/converting-lists-of-digits-stored-as-strings-into-integers-python-2-7) 
                _tmp_dict = dict( zip(F_VAL_SEQ, map(int, _values)) )

                ## update
                _ht_tl_val = _tmp_dict["HT"]
                if (_ht_tl_val == ht_tl_disable):
                    if (headTiltBaseBehavior.__ht_enabled == True):
                        headTiltBaseBehavior.__ht_enabled = False
                        rospy.logdebug("TL feedback received; disable headTiltBaseBehavior.__ht_enabled flag: " + str(headTiltBaseBehavior.__ht_enabled))
                elif (_ht_tl_val == ht_tl_enable):
                    if (headTiltBaseBehavior.__ht_enabled == False):
                        headTiltBaseBehavior.__ht_enabled = True
                        rospy.logdebug("TL feedback received; enable headTiltBaseBehavior.__ht_enabled flag: " + str(headTiltBaseBehavior.__ht_enabled))
                else:
                    rospy.logerr( "Invalid head tilt torque limit: " + str(_ht_tl_val) )

        ## call base class
        baseBehavior.parseMAKIFeedbackMsg( self, recv_msg )
        return


    ## override base class
    def pubTo_maki_command( self, commandOut, cmd_prop=True, time_ms=100, time_inc=0.01 ):
        ## Check to see if goal position is specified for head tilt
        ## get HTGP value
        _tmp = re.search( self.htgp_regex, commandOut )
        if (((_tmp != None) or (commandOut == "reset")) and
            (not headTiltBaseBehavior.isHTEnabled( self ))):
            ## if head tilt is not enabled before publishing a message to /maki_command
            headTiltBaseBehavior.enableHT( self )
            rospy.logdebug("pubTo_maki_command(): About to publish on /maki_command (" + str(commandOut) + ") but isHTEnabled==False... FIRST call enableHT()")

        return baseBehavior.pubTo_maki_command( self, commandOut, cmd_prop=cmd_prop, time_ms=time_ms, time_inc=time_inc )


    ## override base class
    def monitorMoveToGP( self, gp_cmd, hp_gp=None, ht_gp=None, ll_gp=None, lr_gp=None, ep_gp=None, et_gp=None, delta_pp=DELTA_PP, cmd_prop=True ):
        #rospy.loginfo("gp_cmd=" + str(gp_cmd) + ", hp_gp=" + str(hp_gp) + ", ht_gp=" + str(ht_gp))
        ## Check to see if goal position is specified for head tilt
        ## get HTGP value
        _tmp = re.search( self.htgp_regex, gp_cmd )
        if (((_tmp != None) or (gp_cmd == "reset")) and
            (not headTiltBaseBehavior.isHTEnabled( self ))):
            ## if head tilt is not enabled before publishing a message to /maki_command
            headTiltBaseBehavior.enableHT( self )
            rospy.logdebug("monitorMoveToGP(): About to publish on /maki_command (" + str(gp_cmd) + ") but isHTEnabled==False... FIRST call enableHT()")
        return baseBehavior.monitorMoveToGP( self, gp_cmd, hp_gp=hp_gp, ht_gp=ht_gp, ll_gp=ll_gp, lr_gp=lr_gp, ep_gp=ep_gp, et_gp=et_gp, delta_pp=delta_pp, cmd_prop=cmd_prop )
    

########################
## All behavior macros involving eyelids (LL) will use this as base class
##
## TODO: Incorporate LR even though doesn't exist in our Maki-ro
########################
class eyelidBaseBehavior( baseBehavior ):
    ## variables private to this class
    ## all instances of this class share the same value
    # none


    def __init__(self, verbose_debug, ros_pub):
        ## call base class' __init__
        baseBehavior.__init__( self, verbose_debug, ros_pub )
        ## add anything else needed by an instance of this subclass

        if self.makiPP == None:
            self.makiPP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )

        self.origin_ll = LL_OPEN_DEFAULT        ## default is neutral
        self.ll_open = LL_OPEN_DEFAULT
        self.ll_close = LL_CLOSE_MAX
        self.ll_delta_range = abs(self.ll_open - self.ll_close)     ## ticks

        self.ALIVE = True
        return

    def setEyelidRange( self, ll, ll_delta=None, ll_close=None):
        _start_time = rospy.get_time()

        if ll_delta == None and ll_close == None:   
            ll_delta = self.ll_delta_range  
        elif ll_delta == None and ll_close != None:
            self.ll_delta_range = abs(ll - ll_close)
            ll_delta = self.ll_delta_range
        else:
            ## update with new value
            self.ll_delta_range = ll_delta

        self.ll_close = ll - ll_delta
        self.ll_open = ll   ## passed position

        ## check range values
        if self.ll_close < LL_CLOSE_MAX:    self.ll_close = LL_CLOSE_MAX
        if self.ll_open > LL_OPEN_MAX:  self.ll_open = LL_OPEN_MAX
        rospy.logdebug("Eyelid range: (" + str(self.ll_close) + ", " + str(self.ll_open) + ")")

        rospy.loginfo("setEyelidRange(): elapsed time = " + str( rospy.get_time() - _start_time ) + " seconds" )
        return

    def setEyelidNeutralPose( self, ll, monitor=False, cmd_prop=False):
        _start_time = rospy.get_time()

        self.origin_ll = ll
        _pub_cmd = "LLGP" + str(self.origin_ll) + str(TERM_CHAR_SEND) 

        if monitor:
            try:
                eyelidBaseBehavior.monitorMoveToGP( self, _pub_cmd, ll_gp=self.origin_ll )
            except rospy.exceptions.ROSException as e:
## 2016-06-16, KATE
                ## last effort before raising exception
                eyelidBaseBehavior.pubTo_maki_command( self, _pub_cmd, cmd_prop=cmd_prop)
                raise e
        else:
            eyelidBaseBehavior.pubTo_maki_command( self, _pub_cmd, cmd_prop=cmd_prop)

        
        rospy.loginfo("setEyelidNeutralPose(): elapsed time = " + str( rospy.get_time() - _start_time ) + " seconds" )
        return

    def eyelidClose( self, ll_close=None, monitor=False, cmd_prop=False ):
        _pub_cmd = "LLGP" 
        if ll_close != None:    
            _pub_cmd += str(ll_close) 
        else:
            _pub_cmd += str(self.ll_close) 
        _pub_cmd += str(TERM_CHAR_SEND) 

        if monitor:
            try:
                eyelidBaseBehavior.monitorMoveToGP( self, _pub_cmd, ll_gp=self.ll_close )
            except rospy.exceptions.ROSException as e:
## 2016-06-16, KATE
                ## last effort before raising exception
                eyelidBaseBehavior.pubTo_maki_command( self, _pub_cmd, cmd_prop=cmd_prop)
                raise e
        else:
            eyelidBaseBehavior.pubTo_maki_command( self, _pub_cmd, cmd_prop=cmd_prop)
        return
        
    def eyelidOpen( self, ll_open=None, monitor=False, cmd_prop=False ):
        _start_time = rospy.get_time()

        _pub_cmd = "LLGP" 
        if ll_open !=None:
            _pub_cmd += str(ll_open)
        else:
            _pub_cmd += str(self.ll_open) 
        _pub_cmd += str(TERM_CHAR_SEND) 

        if monitor:
            try:
                eyelidBaseBehavior.monitorMoveToGP( self, _pub_cmd, ll_gp=self.ll_open )
            except rospy.exceptions.ROSException as e:
## 2016-06-16, KATE
                ## last effort before raising exception
                eyelidBaseBehavior.pubTo_maki_command( self, _pub_cmd, cmd_prop=cmd_prop)
                raise e
        else:
            eyelidBaseBehavior.pubTo_maki_command( self, _pub_cmd, cmd_prop=cmd_prop)
        
        rospy.logdebug("eyelidOpen(): elapsed time = " + str( rospy.get_time() - _start_time ) + " seconds" )
        return


########################
##
## All behavior macros involving eyelids (LL) and head tilt (HT)
##  will use this as base class. Inherits from both
##  eyelidBaseBehavior and headTiltBaseBehavior
##
########################
class eyelidHeadTiltBaseBehavior( eyelidBaseBehavior, headTiltBaseBehavior ):
    ## variables private to this class
    ## all instances of this class share the same value
    # none


    def __init__(self, verbose_debug, ros_pub):
        ## call eyelid base class' __init__
        eyelidBaseBehavior.__init__( self, verbose_debug, ros_pub )
        ## call headTilt base class' __init__
        headTiltBaseBehavior.__init__( self, verbose_debug, self.ros_pub )
        ## add anything else needed by an instance of this subclass

        return

    def start( self, makiPP=None, enable_ht=True ):
        ## eyelidHeadTiltBaseBehavior inherits from both
        ## eyelidBaseBehavior and headTiltBaseBehavior,
        ## and both inherit from baseBehavior.
        ## eyelidBaseBehavior.start() defaults to
        ## baseBehavior.start(), headTiltBaseBehavior.start()
        ## overrides this, so  
        ## call headTilt base class' start()
        return headTiltBaseBehavior.start( self, makiPP, enable_ht )

    def stop( self, disable_ht=True ):
        ## eyelidHeadTiltBaseBehavior inherits from both
        ## eyelidBaseBehavior and headTiltBaseBehavior,
        ## and both inherit from baseBehavior.
        ## eyelidBaseBehavior.stop() defaults to
        ## baseBehavior.stop(), headTiltBaseBehavior.stop()
        ## overrides this, so  
        ## call headTilt base class' stop()
        return headTiltBaseBehavior.stop( self, disable_ht=disable_ht )

    def parseMAKIFeedbackMsg ( self, recv_msg ):
        ## eyelidHeadTiltBaseBehavior inherits from both
        ## eyelidBaseBehavior and headTiltBaseBehavior,
        ## and both inherit from baseBehavior.
        ## eyelidBaseBehavior.parseMAKIFeedbackMsg() defaults to
        ## baseBehavior.parseMAKIFeedbackMsg(), headTiltBaseBehavior.parseMAKIFeedbackMsg()
        ## overrides this, so  
        ## call headTilt base class' parseMAKIFeedbackMsg()
        return headTiltBaseBehavior.parseMAKIFeedbackMsg( self, recv_msg )



