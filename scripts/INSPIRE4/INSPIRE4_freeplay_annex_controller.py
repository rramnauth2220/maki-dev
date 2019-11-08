#! /usr/bin/env python

# RUN AS:    rosrun maki_robot INSPIRE4_freeplay_annex_controller.py

# NOTE:    To be run in conjunction with master-table.xls

import rospy
import re
from std_msgs.msg import String
from std_srvs.srv import Empty

import signal
import sys
import string
import random
import thread

from maki_robot_common import *
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt

from blinking import *
from selective_attention import *
from asleep_awake import *
from INSPIRE4_look_intro import *
from INSPIRE4_engagement import *
from INSPIRE4_look_interactions import *


## Copied with light modifications from INSPIRE4_controller.py
## TODO: needs cleaning

## ------------------------------
class freeplayAnnexController( object ):
    ## all instances of this class will share the same value
    ## variables private to this class
    RESET_EXP = 0
    READY = 1
    SYNC = 2
    INTRO = 3
    ENGAGEMENT = 4
    STIMULI = 5
    END = 6
    INVALID_TRIAL = 7

    NUMBER_OF_INTERACTIONS = 6

    #state_dict = {RESET_EXP: 'reset experiment', READY: 'ready', SYNC: 'sync', INTRO: 'intro', ENGAGEMENT: 'engagement', STIMULI: 'stimuli', END: 'end', INVALID_TRIAL: 'invalid trial'}
    state_dict = {RESET_EXP: 'RESET EXPERIMENT', READY: 'READY', SYNC: 'SYNC', INTRO: 'INTRO', ENGAGEMENT: 'ENGAGEMENT', STIMULI: 'STIMULI', END: 'THE END', INVALID_TRIAL: 'INVALID TRIAL'}


    def __init__(self, verbose_debug, ros_pub):


        ## primarily will publish to /maki_macro
        if ros_pub == None:
            self.initROS( self )
        else:
            self.ros_pub = ros_pub ## can we pass a ros publisher??? Apparently so!
        freeplayAnnexController.initROSPub( self )

        ## publisher for curated message to the experimenter
        self.exp_pub = rospy.Publisher("experiment_info", String, queue_size = 10)
        ## NOTE: seems that we can't publish from within __init__
        self.exp_pub.publish("...\tInitializing Freeplay annex controller... Please wait.")

        ## secondarily may publish to /maki_internal_monologue when fancied
        self.ros_pub_MIM = rospy.Publisher( "maki_internal_monologue", String, queue_size = 10)
        self.MIM_count = 0

        self.ALIVE = False
        self.__is_robot_ready = False
        self.__is_running_stimuli=False
        freeplayAnnexController.resetInteractionCount( self )

        self.state = None
        self.previous_state = None

        self.durationHeadTurn = 1.0
        self.durationWatchStimuli = 8.0
        self.blocking_gui = True

        ## instead of passing messages, instantiate the behaviors
        self.asleepAwake = asleepAwake( verbose_debug, self.ros_pub )
        self.lookIntro = lookINSPIRE4Intro( verbose_debug, self.ros_pub )
        self.startleGame = engagementStartleGame( verbose_debug, self.ros_pub )
        self.lookStimuli = lookINSPIRE4Interaction( verbose_debug, self.ros_pub )
        self.eyelids = blinking( verbose_debug, self.ros_pub )
        #self.blinking = blinking( verbose_debug, self.ros_pub )
        #self.scanning = selectiveAttention( verbose_debug, self.ros_pub )
        ## and a generic one for dealing with resetting to neutral
        self.htBB = headTiltBaseBehavior( False, self.ros_pub )

        self.data_logger_status = "unknown"
        self.start_logger_timer = None
        self.stop_logger_timer = None

        self.start_watch_timer = None
        self.stop_watch_timer = None
        return

    #def __del__(self):
    # rospy.loginfo( "signal_handler: CTRL+C" )
    # controller.controllerExit()
    # rospy.loginfo( "signal_handler: CTRL+C says goodnight" )

    def resetInteractionCount( self ):
        self.interaction_count = 0
        return

    def start( self, neutral_head=True ):
        ## always begin in neutral position
        if neutral_head:
            rospy.loginfo("start(): call to controllerReset()")
            ## 2016-07-21 ktsui: to prevent jerk, don't disable HT motor
            freeplayAnnexController.controllerReset( self, disable_ht=False )
        #self.exp_pub.publish("...\tInitializing Freeplay annex controller... DONE")

        self.__sync_count = 0
        self.__is_sync_done = False
        freeplayAnnexController.resetInteractionCount( self )
        self.state = None
        self.previous_state = None
        freeplayAnnexController.cancelWatchStimuliCallbacks( self )

        self.htBB.start()
        self.ALIVE = True
        return

    def stop( self, neutral_head=True, disable_ht=True ):
        ## STEP 1: CLEAN UP
        ## return to neutral position
        if self.previous_state == freeplayAnnexController.ENGAGEMENT:
            ## if pressed during startleGame, then stop
            self.startleGame.stopStartleGame( disable_ht=False )
        elif self.previous_state == freeplayAnnexController.STIMULI:
            freeplayAnnexController.cancelWatchStimuliCallbacks( self )
            ## also taken care of in runWatchStimuli
            ## KATE 14:39
            #rospy.sleep(1) ## need time to face front
            pass
        else:
            pass

        ## always begin in neutral position
        ## NOTE: head tilt motor will be disabled after reset
        if neutral_head:
            rospy.loginfo("stop(): call to controllerReset()")
            freeplayAnnexController.controllerReset( self, disable_ht=disable_ht )

        return

    #####################
    ## Initialize ROS node
    #####################
    def initROS( self, nodename="anon" ):
        ## get function name for logging purposes
        _fname = sys._getframe().f_code.co_name ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
        print str(_fname) + ": BEGIN" ## THIS IS BEFORE ROSNODE INIT

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
            # self.ros_pub = rospy.init_node(str(nodename), anonymous=_anon_rosnode, log_level=rospy.DEBUG)
            # rospy.logdebug("log_level=rospy.DEBUG")
            # else:
            self.ros_pub = rospy.init_node(nodename, anonymous=_anon_rosnode) ## defaults to log_level=rospy.INFO

            rospy.logdebug("anonymous=" + str(_anon_rosnode))
            rospy.loginfo( str(_fname) + ": END")
        return


    #####################
    ## Set up publisher to /maki_macro
    #####################
    def initROSPub( self, latch=False ):
        rospy.logdebug( "setup rostopic publisher to /maki_macro" )

        self.ros_pub = rospy.Publisher( "maki_macro", String, queue_size = 26, latch = latch) ## if LATCH==True, any new subscribers will see the most recent message published

        return

    def pubTo_maki_macro( self, commandOut ):
        rospy.logdebug( commandOut )
        if not rospy.is_shutdown():
            self.ros_pub.publish( commandOut )
        return

    def pubTo_maki_internal_monologue( self, thought ):
        if not rospy.is_shutdown():
            self.MIM_count += 1
            self.ros_pub_MIM.publish( "Thought #" + str(self.MIM_count) + ": " + thought )
        return

    ## Occasionally, will need to publish to self
    def pubTo_freeplay_annex_command( self, commandOut ):
        rospy.logdebug( ">>>>> SELF PUBLISHING: " + commandOut )
        if not rospy.is_shutdown():
            _ros_pub = rospy.Publisher( "freeplay_annex_command", String, queue_size = 10)
            _ros_pub.publish( commandOut )
        return

    #####################
    ## Set up subscriber to /freeplay_annex_command
    #####################
    def initROSSub( self ):
        rospy.Subscriber( "freeplay_annex_command", String, freeplayAnnexController.parse_annex_command )
        rospy.logdebug( "now subscribed to /freeplay_annex_command" )
        return


    def setBlinkAndScan( self, blink=False, auto_reset_eyelid=True, scan=False, disable_ht=False ):
        rospy.logdebug("setBlinkAndScan(): BEGIN")
        _pub_cmd = ""

        if (isinstance(blink, bool) and blink):
            freeplayAnnexController.pubTo_maki_macro( self, "spontaneousBlink start" )
        else:
            if auto_reset_eyelid:
                freeplayAnnexController.pubTo_maki_macro( self, "spontaneousBlink stop auto_reset_eyelid" )
            else:
                freeplayAnnexController.pubTo_maki_macro( self, "spontaneousBlink stop" )

        if (isinstance(scan, bool) and scan):
            freeplayAnnexController.pubTo_maki_macro( self, "visualScan start" )
        else:
            _pub_cmd = "visualScan stop"
            if not disable_ht: _pub_cmd += " disable_ht=False"
            freeplayAnnexController.pubTo_maki_macro( self, _pub_cmd )

        rospy.logdebug("setBlinkAndScan(): END")
        return


    #########################
    ##
    ## Run this for setup and ending
    ##
    #########################
    def doSetup( self ):
        rospy.logdebug("doSetup(): BEGIN")
        freeplayAnnexController.setBlinkAndScan( self, blink=False, scan=False )

        ## has own invocation to headTiltBaseBehavior.start() and .stop( disable_ht=True )
        self.asleepAwake.runAsleep()

        rospy.logdebug("doSetup(): END")
        return

    ## NOTE: This function doesn't actually do very much anymore
    def transitionToEngagement( self, msg=None ):
        rospy.logdebug("transitionToEngagement(): BEGIN")
        freeplayAnnexController.setBlinkAndScan( self, blink=False, scan=False )

        if (self.state == None) or (not isinstance(self.state, int)):
            rospy.logerr("transitionToEngagement(): ERROR: Unknown self.state: " + self.state_dict[self.state])
            rospy.logwarn("transitionToEngagement(): WARN: Expected transitions from INTRO, STIMULI, or INVALID_TRIAL")
            return

        elif self.state == freeplayAnnexController.INTRO:
            pass
        elif self.state == freeplayAnnexController.STIMULI:
            self.lookStimuli.stop( disable_ht=False )
        elif self.state == freeplayAnnexController.INVALID_TRIAL:
            pass
        else:
            rospy.logwarn("transitionToEngagement(): WARNING: Unexpect transition from self.state: " + self.state_dict[self.state])
            rospy.logwarn("transitionToEngagement(): WARN: Expected transitions from INTRO, STIMULI, or INVALID")
            return
        rospy.logdebug("transitionToEngagement(): END")
        return


    def transitionToStimuli( self ):
        rospy.logdebug("transitionToStimuli(): BEGIN")
        freeplayAnnexController.setBlinkAndScan( self, blink=False, scan=False )

        self.startleGame.stopStartleGame( disable_ht=False )
        self.startleGame.eyelidOpen() ## make sure that eyelid is open

        self.lookStimuli.start()
        self.state = freeplayAnnexController.STIMULI
        self.exp_pub.publish('[state] ' + self.state_dict[self.state])
        rospy.logdebug("transitionToStimuli(): END")
        return


    ## ------------------------------
    ## durations in seconds
    def setAutoTransitionWatchStimuli( self, durationTurnToScreen=1.0, durationWatchStimuli=8.0 ):
        rospy.logdebug("setAutoTransitionFromStimuli(): BEGIN")
        #self.durationHeadTurn = 1.0
        #self.durationWatchStimuli = 8.0

        self.start_watch_timer = rospy.Timer(rospy.Duration(durationTurnToScreen), self.startWatchStimuli_callback, oneshot=True)
        self.stop_watch_timer = rospy.Timer(rospy.Duration(durationTurnToScreen + durationWatchStimuli), self.stopWatchStimuli_callback, oneshot=True)
        rospy.logdebug("setAutoTransitionFromStimuli(): END")
        return

    def cancelWatchStimuliCallbacks( self ):
        rospy.logdebug("cancelWatchStimuliCallbacks(): BEGIN")

        ## neutralize outstanding timers
        if self.stop_watch_timer != None:
            self.stop_watch_timer.shutdown()
            self.stop_watch_timer = None
            rospy.logdebug("CANCELLED self.stop_watch_timer")

        if self.start_watch_timer != None:
            self.start_watch_timer.shutdown()
            self.start_watch_timer = None
            rospy.logdebug("CANCELLED self.start_watch_timer")

        rospy.logdebug("cancelWatchStimuliCallbacks(): END")
        return

    def startWatchStimuli_callback( self, event ):
        rospy.logdebug("startWatchStimuli(): BEGIN")
        rospy.logdebug("startWatchStimuli_callback() called at " + str( event.current_real))
        freeplayAnnexController.runWatchStimuli( self, watch=True, auto_return=False )
        rospy.logdebug("startWatchStimuli(): END")
        return

    def stopWatchStimuli_callback( self, event ):
        rospy.logdebug("stopWatchStimuli(): BEGIN")
        rospy.logdebug("stopWatchStimuli_callback() called at " + str( event.current_real))
        freeplayAnnexController.runWatchStimuli( self, watch=False, auto_return=True )
        rospy.logdebug("stopWatchStimuli(): END")
        return

    def runWatchStimuli( self, watch=True, auto_return=True ):
        rospy.logdebug("runWatchStimuli(): BEGIN")
        if self.blocking_gui: self.__is_running_stimuli=True

        if watch and not (self.state == freeplayAnnexController.INVALID_TRIAL):
            _start_time = rospy.get_time()
            #freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=True )
            freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=False )
            _elapsed_duration = rospy.get_time() - _start_time
            while ((_elapsed_duration < self.durationWatchStimuli) and
                not (self.state == freeplayAnnexController.INVALID_TRIAL) and
                not (self.state == freeplayAnnexController.READY) and
                not (self.state == freeplayAnnexController.RESET_EXP) and
                not (self.state == freeplayAnnexController.END)):
                _elapsed_duration = rospy.get_time() - _start_time
                rospy.logdebug("watching stimuli; Elapsed Duration: " + str(_elapsed_duration) + " seconds")
                self.exp_pub.publish("watching stimuli; Elapsed Duration: " + str(_elapsed_duration) + " seconds")
                rospy.sleep(1) ## sleep for 1 second

        if auto_return and not (self.state == freeplayAnnexController.INVALID_TRIAL):
            _data = "turnToInfant" ## QUICK HACK
            rospy.logdebug("====> turnToInfant")
            self.lookStimuli.turnToInfant() ## blocking call, monitorMoveToGP
            freeplayAnnexController.transitionToEngagement( self, _data )
            self.exp_pub.publish("Stimuli done... Returning to face infant... END OF INTERACTION")
            self.interaction_count += 1
            ##self.exp_pub.publish('[interaction count] ' + str(self.interaction_count))
            self.exp_pub.publish( str(self.interaction_count) + " of " + str(freeplayAnnexController.NUMBER_OF_INTERACTIONS) + " INTERACTIONS have occurred" )
            rospy.loginfo( str(self.interaction_count) + " of " + str(freeplayAnnexController.NUMBER_OF_INTERACTIONS) + " INTERACTIONS have occurred" )
            if self.interaction_count < freeplayAnnexController.NUMBER_OF_INTERACTIONS:
                pass
                ## TODO: automatically start playing startle game????
            else:
                rospy.loginfo ("======== Maximum Number of Iterations Reached: " + str(self.interaction_count))
                ## TODO: automatically end????

        self.__is_running_stimuli=False
        rospy.logdebug("runWatchStimuli(): END")
        return

    ## ------------------------------
    ## based on MAKI-WOz-INSPIRE4.py
    ##
    ## To toggle recording, pass the opposite status
    ##
    ## To start recording, pass dl_status as "stopped"
    ## To stop recording, pass dl_status as "started"
    def toggleDataLoggerRecording( self, dl_status ):
        rospy.logdebug("BEGIN")

        if ( (not dl_status.isalpha()) or (dl_status != "started") and (dl_status != "stopped") ):
            rospy.logwarn( "Invalid data_logger_status (" + str( dl_status ) + "); rosnode /freeplay_annex_data_logger may not be running")
            return

        _service_exception_flag = False

        if (dl_status == "started"):
            ## dl_status == "started", so stop /data_logger rosbag recording
            try:
                rospy.wait_for_service('/freeplay_annex_data_logger/stop', 5) ## timeout is 5 seconds
            except rospy.exceptions.ROSException as _e0:
                rospy.logerr( "Service /freeplay_annex_data_logger/stop not found: " + str(_e0) )
                rospy.logwarn( "rosnode /freeplay_annex_data_logger may not be running" )
                self.data_logger_status = "unknown"
                return
            dl_stop = rospy.ServiceProxy('/freeplay_annex_data_logger/stop', Empty)
            try:
                rospy.logdebug( "calling rosservice /freeplay_annex_data_logger/stop..." )
                dl_stop_resp = dl_stop()
            except rospy.ServiceException as _e1:
                rospy.logwarn( "Service /freeplay_annex_data_logger/stop did not process request: " + str(_e1) )
                _service_exception_flag = True
            else:
                rospy.logdebug( "calling rosservice /freeplay_annex_data_logger/stop... SUCCESS" )

        else:
            ## dl_status == "stopped", so start /data_logger rosbag recording
            try:
                rospy.wait_for_service('/freeplay_annex_data_logger/start', 5) ## timeout = 5 seconds
            except rospy.exceptions.ROSException as _e2:
                rospy.logerr( "Service /freeplay_annex_data_logger/start not found: " + str(_e2) )
                rospy.logwarn( "rosnode /freeplay_annex_data_logger may not be running" )
                self.data_logger_status = "unknown"
                return
            dl_start = rospy.ServiceProxy('/freeplay_annex_data_logger/start', Empty)
            try:
                rospy.logdebug( "calling rosservice /freeplay_annex_data_logger/start..." )
                dl_start_resp = dl_start()
            except rospy.ServiceException as _e3:
                rospy.logwarn( "Service /freeplay_annex_data_logger/start did not process request: " + str(_e3) )
                _service_exception_flag = True
            else:
                rospy.logdebug( "calling rosservice /freeplay_annex_data_logger/start... SUCCESS" )

        if _service_exception_flag:
            if self.data_logger_status == "started":
                rospy.logwarn( "Unable to toggle rosbag recording to ON... Recording remains " + str(self.data_logger_status) )
            elif self.data_logger_status == "stopped":
                rospy.logwarn( "Unable to toggle rosbag recording to OFF... Recording remains " + str(self.data_logger_status) )
            else:
                rospy.logwarn( "Unable to toggle rosbag recording... INVALID STATUS; self.data_logger_status = " + str(self.data_logger_status) )

        rospy.logdebug("END")
        return


    ## self.data_logger_status is only updated by the value
    ## published to rostopic /data_logger/status
    ## by calling its start/stop services
    def updateDataLoggerStatus_callback( self, msg ):
        rospy.logdebug( "updateDataLoggerStatus_callback(): BEGIN" )
        _data = str( msg.data )
        if ((not _data.isalpha) or ((_data != "started") and (_data != "stopped"))):
            rospy.logwarn( "INVALID INPUT: expected string 'started' or 'stopped'; received " + str( _data ) )
            rospy.logdebug( "self.data_logger_status remains " + str(self.data_logger_status) )
            return

        if (_data != self.data_logger_status):
            rospy.logdebug( "Updating self.data_logger_status to " + str(_data) + " (from " + str(self.data_logger_status) + ")" )
            self.data_logger_status = _data

            if self.data_logger_status == "started":
                rospy.loginfo( "Toggle rosbag recording to ON" )
                self.exp_pub.publish( "Toggle rosbag recording to ON" )
            else:
                rospy.loginfo( "Toggle rosbag recording to OFF" )
                self.exp_pub.publish( "Toggle rosbag recording to OFF" )

        rospy.logdebug( "updateDataLoggerStatus_callback(): END" )
        return


    ## durations in seconds
    def setAutoDataLogging( self, durationToAutoOff=None, durationToAutoOn=None ):
        rospy.logdebug( "setAutoDataLogging(): BEGIN" )

        if ((durationToAutoOn != None) and (isinstance(durationToAutoOn, float) or isinstance(durationToAutoOn, int))):
            self.start_logger_timer = rospy.Timer( rospy.Duration(durationToAutoOn), self.startAutoDataLogger_callback, oneshot=True)
            rospy.logdebug("CREATED self.start_logger_timer; will fire in " + str(durationToAutoOn) + " seconds...")

        if ((durationToAutoOff != None) and (isinstance(durationToAutoOff, float) or isinstance(durationToAutoOff, int))):
            self.stop_logger_timer = rospy.Timer( rospy.Duration(durationToAutoOff), self.stopAutoDataLogger_callback, oneshot=True)
            rospy.logdebug("CREATED self.stop_logger_timer; will fire in " + str(durationToAutoOff) + " seconds")

        rospy.logdebug( "setAutoDataLogging(): END" )
        return


    def startAutoDataLogger_callback( self, event ):
        rospy.logdebug("BEGIN")
        if self.start_logger_timer != None:
            rospy.logdebug("startAutoDataLogger_callback() called at " + str( event.current_real))
            freeplayAnnexController.toggleDataLoggerRecording( self, "stopped" ) ## we want to start recording
        else:
            rospy.logdebug("INVALID ACTION: self.start_logger_timer = " + str(self.start_logger_timer))
        rospy.logdebug("END")
        return


    def stopAutoDataLogger_callback( self, event ):
        rospy.logdebug("BEGIN")
        if self.stop_logger_timer != None:
            rospy.loginfo("stopAutoDataLogger_callback() called at " + str( event.current_real))
            freeplayAnnexController.toggleDataLoggerRecording( self, "started" ) ## we want to stop recording
        else:
            rospy.logdebug("INVALID ACTION: self.stop_logger_timer = " + str(self.stop_logger_timer))
        rospy.logdebug("END")
        return


    def cancelAutoDataLoggerCallbacks( self ):
        rospy.logdebug("cancelAutoDataLoggerCallbacks(): BEGIN")

        ## neutralize outstanding timers
        if self.stop_logger_timer != None:
            self.stop_logger_timer.shutdown()
            self.stop_logger_timer = None
            rospy.logdebug("CANCELLED self.stop_logger_timer")

        if self.start_logger_timer != None:
            self.start_logger_timer.shutdown()
            self.start_logger_timer = None
            rospy.logdebug("CANCELLED self.start_logger_timer")

        rospy.logdebug("cancelAutoDataLoggerCallbacks(): END")
        return


    ## ------------------------------
    ## stand alone timed skit
    ## "Friend" actor will adapt to Maki-ro's skit
    def runFamiliarizationSkit( self ):
        rospy.logdebug("runFamiliarizationSkit(): BEGIN")
        self.exp_pub.publish('familiarization skit: BEGIN... Please wait.')

        _verbose = True
        _thought = "" ## start off not thinking of anything, beginner's mind

        #AA = asleepAwake( _verbose, self.ros_pub )
        #lookIntro = lookINSPIRE4Intro( _verbose, self.ros_pub )

        ## STEP -1: --- PRE-CHECK ---
        ## Assumes that Maki-ro begins in the asleep position
        #rospy.logdebug( "...\tPRE-CHECK...\tIs Maki-ro already sleeping? AA.asleep_p() = " + str( AA.asleep_p() ) )
        rospy.logdebug( "...\tPRE-CHECK...\tIs Maki-ro already sleeping? self.asleepAwake.asleep_p() = " + str( self.asleepAwake.asleep_p() ) )
        #if AA.asleep_p():
        if self.asleepAwake.asleep_p():
            self.publishMonologue('runFamiliarizationSkit', "Maki-ro is already asleep... ZZZzzzz ZZZZzzzzzz...")
            pass
        else:
            self.publishMonologue('runFamiliarizationSkit', "Maki-ro is not yet asleep... Going to sleep...")
            ## TODO: Add timers to see how long each step takes
            #_start_time = rospy.get_time()

            ## blocking until Maki-ro is in the asleep position
            #AA.runAsleep()
            self.asleepAwake.runAsleep()

        ## STEP 0: Officially declare the familiarization begun
        #lookIntro.introStart()
        self.lookIntro.introStart()

        ## STEP 1: Friend touches Maki-ro on the shoulder
        pass

        ## STEP 2: Maki-ro wakes up and faces Friend
        self.publishMonologue('runFamiliarizationSkit', "Maki-ro, wake up!!! Face Friend...")

        self.asleepAwake.runAwakeExperimenter( disable_ht=False )
        rospy.logdebug("...\tMaki-ro should now be awake... self.asleepAwake.awake_p()=" + str( self.asleepAwake.awake_p() ))

        ## STEP 3: Experimenter greets Maki-ro
        rospy.sleep(0.75) ## 750 ms

        ## STEP 4: Maki-ro acknowledges greeting
        self.publishMonologue('runFamiliarizationSkit', "Maki-ro, mind your manners and say hello...")

        self.lookIntro.macroGreeting()

        ## STEP 5: Friend plays peek-a-boo. Covers own eyes with hands, then uncover
        rospy.sleep(0.75) ## 750 ms

        ## STEP 6: Maki-ro reacts with a startle expression and immediately relaxes
        self.publishMonologue('runFamiliarizationSkit', "Maki-ro is startled!!! Peek-a-boo does that...")

        self.lookIntro.startStartle( relax=True )

        ## STEP 7: Friend show Maki-ro the flashing ball
        rospy.sleep(0.5) ## 500 ms

        ## TODO: Insert agentic behaviors where appropriate
        #freeplayAnnexController.setBlinkAndScan( self, blink=True )

        ## STEP 8: Maki-ro nods
        self.publishMonologue('runFamiliarizationSkit', "Oooo a blinky... Maki-ro tries to nod enthusiastically...")
        ## This hack works better than specifying repetitions as optional input parameter
        #lookIntro.macroGreeting()
        self.lookIntro.macroGreeting()
        rospy.sleep(0.5)
        self.lookIntro.macroGreeting()
        self.publishMonologue('runFamiliarizationSkit', "Maki-ro might be a little dizzy now...")

        ## STEP 9: Friend moves the flashing ball to the UPPER RIGHT calibration point
        ## and jiggles it to attract Maki-ro's attention
        rospy.sleep(0.5)

        ## STEP 10: Maki-ro looks at the upper right calibration point
        self.publishMonologue('runFamiliarizationSkit', "Maki-ro likes the fun flashy ball and here comes faux smooth pursuit...")
        self.lookIntro.macroLookAtBallLocationRight( upper=True )

        ## STEP 11: Friend moves the flashing ball to the LOWER RIGHT calibration point
        ## and jiggles it to attract Maki-ro's attention
        rospy.sleep(0.5)

        ## STEP 12: Maki-ro looks at the lower right calibration point
        self.publishMonologue('runFamiliarizationSkit', "Maki-ro thinks the flashy ball is pretty... There Maki-ro goes chasing the ball...")
        self.lookIntro.macroLookAtBallLocationRight( lower=True )

        ## STEP 13: Friend moves the flashing ball between the infant and Maki-ro
        ## and jiggles it to attract Maki-ro's attention
        #rospy.sleep(0.5)

        ## KATE 14:39
        ### STEP 14: Maki-ro looks at the infant
        self.publishMonologue('runFamiliarizationSkit', "Maki-ro... Follow, follow, follow, follow the flashy ball... to in front of the infant...")
        self.lookIntro.macroLookAtInfant()

        # STEP 15: Friend retracts the flashy ball wand
        rospy.sleep(0.75)

        ## STEP 16: Maki-ro follows the ball and faces Friend
        self.publishMonologue('runFamiliarizationSkit', "Maki-ro... Follow and follow, follow the flashy ball... Oh hiya, Friend!!!...")
        self.lookIntro.macroLookAtExperimenter()

        ## STEP 17: Maki-ro watches Friend leave
        self.publishMonologue('runFamiliarizationSkit', "Hey, wait!!! Where did Friend go?? Maki-ro was playing with Friend...")
        #rospy.sleep(0.5)
        ## KATE 14:39
        rospy.sleep(1.0)

        ## STEP 18: Maki-ro turns back to Infant
        self.publishMonologue('runFamiliarizationSkit', "Maki-ro misses Friend... Sad... Lonely... Who will play with Maki-ro now??...")
        self.lookIntro.macroLookAtInfant()

        ## STEP 19: Maki-ro makes a new pal
        self.publishMonologue('runFamiliarizationSkit', "Maki-ro spies with its eyes... a human baby!?!?")
        rospy.sleep(0.5)

        ## STEP 20: END OF FAMILIARIZATION
        self.publishMonologue('runFamiliarizationSkit', "<<<< END SCENE >>>>")
        self.lookIntro.stop( disable_ht=False )
        ## disable_ht might have to change to False once in the
        ## larger context of the experiment

        rospy.logdebug("runFamiliarizationSkit(): END")
        self.exp_pub.publish('familiarization skit: END')
        return

    def publishMonologue(self, func, thought):
        freeplayAnnexController.pubTo_maki_internal_monologue( self, thought )
        rospy.logdebug(func + '(): ' + thought)
        return

    ## ------------------------------
    def transitionToReady( self, msg=None ):
        rospy.logdebug("transitionToReady: BEGIN")

        ## STEP 0: Fall asleep
        freeplayAnnexController.doSetup( self )

        ## STEP 1: Stop logging
        if msg != None: rospy.loginfo( "ADD SYNC MARKER: " + str(msg) ) ## add BEFORE stop recording
        freeplayAnnexController.toggleDataLoggerRecording( self, "started" ) ## we want to stop the rcording
        freeplayAnnexController.cancelAutoDataLoggerCallbacks( self )

        ### STEP 2: Reset number of interactions
        #freeplayAnnexController.resetInteractionCount( self )

        ## STEP 3: Update state
        self.state = freeplayAnnexController.READY
        self.exp_pub.publish('[state] ' + self.state_dict[self.state])
        rospy.logdebug("transitionToReady: END")
        return

    def transitionToIntro( self, blocking=False ):
        if not blocking:
            try:
                thread.start_new_thread( freeplayAnnexController.runFamiliarizationSkit, ( self,  ) )
                self.state = freeplayAnnexController.INTRO
                self.exp_pub.publish('[state] ' + self.state_dict[self.state])
            except Exception as _e_rFS:
                rospy.logerr("Unable to start new thread for freeplayAnnexController.runFamiliarizationSkit()")
        else:
            ## BLOCKING!!!!
            freeplayAnnexController.runFamiliarizationSkit( self )
            self.state = freeplayAnnexController.INTRO
            self.exp_pub.publish('[state] ' + self.state_dict[self.state])

        return

    def transitionUsage( self, state, prefix_msg="" ):

        if (state == None):
            rospy.loginfo("[usage] Press 'Get ready'")
            self.exp_pub.publish( prefix_msg + "[usage] Press 'Get ready'")

## 2016-06-16, KATE
        ## We should be able to get to these states at any time
        #if ((state == freeplayAnnexController.READY) or
        # (state == freeplayAnnexController.RESET_EXP) or
        # (state == freeplayAnnexController.END)):
        # rospy.loginfo("[usage] You can transition from any state to this state '" + self.state_dict[state] + "' anytime")
        # self.exp_pub.publish( prefix_msg + "[usage] You can transition from any state to this state '" + self.state_dict[state] + "' anytime")

        ## We should be able to transition into this state from EVERY state
        if (state == freeplayAnnexController.READY):
            rospy.loginfo("[usage] You can transition from every state (including itself) to this state '" + self.state_dict[state] + "' anytime")
            self.exp_pub.publish("[usage] You can transition from every state (including itself) to this state '" + self.state_dict[state] + "' anytime")

        ## We should be able to transition into this state from ANY OTHER
        if ((state == freeplayAnnexController.RESET_EXP) or
            (state == freeplayAnnexController.END)):
            rospy.loginfo("[usage] You can transition from any state (except itself) to this state '" + self.state_dict[state] + "' anytime")
            self.exp_pub.publish("[usage] You can transition from any state (except itself) to this state '" + self.state_dict[state] + "' anytime")


        if (state == freeplayAnnexController.READY):
            rospy.loginfo("[usage] From state '" + self.state_dict[self.state] + "', you can choose to press buttons: 'Tobii verify *' or 'Visual clap sync'")
            self.exp_pub.publish( prefix_msg + "[usage] From state '" + self.state_dict[self.state] + "', you can choose to press buttons: 'Tobii verify *' or 'Visual clap sync'")

        if (state == freeplayAnnexController.SYNC):
            if self.__is_sync_done:
                rospy.loginfo("[usage] From state '" + self.state_dict[self.state] + "' if all 3 'Tobii verify' buttons and 'Visual clap sync' buttons have been pressed, you can choose to press button 'Run Familiarization Skit'")
                self.exp_pub.publish( prefix_msg + "[usage] From state '" + self.state_dict[self.state] + "' if all 3 'Tobii verify' buttons and 'Visual clap sync' buttons have been pressed, you can choose to press button 'Run Familiarization Skit'")
            else:
                rospy.loginfo("[usage] From state '" + self.state_dict[self.state] + "', you can choose to press buttons: 'Tobii verify *', 'Visual clap sync', or 'Run Familiarization Skit'")
                self.exp_pub.publish( prefix_msg + "[usage] From state '" + self.state_dict[self.state] + "', you can choose to press buttons: 'Tobii verify *', 'Visual clap sync', or 'Run Familiarization Skit'")

        if (state == freeplayAnnexController.INTRO):
            rospy.loginfo("[usage] From state '" + self.state_dict[self.state] + "', you can choose to press button 'Run Engagement Game'")
            self.exp_pub.publish( prefix_msg + "[usage] From state '" + self.state_dict[self.state] + "', you can choose to press button 'Run Engagement Game'")

        if (state == freeplayAnnexController.ENGAGEMENT):
            rospy.loginfo("[usage] From state '" + self.state_dict[self.state] + "', you can choose to press buttons: 'Turn to LEFT SCREEN', 'Turn to RIGHT SCRREN', or 'Invalid Trial'")
            self.exp_pub.publish( prefix_msg + "[usage] From state '" + self.state_dict[self.state] + "', you can choose to press buttons: 'Turn to LEFT SCREEN', 'Turn to RIGHT SCRREN', or 'Invalid Trial'")

        if (state == freeplayAnnexController.STIMULI):
            rospy.loginfo("[usage] From state '" + self.state_dict[self.state] + "', you can choose to press buttons: 'Run Engagement Game', or 'Invalid Trial'")
            self.exp_pub.publish( prefix_msg + "[usage] From state '" + self.state_dict[self.state] + "', you can choose to press buttons: 'Run Engagement Game', or 'Invalid Trial'")

        if (state == freeplayAnnexController.INVALID_TRIAL):
            rospy.loginfo( prefix_msg + "[usage] From state '" + self.state_dict[self.state] + "', you can choose to press button 'Run Engagement Game'")
            self.exp_pub.publish("[usage] From state '" + self.state_dict[self.state] + "', you can choose to press button 'Run Engagement Game'")

        return


    def invalidTransition( self, current_state, future_state ):
        _invalid_transition = True

        #if (self.state == None) or (not isinstance(self.state, int)):
        # rospy.logerr("transitionToEngagement(): ERROR: Unknown self.state: " + self.state_dict[self.state])
        # rospy.logwarn("transitionToEngagement(): WARN: Expected transitions from INTRO, STIMULI, or INVALID_TRIAL")
        # return


## 2016-06-16, KATE
        ## We should be able to get to these states at any time
        #if ((future_state == freeplayAnnexController.READY) or
        # (future_state == freeplayAnnexController.RESET_EXP) or
        # (future_state == freeplayAnnexController.END)):
        # rospy.loginfo("Transitions from any state to state " + self.state_dict[future_state] + " are VALID anytime")
        # _invalid_transition = False

        if (future_state == freeplayAnnexController.READY):
            rospy.loginfo("Transitions from any state to state " + self.state_dict[future_state] + " are VALID anytime")
            _invalid_transition = False

        if (future_state == freeplayAnnexController.RESET_EXP):
            ## Don't allow clicking multiple times on "reset experiment"
            if (current_state == freeplayAnnexController.RESET_EXP):
                _invalid_transition = True
            ## But otherwise, we should be able to get to this state from any other
            else:
                _invalid_transition = False

        if (future_state == freeplayAnnexController.END):
            ## Don't allow clicking multiple times on "the end"
            if (current_state == freeplayAnnexController.END):
                _invalid_transition = True
            ## But otherwise, we should be able to get to this state from any other
            else:
                _invalid_transition = False

        if (future_state == freeplayAnnexController.SYNC):
            if (current_state == freeplayAnnexController.READY):
                _invalid_transition = False
            elif (current_state == freeplayAnnexController.RESET_EXP):
                _invalid_transition = False
            elif (current_state == freeplayAnnexController.SYNC):
                _invalid_transition = False
            else:
                #if self.__is_sync_done:
                # rospy.logwarn("Invalid transition... If all 3 'Tobii verify' buttons and 'Visual clap sync' buttons have been pressed, you can choose to press button 'Run Familiarization Skit'")
                #else:
                # rospy.logwarn("Invalid transition... You can choose to press buttons: 'Tobii verify *' or  'Visual clap sync'")
                rospy.logwarn("Invalid transition... TODO: USAGE MESSAGE'")
                _invalid_transition = True

        if (future_state == freeplayAnnexController.INTRO):
            if (current_state == freeplayAnnexController.SYNC):
                _invalid_transition = False
            else:
                #rospy.logwarn("Invalid transition... You can choose to press button 'Run Familiarization Skit'")
                rospy.logwarn("Invalid transition... TODO: USAGE MESSAGE'")
                _invalid_transition = True

        if (future_state == freeplayAnnexController.ENGAGEMENT):
            if (current_state == freeplayAnnexController.INTRO):
                _invalid_transition = False
            elif (current_state == freeplayAnnexController.STIMULI):
                _invalid_transition = False
            elif (current_state == freeplayAnnexController.INVALID_TRIAL):
                _invalid_transition = False
            else:
                rospy.logwarn("Invalid transition... TODO: USAGE MESSAGE'")
                _invalid_transition = True

        if (future_state == freeplayAnnexController.STIMULI):
            if (current_state == freeplayAnnexController.ENGAGEMENT):
                _invalid_transition = False
            else:
                rospy.logwarn("Invalid transition... TODO: USAGE MESSAGE'")
                _invalid_transition = True

        if (future_state == freeplayAnnexController.INVALID_TRIAL):
            if (current_state == freeplayAnnexController.ENGAGEMENT):
                _invalid_transition = False
            elif (current_state == freeplayAnnexController.STIMULI):
                _invalid_transition = False
            else:
                rospy.logwarn("Invalid transition... TODO: USAGE MESSAGE'")
                _invalid_transition = True

        return _invalid_transition


    def hide( self, ipt_ms=750 ):
        _pub_cmd = ""
        _pub_cmd += "HT" + SC_SET_GP + str(HT_DOWN)
        _pub_cmd += "LL" + SC_SET_GP + str(LL_CLOSE_MAX)
        _pub_cmd += SC_SET_IPT + str(ipt_ms)
        _pub_cmd += TERM_CHAR_SEND

        self.htBB.pubTo_maki_command( _pub_cmd )
        return

    ## TO FIX: HANDLE QUEUED MESSAGES. VALID STATE TRANSITIONS
    ## ARE POSSIBLE AND MAY YIELD CONCURRENTLY EXECUTING
    ## BEHAVIORS, e.g., watchStimuli and engagementGame
    def parse_annex_command( self, msg ):
        rospy.logdebug("parse_annex_command(): BEGIN")
        rospy.logdebug("received: " + str(msg))

        self.previous_state = self.state ## for later comparison
        _unknown_flag = False
        _invalid_transition = False
        
        _data_list = str(msg.data).split(':')
        rospy.loginfo(msg)
        try:
            _data = _data_list[0]
            _data_list2 = _data_list[1].split()
            _msg_id  = _data_list2[0]
            _estimate = _data_list[1]
        except IndexError:
             rospy.logerr("ERROR: Msg not well formed!")
             _data = str(msg.data)
             _msg_id = None
             _estimate = None
             #return
        rospy.logdebug("_data = " + _data)
        vh_pub =rospy.Publisher("behavior_status",String,queue_size=10)

        vh_pub.publish("{} START {}".format(_msg_id,_estimate))
        if _data == "start":
            ## There is no actively recording rosbag,
            ## so start a new one
            freeplayAnnexController.toggleDataLoggerRecording( self, "stopped" ) ## we want to start recording
            rospy.sleep(0.5) ## 0.5 second delay to allow time to open rosbag
            freeplayAnnexController.toggleDataLoggerRecording( self, "stopped" ) ## we want to start recording
            rospy.sleep(0.5) ## 0.5 second delay to allow time to open rosbag
            rospy.loginfo("==== rosbag recording STARTED ====")

            ## DON'T ENABLE HEAD TILT SERVO
            ## Done during awake behavior
            self.lookIntro.introStart( enable_ht=False )
            
            self.lookStimuli.start( enable_ht=False, auto_face_infant=False )
            self.lookStimuli.waitForMovementToComplete()

            freeplayAnnexController.setBlinkAndScan( self, blink=False, auto_reset_eyelid=False, scan=False )

        elif _data == "end":
            freeplayAnnexController.setBlinkAndScan( self, blink=False, auto_reset_eyelid=False, scan=False )
            self.lookIntro.stop()
            self.lookStimuli.stop()
            self.lookStimuli.waitForMovementToComplete()

            rospy.loginfo("==== rosbag recording STOPPED ====")
            ## There is an actively recording rosbag, so close the existing one
            freeplayAnnexController.toggleDataLoggerRecording( self, "started" ) ## we want to stop recording
            freeplayAnnexController.cancelAutoDataLoggerCallbacks( self )
            rospy.sleep(0.5) ## 0.5 second delay to allow time to close rosbag

        elif _data == "asleep":
            freeplayAnnexController.setBlinkAndScan( self, blink=False, scan=False )
            ## has own invocation to headTiltBaseBehavior.start() and .stop( disable_ht=True )
            self.asleepAwake.runAsleep()
            self.asleepAwake.waitForMovementToComplete()

        elif _data == "awake":
            ## AWAKE TO FACE INFANT
            freeplayAnnexController.toggleDataLoggerRecording( self, "stopped" ) ## we want to start recording
            self.asleepAwake.runAwake( disable_ht=False )
            self.asleepAwake.waitForMovementToComplete()
            freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=False )

        elif (_data == "greeting" or _data == "headnod"):
            ## ONE HEAD NOD
            self.lookIntro.macroGreeting()
            self.lookIntro.waitForMovementToComplete()
            freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=False )

        elif _data == "startle":
            freeplayAnnexController.setBlinkAndScan( self, blink=False, auto_reset_eyelid=False, scan=False )
            ## ONE STARTLE WITH RELAX
            self.lookIntro.macroStartleRelax( startle=True, relax=True )
            self.lookIntro.waitForMovementToComplete()
            freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=False )

        elif (_data == "startle start" or _data == "startle hold"):
            freeplayAnnexController.setBlinkAndScan( self, blink=False, auto_reset_eyelid=False, scan=False )
            ## ONE STARTLE WITHOUT RELAX
            self.lookIntro.macroStartleRelax( startle=True, relax=False )
            self.lookIntro.waitForMovementToComplete()

        elif (_data == "startle stop" or _data == "startle relax"):
            ## STARTLE RELAX ONLY
            self.lookIntro.macroStartleRelax( startle=False, relax=True )
            self.lookIntro.waitForMovementToComplete()
            freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=False )

        elif _data == "hideFromStartle":
            #self.startleGame.hideFromStartle() ## requires context of the game
            freeplayAnnexController.hide( self )
            self.lookIntro.waitForMovementToComplete()

        elif _data == "hide":
            freeplayAnnexController.setBlinkAndScan( self, blink=False, auto_reset_eyelid=False, scan=False )
            freeplayAnnexController.hide( self )
            self.lookIntro.waitForMovementToComplete()

        elif _data == "unhideIntoStartle":
            self.startleGame.unhideIntoStartle( unhide=True, startle=True )
            self.startleGame.waitForMovementToComplete()

        elif _data == "unhide":
            self.startleGame.unhideIntoStartle( unhide=True, startle=False )
            self.startleGame.waitForMovementToComplete()
            freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=False )

        elif _data == "lookAt Alyssa":
            freeplayAnnexController.setBlinkAndScan( self, blink=False, scan=False )
            ## NOTE: This has blocking call (monitorMoveToGP)
            self.lookStimuli.turnToScreen( right_screen=True )
            freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=False )
        elif _data == "lookAt Left":
            freeplayAnnexController.setBlinkAndScan( self, blink=False, scan=False )
            ## NOTE: This has blocking call (monitorMoveToGP)
            self.lookStimuli.turnToScreen( right_screen=False )
            freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=False )

        elif _data == "lookAt infant":
            freeplayAnnexController.setBlinkAndScan( self, blink=False, scan=False )
            self.lookStimuli.turnToInfant() ## blocking call, monitorMoveToGP
            freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=False )

        elif _data == "reset neutral":
            freeplayAnnexController.setBlinkAndScan( self, blink=False, auto_reset_eyelid=False, scan=False )
            freeplayAnnexController.controllerReset( self, disable_ht=False )
            self.lookStimuli.waitForMovementToComplete()
            #freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=False )

        elif _data == "reset eyelids":
            freeplayAnnexController.setBlinkAndScan( self, blink=False, auto_reset_eyelid=False, scan=False )
            freeplayAnnexController.eyelids.setEyelidNeutralPose( self, LL_OPEN_DEFAULT, cmd_prop=True, monitor=True )
            self.lookStimuli.waitForMovementToComplete()

        elif _data == "spontaneousBlink start":
            freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=False )

        elif _data == "spontaneousBlink stop":
            freeplayAnnexController.setBlinkAndScan( self, blink=False, scan=False )

        elif _data == "selectiveAttention start":
            #freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=True )
            freeplayAnnexController.setBlinkAndScan( self, blink=False, scan=True )
            freeplayAnnexController.eyelids.setEyelidNeutralPose( self, LL_OPEN_DEFAULT, cmd_prop=True, monitor=True )
            self.lookStimuli.waitForMovementToComplete()

        elif _data == "selectiveAttention stop":
            freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=False )

        else:
            rospy.logwarn("[WARNING] UNKNOWN ANNEX COMMAND: " + _data)
            freeplayAnnexController.setBlinkAndScan( self, blink=True, scan=False )
        vh_pub.publish("{} COMPLETED".format(_msg_id))
        
        '''
        if _data != 'turnToInfant':
            self.exp_pub.publish('[button pressed] ' + _data)


        if _data == "reset experiment":
            ## STEP 0:
            ## We should always be able to get to this controller state from ANY other
            _invalid_transition = freeplayAnnexController.invalidTransition( self, self.state, freeplayAnnexController.RESET_EXP )

            ## TODO: Could depend on previous state
            ## need to issue '* stop'

            if not _invalid_transition:
                self.exp_pub.publish('[RESET] resetting experiment...')

                ## STEP 0: Reset self.state and self.previous_state
                ## Reset interaction count
                ## Reset to neutral pose
                ## 2016-07-21 ktsui: to prevent jerk, don't disable HT motor
                freeplayAnnexController.stop( self, disable_ht=False )
                freeplayAnnexController.start( self )

                ## STEP 1: Move to ready state
                freeplayAnnexController.transitionToReady( self, msg=_data )

                ## override state
                self.state = freeplayAnnexController.RESET_EXP

        elif _data == "get ready":
            ## We should always be able to get to this controller state from ANY other
            _invalid_transition = freeplayAnnexController.invalidTransition( self, self.state, freeplayAnnexController.READY )

            if not _invalid_transition:
                ### STEP 0: Reset but don't move to neutral head pose first
                #freeplayAnnexController.stop( self, neutral_head=False )
                #freeplayAnnexController.start( self, neutral_head=False )
                #
                ### STEP 1: Move to ready state
                #freeplayAnnexController.transitionToReady( self, msg=_data )
                freeplayAnnexController.doGetReady( self, msg=_data )

        elif _data.startswith( "sync" ):
            ## We should only be able to get to this controller state from READY
            ## or if in SYNC state since there are multiple sync points to ePrime
            _invalid_transition = freeplayAnnexController.invalidTransition( self, self.state, freeplayAnnexController.SYNC )
            #if ((self.previous_state != freeplayAnnexController.READY) and
            # (self.previous_state != freeplayAnnexController.SYNC)):
            # rospy.logwarn("INVALID STATE TRANSITION: Expected to enter state SYNC from READY or SYNC...\tcurrent STATE = " + self.state_dict[self.state])
            # _invalid_transition = True
            # self.exp_pub.publish('[WARNING] Invalid state transition at (' + self.state_dict[self.state] + ')')
            # ## TODO: auto fix prior state

            #elif (self.previous_state == freeplayAnnexController.READY):
            if (not _invalid_transition) and (self.previous_state == freeplayAnnexController.READY):
                if self.data_logger_status == "started":
                    ## There is an actively recording rosbag,
                    ## so close the existing one and start a new one
                    rospy.loginfo( "ADD SYNC MARKER: " + str(_data) ) ## add BEFORE stop recording
                    freeplayAnnexController.toggleDataLoggerRecording( self, "started" ) ## we want to stop recording
                    freeplayAnnexController.cancelAutoDataLoggerCallbacks( self )
                    rospy.sleep(0.5) ## 0.5 second delay to allow time to close rosbag
                    freeplayAnnexController.toggleDataLoggerRecording( self, "stopped" ) ## we want to start recording
                    ## resend this message for synchronization purposes
                    _msg = _data + " (resend to mark the new rosbag)"
                    freeplayAnnexController.pubTo_freeplay_annex_command( self, _msg )
                else:
                    ## There is no actively recording rosbag,
                    ## so start a new one
                    freeplayAnnexController.toggleDataLoggerRecording( self, "stopped" ) ## we want to start recording
                    rospy.sleep(0.5) ## 0.5 second delay to allow time to open rosbag
                    ## resend this message for synchronization purposes
                    _msg = _data + " (resend to mark the new rosbag)"
                    freeplayAnnexController.pubTo_freeplay_annex_command( self, _msg )

            else:
                pass

            if _invalid_transition:
                pass ## jump past this logic
            elif _data.endswith( "Tobii calibration start" ):
                ## no longer shown in pilt GUI
                pass
            elif _data.endswith( "Tobii calibration done" ):
                ## no longer shown in pilt GUI
                pass
            elif _data.endswith( "visual clap" ):
                self.__sync_count += 1
                pass
            elif _data.endswith( "Tobii verify left screen" ):
                self.__sync_count += 1
                pass
            elif _data.endswith( "Tobii verify maki" ):
                self.__sync_count += 1
                pass
            elif _data.endswith( "Tobii verify right screen" ):
                self.__sync_count += 1
                pass
            else:
                _unknown_flag = True

            if (not _unknown_flag) and (not _invalid_transition):
                rospy.loginfo( "ADD SYNC MARKER: " + str(_data) )
                self.state = freeplayAnnexController.SYNC
                self.exp_pub.publish('[state] ' + self.state_dict[self.state])

                if self.__sync_count == 4:
                    self.__is_sync_done = True
                    rospy.loginfo("If all 3 'Tobii verify' buttons and 'Visual clap sync' buttons have been pressed, you can choose to press button 'Run Familiarization Skit'")
                    self.exp_pub.publish("If all 3 'Tobii verify' buttons and 'Visual clap sync' buttons have been pressed, you can choose to press button 'Run Familiarization Skit'")

        elif _data == "runFamiliarizationSkit":
            ## We should only be able to get to this controller state from SYNC
            _invalid_transition = freeplayAnnexController.invalidTransition( self, self.state, freeplayAnnexController.INTRO )
            #if (self.previous_state != freeplayAnnexController.SYNC):
            # rospy.logwarn("INVALID STATE TRANSITION: Expected to enter state INTRO from SYNC...\tcurrent STATE = " + self.state_dict[self.state])
            # _unknown_flag = True
            # self.exp_pub.publish('[WARNING] Invalid state transition at (' + self.state_dict[self.state] + ')')
            # ## TODO: auto fix prior state

            if not _invalid_transition:
                self.exp_pub.publish('[state] run familiarization skit')
                freeplayAnnexController.transitionToIntro( self, self.blocking_gui )


        elif (_data == "startleGame start" ):
            ## We should only be able to get to this controller state from
            ## INTRO, STIMULI, or INVALID_TRIAL
            _invalid_transition = freeplayAnnexController.invalidTransition( self, self.state, freeplayAnnexController.ENGAGEMENT )
            #if ((self.previous_state != freeplayAnnexController.INTRO) and
            # (self.previous_state != freeplayAnnexController.STIMULI) and
            # (self.previous_state != freeplayAnnexController.INVALID_TRIAL)):
            # rospy.logwarn("INVALID STATE TRANSITION: Expected to enter state ENGAGEMENT from INTRO, STIMULI, or INVALID_TRIAL...\tcurrent STATE = " + self.state_dict[self.state])
            # _unknown_flag = True
            # self.exp_pub.publish('[WARNING] Invalid state transition at (' + self.state_dict[self.state] + ')')
            # ## TODO: auto fix prior state

            if self.__is_running_stimuli:
                rospy.logerr("Do NOT attempt to interrupt the turnToScreen stimuli!!!")
                _unknown_flag = True

            if (not _invalid_transition) and (not _unknown_flag):
                self.exp_pub.publish('[state] startle game start')
                rospy.loginfo("Start engagement game; forward the message contents to /maki_macro: " + _data)
                self.startleGame.startStartleGame() ## runs game in new thread
                self.state = freeplayAnnexController.ENGAGEMENT
                self.exp_pub.publish('[state] ' + self.state_dict[self.state])

        elif ("turnToScreen" in _data):
            ## We should only be able to get to this controller state from ENGAGEMENT
            _invalid_transition = freeplayAnnexController.invalidTransition( self, self.state, freeplayAnnexController.STIMULI )
            #if (self.previous_state != freeplayAnnexController.ENGAGEMENT):
            # rospy.logwarn("INVALID STATE TRANSITION: Expected to enter state STIMULI from ENGAGEMENT...\tcurrent STATE = " + self.state_dict[self.state])
            # _unknown_flag = True
            # self.exp_pub.publish('[WARNING] Invalid state transition at (' + self.state_dict[self.state] + ')')
            # ## TODO: auto fix prior state

            _right_screen = None
            if ("left" in _data):
                _right_screen = False
            elif ("right" in _data):
                _right_screen = True
            else:
                _unknown_flag = True


            if (not _invalid_transition) and (not _unknown_flag):
                #self.exp_pub.publish('[state] ' + str(self.state)) #cmhuang: TODO from here
                rospy.loginfo( "ADD SYNC MARKER: " + str(_data) )
                self.state = freeplayAnnexController.STIMULI
                freeplayAnnexController.transitionToStimuli( self )

                ## NOTE: This has blocking call (monitorMoveToGP)
                self.lookStimuli.turnToScreen( right_screen=_right_screen )

                if _data.endswith( "auto_return=True" ):
                    if self.blocking_gui:
                        try:
                            thread.start_new_thread( freeplayAnnexController.runWatchStimuli, ( self, True, True, ))
                        except Exception as _e_TTS:
                            rospy.logerr("Unable to start thread for freeplayAnnexController.runWatchStimuli()")
                            ## at least set the timers
                            freeplayAnnexController.setAutoTransitionWatchStimuli( self )
                    else:
                        ## add timed trigger 'watch stimuli' behavior
                        ## and followed by turning back to face the infant
                        freeplayAnnexController.setAutoTransitionWatchStimuli( self )
                        ## TODO: Set a timer to enable blink and scan

        elif _data == "reset interaction":
            ## We should only be able to get to this controller state from STIMULI or ENGAGEMENT
            _invalid_transition = freeplayAnnexController.invalidTransition( self, self.state, freeplayAnnexController.INVALID_TRIAL )
            #if ((self.previous_state != freeplayAnnexController.ENGAGEMENT) and
            # (self.previous_state != freeplayAnnexController.STIMULI)):
            # rospy.logwarn("INVALID STATE TRANSITION: Expected to enter state INVALID_TRIAL from ENGAGEMENT or STIMULI...\tcurrent STATE = " + self.state_dict[self.state])
            # _unknown_flag = True
            # self.exp_pub.publish('[WARNING] Invalid state transition at (' + self.state_dict[self.state] + ')')

            if not _invalid_transition:
                ## STEP 1: update state
                self.state = freeplayAnnexController.INVALID_TRIAL

                ## STEP 2: return to neutral position
                if self.previous_state == freeplayAnnexController.ENGAGEMENT:
                    ## if pressed during startleGame, then stop
                    self.startleGame.stopStartleGame( disable_ht=False )

                elif self.previous_state == freeplayAnnexController.STIMULI:
                    ## added check for .__is_stimuli_running in runWatchStimuli()
                    pass

                else:
                    pass ## cannot get here

                ## always begin in neutral position
                ## 2016-07-21 ktsui: to prevent jerk, don't disable HT motor
                rospy.loginfo("INVALID TRIAL BUTTON: call to controllerReset()")
                freeplayAnnexController.controllerReset( self, disable_ht=False )

                self.exp_pub.publish( "Reset interaction; RE-DO interaction #" + str(self.interaction_count ) + "\tPress button 'Run Engagement Game'")


        elif _data == "the end":
            ## We should be able to get to this state from ANY other
            _invalid_transition = freeplayAnnexController.invalidTransition( self, self.state, freeplayAnnexController.END )

            if not _invalid_transition:
                rospy.loginfo( "ADD SYNC MARKER: " + str(_data) )

                ## HACK to shorten ending
                ## if 6 interactions have taken place without any issue and we've
                ## just finished watching stimuli, then the robot is already
                ## in neutral pose... SKIP
                if (self.state == freeplayAnnexController.STIMULI) and (self.interaction_count >= freeplayAnnexController.NUMBER_OF_INTERACTIONS):
                    freeplayAnnexController.cancelWatchStimuliCallbacks( self ) ## just in case
                    pass
                else:
                    ## clean up first, and reset to neutral pose
                    ## 2016-07-21 ktsui: to prevent jerk, don't disable HT motor
                    freeplayAnnexController.stop( self, disable_ht=False )

                freeplayAnnexController.transitionToReady(self, _data)
                self.state = freeplayAnnexController.END ## override state
                self.exp_pub.publish( str(self.interaction_count) + " of " + str(freeplayAnnexController.NUMBER_OF_INTERACTIONS) + " interactions performed with this participant")
                self.exp_pub.publish('[state] ' + self.state_dict[self.state] + "\t---- END OF INSPIRE4 EXPERIMENT ----")


        else:
            _unknown_flag = True


        if _unknown_flag:
            rospy.logwarn( "UNKNOWN pilot command: " + str(_data) + "; REMAINS in self.state " + self.state_dict[self.state] )
            freeplayAnnexController.transitionUsage( self, self.state )

        if ((self.state == None) or _invalid_transition):
            freeplayAnnexController.transitionUsage( self, self.state, "Invalid transition\t" )

        if self.state != self.previous_state:
            if self.previous_state == None:
                rospy.logdebug("Update self.state from [None] to [" + self.state_dict[self.state] + "]")
            else:
                rospy.logdebug("Update self.state from [" + self.state_dict[self.previous_state] + "] to [" + self.state_dict[self.state] + "]")
            freeplayAnnexController.transitionUsage( self, self.state , "NEW state\t\t")
        '''

        rospy.logdebug("parse_annex_command(): END")
        return

    def doGetReady( self, msg=None ):
        ## STEP 0: Reset but don't move to neutral head pose first
        ## 2016-07-21 ktsui: to prevent HT jerk, don't disable HT motor
        freeplayAnnexController.stop( self, neutral_head=False , disable_ht=False)
        freeplayAnnexController.start( self, neutral_head=False )

        ## STEP 1: Move to ready state
        freeplayAnnexController.transitionToReady( self, msg=msg )
        return

    def controllerExit( self ):
        rospy.logdebug("controllerExit(): BEGIN")
        freeplayAnnexController.toggleDataLoggerRecording( self, "started" ) ## we want to stop recording
        if self.ALIVE:
            #self.ALIVE = False
            ## NOTE: head tilt motor will be disabled after reset
            rospy.loginfo("controllerExit(): call to controllerReset()")
            freeplayAnnexController.controllerReset( self )

## 2016-06-16, KATE
        ## nicely exit the behaviors too
        self.asleepAwake.abort()
        self.lookIntro.abort()
        self.startleGame.abort()
        self.lookStimuli.abort()
        self.htBB.stop() ## make sure that the head tilt motor is disengaged

        rospy.sleep(1) # give a chance for everything else to shutdown nicely

        ## 2016-07-20, ktsui: nicely deal with neck; don't jerk up when restarted
        _BB = baseBehavior( False, self.ros_pub )
        _BB.start()
        #_BB.requestFeedback( SC_GET_PP )
        rospy.loginfo( "controllerExit: Cleanup to prevent HT jerking... HT PP=" + str(_BB.makiPP["HT"]) )
        _BB.pubTo_maki_command( "HTGP" + str(_BB.makiPP["HT"]) + TERM_CHAR_SEND) ## after head has fallen down, make nice
        _BB.requestFeedback( SC_GET_GP )

        self.ALIVE = False
        rospy.sleep(1) # give a chance for everything else to shutdown nicely
        rospy.logdebug( "controllerExit: SHUTTING DOWN FREEPLAY ANNEX..." )
        self.exp_pub.publish('----- SHUTTING DOWN FREEPLAY ANNEX -----')
        self.exp_pub.publish("==========================================")
        rospy.logdebug("controllerExit(): END")
        #exit ## meant for interactive interpreter shell; unlikely this actually exits


    ## NOTE: head tilt motor will be disabled after reset movement
    #def controllerReset( self, disable_ht=True ):
    def controllerReset( self, disable_ht=True ):
## 2016-06-16, KATE
        ## TODO: FIX: There seems to be a lot of lag time in this function

        _delta_pp = 2 #ticks
        _reset_duration = 0 #ms
        _reset_buffer = 500 #ms

        ## NOTE: changed to self.htBB since instantiating this behavior
        ## has become expensive given the number of times controllerReset()
        ## is called
        #_htBB = headTiltBaseBehavior( True, self.ros_pub )
        self.htBB.start()

        ## check if we are already in neutral before publishing the goal positions
        if (self.htBB.verifyPose( ht=HT_MIDDLE, hp=HP_FRONT, ll=LL_OPEN_DEFAULT, ep=EP_FRONT, et=ET_MIDDLE, delta_pp=_delta_pp )):
            return

        _pub_reset = ""
        _pub_reset += "HTGP" + str(HT_MIDDLE) + "HPGP" + str(HP_FRONT) + "LLGP" + str(LL_OPEN_DEFAULT) + "EPGP" + str(EP_FRONT) + "ETGP" + str(ET_MIDDLE)
        if ((abs(self.htBB.makiPP["HT"] - HT_MIDDLE) < 10) and
            (abs(self.htBB.makiPP["HP"] - HP_FRONT) < 25) and
            (abs(self.htBB.makiPP["ET"] - ET_MIDDLE) < 10) and
            (abs(self.htBB.makiPP["LL"] - LL_OPEN_DEFAULT) < 25)):
            _reset_duration = 250
            _pub_reset += SC_SET_IPT + str(_reset_duration)
        elif ((abs(self.htBB.makiPP["HT"] - HT_MIDDLE) < 50) and (abs(self.htBB.makiPP["HP"] - HP_FRONT) < 150)):
            _reset_duration = 750
            _pub_reset += SC_SET_IPT + str(_reset_duration)
        elif ((abs(self.htBB.makiPP["HT"] - HT_MIDDLE) < 100) and (abs(self.htBB.makiPP["HP"] - HP_FRONT) < 300)):
            _reset_duration = 1250
            _pub_reset += SC_SET_IPT + str(_reset_duration)
        else:
            _reset_duration = 2000
            _pub_reset += SC_SET_IPT + str(_reset_duration)
        _pub_reset += TERM_CHAR_SEND
        rospy.loginfo("controllerReset(): _pub_reset = " + str(_pub_reset))

        try:
            if (_reset_duration >= 300): ## ms
                ## THIS IS CUSTOM RESET
                ## reset goal speeds and goal positions
                ## and monitor moving into goal positions
                self.htBB.monitorMoveToGP( _pub_reset, ht_gp=HT_MIDDLE, hp_gp=HP_FRONT, ll_gp=LL_OPEN_DEFAULT, ep_gp=EP_FRONT, et_gp=ET_MIDDLE, delta_pp=_delta_pp )
            else: ## monitoring is a waste for anything less than 300 ms
                self.htBB.pubTo_maki_command( _pub_reset )

            if disable_ht:
                rospy.logdebug("controllerReset(): publish reset command... Now STOP using self.htBB.stop()") ## debugging
                self.htBB.stop() ## NOTE: .stop() is closed loop and depends on feedback from motors
                self.htBB.requestFeedback( SC_GET_TL ) ## debugging

        except rospy.exceptions.ROSException as _e:
            if (not self.htBB.verifyPose( ht=HT_MIDDLE, hp=HP_FRONT, ll=LL_OPEN_DEFAULT, ep=EP_FRONT, et=ET_MIDDLE )):
                rospy.logwarn("controllerReset(): WARN: Could not complete monitoring move to neutral position...STALLED??...")
                rospy.logdebug("controllerReset(): ERROR: " + str(_e))
                self.htBB.pubTo_maki_command( _pub_reset )
                rospy.sleep( _reset_duration + _reset_buffer )
            if disable_ht: self.htBB.pubTo_maki_command( "HTTL0Z" )

        except TypeError as _e1:
            if (not self.htBB.verifyPose( delta_pp=_delta_pp, ht=HT_MIDDLE, hp=HP_FRONT, ll=LL_OPEN_DEFAULT, ep=EP_FRONT, et=ET_MIDDLE )):
                rospy.logerror("controllerReset(): TYPE ERROR: " + str(_e1))
                self.htBB.pubTo_maki_command( _pub_reset )
                rospy.sleep( _reset_duration + _reset_buffer )
            if disable_ht: self.htBB.pubTo_maki_command( "HTTL0Z" )
        return

## ------------------------------
def signal_handler(signal, frame):
    global controller
    rospy.loginfo( "signal_handler: CTRL+C" )
    controller.controllerExit()
    rospy.loginfo( "signal_handler: CTRL+C says goodnight" )
    sys.exit() ## use this instead of exit (which is meant for interactive shells)


if __name__ == '__main__':
    print "__main__: BEGIN"

    global controller
    controller = freeplayAnnexController( True, None )
    rospy.logdebug("-------- controller.__init__() DONE -----------")
    controller.exp_pub.publish("=========================================")
    controller.exp_pub.publish("...\tInitializing Freeplay annex controller... Please wait.")

    # allow closing the program using CTRL+C
    #signal.signal(signal.SIGINT, signal_handler)
    # Register shutdown hook
    rospy.on_shutdown(controller.controllerExit)

    rospy.Subscriber( "freeplay_annex_command", String, controller.parse_annex_command )
    rospy.logdebug( "now subscribed to freeplay_annex_command" )

    rospy.Subscriber( "freeplay_annex_data_logger/status", String, controller.updateDataLoggerStatus_callback )
    rospy.logdebug( "now subscribed to freeplay_annex_data_logger/status" )

    controller.start( neutral_head=False )
    rospy.logdebug("-------- controller.start() DONE -----------")
    controller.exp_pub.publish("...\tInitializing Freeplay annex controller... DONE")


    ## hack to make sure that the head tilt motor is shutoff after controllerReset() in start
    #controller.htBB.stop() ## make sure that the head tilt motor is disengaged
    controller.htBB.requestFeedback( SC_GET_TL ) ## debugging

    rospy.sleep(0.5) ## tiny sleep
    controller.doGetReady( msg="__main__" ) ## when controller comes up, just IMMEDIATELY put Maki-ro into 'get ready'

    rospy.logdebug("-------- controller.__main__() DONE ---------- now rospy.spin()")
    rospy.spin() ## keeps python from exiting until this node is stopped

    print "__main__: END"


