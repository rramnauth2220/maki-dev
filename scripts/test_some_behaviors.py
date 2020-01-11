#!/usr/bin/env python
'''
import rospy
from std_msgs.msg import String


if __name__ == '__main__':
    rospy.init_node("test_behaviors")
    rospy.loginfo("Sending test commands")
    pub = rospy.Publisher("/maki_macro", String, queue_size=10)
    msgs = [
        "reset eyelids" ,
        #"spontaneousBlink start",
        #"nod",
        "shake"
    ]

    for msg in msgs:
        rospy.loginfo(msg)
        pub.publish(String(msg))
        rospy.sleep(2.0)
'''

import rospy, logging, time
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node("test_behaviors")
    rospy.loginfo("launch file executed ") # location: ~/.ros/log/latest/test_cmds-8.log
    pub = rospy.Publisher("/maki_macro", String, queue_size=10)
    
    default = [ # default behaviors
        "reset eyelids",
        "reset selectiveAttention",
        "spontaneousBlink start" #"asleep"
    ]
    current_action = "initialize"

    rospy.loginfo("[CMD-SENT] " + current_action)
    start_time = time.time()
    for msg in default: 
        #rospy.loginfo(msg)
        pub.publish(String(msg))
        rospy.sleep(1.0)
    elapsed_time = time.time() - start_time
    rospy.loginfo("[CMD-COMPLETE, time elapsed = %ss] %s" % (elapsed_time, current_action))

    while True:
        n = raw_input("\n Input message:")
        if (n == "think"): # 'deep thought' behavior
            msgs = [
                "lookAtAlissa",
                #"reset selectiveAttention",
                "visualScan start" 
            ]
            current_action = "think"
        elif (n == "wonder"): # 'wondering' behavior
            msgs = [
                "lookAwayFromAlissa",
                "visualScan start"
            ]
            current_action = "wonder"
        elif (n == "neutral"): # return to neutral position
            msgs = [
                "lookNeutral",
                "visualScan stop"
            ]
            current_action = "neutral"

        elif (n == "shake full"): # 'no' behavior
            msgs = [
                "reset eyelids",
                "visualScan stop",
                "shake full"
            ]
            current_action = "shake full"
        elif (n == "shake right"): # 'no' behavior
            msgs = [
                "reset eyelids",
                "visualScan stop",
                "shake right"
            ]
            current_action = "shake right"
        elif (n == "shake left"): # 'no' behavior
            msgs = [
                "reset eyelids",
                "visualScan stop",
                "shake left"
            ]
            current_action = "shake left"
        elif (n == "nod up"): # 'yes' behavior
            msgs = [
                "reset eyelids",
                "visualScan stop",
                "nod up"
            ]
            current_action = "nod up"
        elif (n == "nod down"): # 'yes' behavior
            msgs = [
                "reset eyelids",
                "visualScan stop",
                "nod down"
            ]
            current_action = "nod down"
        elif (n == "nod full"): # 'yes' behavior
            msgs = [
                "reset eyelids",
                "visualScan stop",
                "nod full"
            ]
            current_action = "nod full"
        elif (n == "nothing"): # 'quiet' behavior
            msgs = [
                "reset eyelids",
                "visualScan stop",
                "spontaneousBlink stop"
            ]
            current_action = "nothing" 
        elif (n == "default"): # 'alive' behavior
            msgs = [ # default behaviors
                "reset eyelids",
                "reset selectiveAttention",
                "spontaneousBlink start" #"asleep"
            ]
            current_action = "default" 

        rospy.loginfo("[CMD-SENT] " + current_action)
        start_time = time.time()
        for msg in msgs:
            pub.publish(String(msg))
            rospy.sleep(1.0)
        elapsed_time = time.time() - start_time
        rospy.loginfo("[CMD-COMPLETE, time elapsed = %ss] %s" % (elapsed_time, current_action))

        #for msg in msgs:
        #    rospy.loginfo(msg)
        #    pub.publish(String(msg))
        #    rospy.sleep(1.0)
    