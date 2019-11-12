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

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node("test_behaviors")
    rospy.loginfo("Sending test commands")
    pub = rospy.Publisher("/maki_macro", String, queue_size=10)
    '''
    default = [ # default behaviors
        "reset selectiveAttention",
        "visualScan start"
    ]
    '''
    default = [ # default behaviors
        "reset eyelids",
        "reset selectiveAttention",
        "spontaneousBlink start" #"asleep"
    ]

    for msg in default: 
        rospy.loginfo(msg)
        pub.publish(String(msg))
        rospy.sleep(1.0)

    while True:
        n = raw_input("\n Input message:")
        if (n == "think"): # 'deep thought' behavior
            msgs = [
                "lookAtAlissa",
                #"reset selectiveAttention",
                "visualScan start" 
            ]
        elif (n == "wonder"): # 'wondering' behavior
            msgs = [
                "lookAwayFromAlissa",
                "visualScan start"
            ]
        elif (n == "neutral"): # return to neutral position
            msgs = [
                "lookNeutral",
                "visualScan stop"
            ]
        elif (n == "shake"): # 'no' behavior
            msgs = [
                "reset eyelids",
                "visualScan stop",
                "shake"
            ]
        elif (n == "nod"): # 'yes' behavior
            msgs = [
                "reset eyelids",
                "visualScan stop",
                "nod"
            ]

        for msg in msgs:
            rospy.loginfo(msg)
            pub.publish(String(msg))
            rospy.sleep(1.0)
    