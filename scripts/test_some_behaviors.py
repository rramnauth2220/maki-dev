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
    
    while True:
        n = raw_input("\n Input message:")
        if (n == "shake"):
            msgs = [
                "reset eyelids" ,
                #"spontaneousBlink start",
                #"nod",
                "shake"
            ]
        # FILL IN REMAINING COMMANDS
        for msg in msgs:
            rospy.loginfo(msg)
            pub.publish(String(msg))
            rospy.sleep(2.0)
    '''msgs = [
        "reset eyelids" ,
        #"spontaneousBlink start",
        #"nod",
        "shake"
    ]'''

    