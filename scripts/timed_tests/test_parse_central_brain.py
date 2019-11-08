#!/usr/bin/env python
# license removed for brevity
import rospy
import random 
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('from_central_brain', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        str1 = "performance m001 Maki-ro t2"
        str2 = "performance m054 Maki"
        str3 = "hi"
        str4 = " performance m689 Maki-ro annex_unhide"
        str5 = "performance m888 Maki-ro inspire20"
        str6 = "performance m888 Maki-ro annex_nod"
        str_list = [str1, str2,str3, str4,str5,str6]
        test_str = random.choice(str_list)
        rospy.loginfo(test_str)
        pub.publish(test_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
