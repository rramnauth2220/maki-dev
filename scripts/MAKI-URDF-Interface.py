#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math

defVals = [512, 512, 512, 512, 500]
curVals = [512, 512, 512, 512, 500]
headers = ["HT", "HP", "EP", "ET", "LL"]

def listener():
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher("maki_command", String, queue_size = 26)
    pub.publish("reset")
    rospy.Subscriber("joint_states", JointState, sendCommand)
    rospy.spin()

def sendCommand(jointState):
    global curVals
    global defVals
    global headers

    command = ""
    comFlag = 0
    for i in range(0, 5):
        servoVal = int(defVals[i] - jointState.position[i]/0.00511326171)
        if (curVals[i] != servoVal):
            comFlag = 1
            command += headers[i] + "GP" + str(servoVal)
            curVals[i] = servoVal
    if (comFlag):
        pub = rospy.Publisher("maki_command", String, queue_size = 26)
        pub.publish(command + "Z")
        comFlag = 0
        

if __name__ == '__main__':
    listener()
