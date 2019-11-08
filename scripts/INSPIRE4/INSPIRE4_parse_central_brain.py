#! /usr/bin/env python

import rospy
import re
import csv
from std_msgs.msg import String



#--------------------
class messageCoordinater():
    def __init__(self, behavior_file, vh_topic="ros2CentralBrain"):

        rospy.init_node('messageCoordinater',anonymous=False)

        #self.ros_pub = rospy.Publisher("test_topic",String,queue_size=10)
        self.behaviorDict = self.createBehaviorDict(behavior_file)
        # This is how we talk to ros2vhmsg
        self.vh_pub= rospy.Publisher(vh_topic,String,queue_size=10)


    def parseVHMessage(self, msg):
        rospy.logdebug("parseVHMessage(): BEGIN")
        rospy.logdebug("received: " + str(msg))
        # Turns message into array for easy manipulation
        _data = str(msg.data).strip().split()
        _data = [d.strip() for d in _data]
        rospy.loginfo("Message received from VH: {}".format(_data))

        ## These will help us check that the message is well formed
        # Right now there is only one command, but might add more later
        _command_list= ["performance"]
        _agent_list = ["Maki-ro", "Maki"]

        #[Jake] For October, all valid messages will adhere to this length
        try:
            _command = _data[0]
            _msg_id = _data[1]
            _agent = _data[2]
            _behavior ,_pub_topic,_estimate = self.behaviorDict[_data[3]]
        except IndexError:
            rospy.logerr("parseVHMessage(): ERROR: Message is not well formed!")
            return
        except KeyError:
            rospy.logerr("parseVHMessage(): ERROR:{} is not a valid behavior!".format(_data[3]))
            return

        if _command.lower() in _command_list and _agent in _agent_list and len(_data)==4: 
            rospy.loginfo("{} is well formed!".format(_data))
            ros_pub = rospy.Publisher(_pub_topic,String, queue_size=10)
            rospy.loginfo("Publishing: {} to Maki on topic: {}".format(_behavior,_pub_topic))
            # Outgoing messages look like:
            # <behavior> <msg_id> <estimate>
            ros_pub.publish(_behavior+":"+_msg_id+" "+_estimate)
        else:
            rospy.logerr("parseVHMessage(): ERROR: Message is not well formed!")

    # Incoming messages look like:
    # <msg_id> <status> <estimate> 
    def parseROSMessage(self, msg):
        rospy.logdebug("BEGIN: parseROSMessage()")
        _data = str(msg.data).strip().split()
        rospy.logdebug("RECEIVED: {}".format(str(msg.data)))
        _data = [d.strip() for d in _data]
        rospy.loginfo("Msg received from Maki: {}".format(_data))

        ## These will help us check that the message is well formed
        # Right now we only care about these two commands
        _command_dict= {"START":"begin-performance", "COMPLETED":"performance-completed"}
        _agent_list = ["Maki-ro"]
        try: 
            _msg_id = _data[0]
            _status = _data[1]
        except IndexError:
            rospy.logerr("parseROSMessage: ERROR: Message not well formed!")
            return
        try:
            _estimate = _data[2]
        except IndexError:
            if _status == "COMPLETED":
                _estimate = ""
            else:
                rospy.logerr("parseROSMessage: ERROR: Missing time estimate!")
                return 
        try:
            new_msg  = "{} {} {} {}".format(_command_dict[_status],_msg_id, "Maki-ro",_estimate)
            rospy.loginfo("sending message: {}".format(new_msg))
        except KeyError: 
            rospy.logerr("parseROSMessage: ERROR: Not a valid behavior status!")
            return 
        self.vh_pub.publish(new_msg.strip())

    # returns column col of the behavior csv as an generator generating an array
    def getCol(self,filename, col):
        for row in csv.reader(open(filename), delimiter=','):
            yield row[col]

    # creates a dict with woz.id's as keys and Answers as associated vals.
    def createBehaviorDict(self, behavior_file):
        _answer_col = self.getCol(behavior_file, 0)
        _woz_id_col = self.getCol(behavior_file, 1)
        _ros_topic_col = self.getCol(behavior_file, 2)
        _beavhior_estiamte_col = self.getCol(behavior_file, 3)
        return dict(zip(_woz_id_col, zip(_answer_col, _ros_topic_col,_beavhior_estiamte_col)))

if __name__ == '__main__':
    try:
        vhmsg_topic="from_central_brain"
        ros_topic ="behavior_status"

        infile = '/home/scaz-maki-1/catkin_ws/src/ros2vhmsg/WoZ/freeplay-annex-behaviors.csv'
        m = messageCoordinater(infile)
        rospy.Subscriber(vhmsg_topic,String,m.parseVHMessage)
        rospy.Subscriber(ros_topic,String,m.parseROSMessage)
        rospy.loginfo("Parsing central brain...")
        while not rospy.is_shutdown():
            rospy.Rate(10).sleep()
    except rospy.ROSInterruptException:
        pass

        

