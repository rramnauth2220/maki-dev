#! /usr/bin/env python

## To be used in conjunction with INSPIRE4_controller.py

import rospy
from std_msgs.msg import String

class InputSimulator(object):
	gui_items = {'0': 'init pilot GUI', '1': 'get ready', '2': 'sync Tobii calibration start', '3': 'runFamiliarizationSkit', '4': 'startleGame start', '5': 'turnToScreen left auto_return=True', '6': 'turnToScreen right auto_return=True', '7': 'the end', '8': 'reset experiment'} #TODO: complete this list

	def __init__(self):
		rospy.init_node('input_simulator', anonymous=True)
		self.input_pub = rospy.Publisher("/inspire_four_pilot_command", String, queue_size=1)
		self.manual_test()
		#self.auto_test()

	def print_menu(self):
		print '-------------------------'
		print '0: init pilot GUI'
		print '1: get ready'
		print '2: sync Tobii calibration start'
		print '3: runFamiliarizationSkit'
		print '4: startleGame start'
		print '5: turnToScreen left auto_return=True'
		print '6: turnToScreen right auto_return=True'
		print '7: the end' 
		print '8: reset experiment'
		print '-------------------------'

	def manual_test(self):
		while True:
			self.print_menu()
			choice = raw_input('Enter your input:')
			print 'You chose ... ' + InputSimulator.gui_items[choice]
			self.input_pub.publish(InputSimulator.gui_items[choice])

	def auto_test(self):
		#TODO: generate several input sequences to test the system
		print 'auto_test'  


if __name__ == '__main__':
	print 'running the input simulator..'
	
	sim = InputSimulator()
	rospy.spin()   ## keeps python from exiting until this node is stopped
