#!/usr/bin/env python
import sys
import unittest
import time
import rostest
import rospy
from  araig_msgs.msg import BoolStamped, Float64Stamped
from cob_msgs.msg import EmergencyStopState
class Test1Emergency(unittest.TestCase):

    def setUp(self):
        _pub_topic_start = '/signal/ui/start_test'
        _pub_topic_reset = '/signal/ui/reset_test'
        _pub_topic_emergency = '/emergency_stop_state'
        _sub_topic_distance = '/data/calc/braking_distance'
        _sub_topic_time = '/data/calc/braking_time'

        rospy.init_node('test1', anonymous=True)
        self.pub_start = rospy.Publisher(_pub_topic_start, BoolStamped, latch=True, queue_size=10)
        self.pub_reset = rospy.Publisher(_pub_topic_reset, BoolStamped, latch=True, queue_size=10)
        self.pub_emergency = rospy.Publisher(_pub_topic_emergency, EmergencyStopState, latch=True, queue_size=10)

        rospy.Subscriber(_sub_topic_distance, Float64Stamped, callback=self.callback_1, queue_size=10)
        rospy.Subscriber(_sub_topic_time, Float64Stamped, callback=self.callback_2, queue_size=10)
        self.result_distance = None
        self.result_time = None
        self.counter = 0

    def test_expected(self):
        while self.counter < 3:
            self.counter +=1
            pub_msg_signal = BoolStamped()
            pub_msg_signal.data = True
            pub_msg_signal.header.stamp = rospy.Time.now()
            
            # pub /signal/ui/start_test
            self.pub_start.publish(pub_msg_signal)

            # pub /emergency_stop_state
            rospy.sleep(8)
            pub_msg_emergency = EmergencyStopState()
            pub_msg_emergency.emergency_state = 1
            self.pub_emergency.publish(pub_msg_emergency)

            # wait for result
            while self.result_time is None:
                rospy.sleep(0.01)

            self.assertAlmostEqual(self.result_time, \
            self.result_time, \
            msg = 'Test{}: braking time: {}, expect {}'.format(self.counter, self.result_time, self.result_time), \
            delta= 0.1)

            self.assertNotAlmostEqual(self.result_time, \
            0, \
            msg = 'Test{}: braking time: {}, expect not ZERO'.format(self.counter, self.result_time), \
            delta= 0.0)

            while self.result_distance is None:
                rospy.sleep(0.01)

            self.assertAlmostEqual(self.result_distance, \
            self.result_distance, \
            msg = 'Test{}: braking distance: {}, expect {}'.format(self.counter, self.result_distance, self.result_distance), \
            delta= 0.1)

            self.assertNotAlmostEqual(self.result_distance, \
            0, \
            msg = 'Test{}: braking distance: {}, expect not ZERO'.format(self.counter, self.result_distance), \
            delta= 0.0)

            # pub /signal/ui/reset_test
            rospy.sleep(4)
            pub_msg_emergency.emergency_state = 0
            self.pub_emergency.publish(pub_msg_emergency)
            rospy.sleep(1)

            pub_msg_signal.data = True
            pub_msg_signal.header.stamp = rospy.Time.now()
            self.pub_reset.publish(pub_msg_signal)
            rospy.sleep(1)

            self.result_distance = None
            self.result_time = None
            
    def callback_2(self, msg):
        self.result_time = msg.data
        
    def callback_1(self, msg):
        self.result_distance = msg.data

if __name__ == '__main__':
    pkg = 'turtlebot3_sim_tests'
    name = 'test1_emergency'
    rostest.rosrun(pkg, name, Test1Emergency)