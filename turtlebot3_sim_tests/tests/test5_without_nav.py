#!/usr/bin/env python
import sys
import unittest
import time
import rostest
import rospy
from  araig_msgs.msg import BoolStamped, Float64Stamped
from geometry_msgs.msg import PoseStamped

class Test4NoGoal(unittest.TestCase):

    def setUp(self):
        _pub_topic_start = '/signal/ui/start_test'
        _pub_topic_reset = '/signal/ui/reset_test'
        _pub_topic_stop = '/signal/runner/stop_robot'

        _sub_topic_completed = '/signal/runner/test_completed'
        _sub_topic_gap = '/data/calc/robot_stop_gap'

        rospy.init_node('test4', anonymous=True)
        self.pub_start = rospy.Publisher(_pub_topic_start, BoolStamped, latch=True, queue_size=10)
        self.pub_reset = rospy.Publisher(_pub_topic_reset, BoolStamped, latch=True, queue_size=10)
        self.pub_stop = rospy.Publisher(_pub_topic_stop, BoolStamped, latch=True, queue_size=10)

        rospy.Subscriber(_sub_topic_completed, BoolStamped, callback=self.callback_1, queue_size=10)
        rospy.Subscriber(_sub_topic_gap, Float64Stamped, callback=self.callback_2, queue_size=10)
        self.result_gap = None
        self.result_completed = None
        self.counter = 0

    def test_expected(self):
        while self.counter < 5:
            self.counter +=1
            pub_msg = BoolStamped()
            pub_msg.data = True
            pub_msg.header.stamp = rospy.Time.now()

            # pub /signal/ui/start_test
            self.pub_start.publish(pub_msg)

            rospy.sleep(5)
            # pub /signal/runner/stop_robot
            pub_msg.header.stamp = rospy.Time.now()
            self.pub_stop.publish(pub_msg)

            # wait for result
            while self.result_completed is None or self.result_gap is None:
                rospy.sleep(0.01)

            self.assertAlmostEqual(self.result_gap, \
            self.result_gap, \
            msg='Test {}: robot stop at {}m from the obstacle, expect {}m'.format(self.counter, self.result_gap, self.result_gap), \
            delta= 0.01)

            self.assertNotAlmostEqual(self.result_gap, \
            0, \
            msg = 'Test{}: robot should not stop at {}m from the obstacle'.format(self.counter, self.result_gap), \
            delta= 0.0)

            self.assertTrue(self.result_completed, msg = 'Test {}: did not complete'.format(self.counter))

            rospy.sleep(5)
            # pub /signal/ui/reset_test
            pub_msg.data = True
            pub_msg.header.stamp = rospy.Time.now()
            self.pub_reset.publish(pub_msg)
            rospy.sleep(1)

            self.result_time = None
            self.result_completed = None
            self.result_start = None
            
    def callback_1(self, msg):
        self.result_completed = msg.data

    def callback_2(self, msg):
        self.result_gap = msg.data

if __name__ == '__main__':
    pkg = 'turtlebot3_sim_tests'
    name = 'test4_no_goal'
    rostest.rosrun(pkg, name, Test4NoGoal)