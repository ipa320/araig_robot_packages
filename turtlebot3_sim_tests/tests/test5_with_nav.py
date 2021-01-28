#!/usr/bin/env python
import sys
import unittest
import time
import rostest
import rospy
from  araig_msgs.msg import BoolStamped, Float64Stamped
from geometry_msgs.msg import PoseStamped

class Test4WithAction(unittest.TestCase):

    def setUp(self):
        _pub_topic_start = '/signal/ui/start_test'
        _pub_topic_reset = '/signal/ui/reset_test'
        _pub_topic_goal_init = '/move_base_simple/goal'

        _sub_topic_completed = '/signal/runner/test_completed'
        _sub_topic_time = '/data/calc/goal_time'

        rospy.init_node('test1', anonymous=True)
        self.pub_start = rospy.Publisher(_pub_topic_start, BoolStamped, latch=True, queue_size=10)
        self.pub_reset = rospy.Publisher(_pub_topic_reset, BoolStamped, latch=True, queue_size=10)
        self.pub_goal = rospy.Publisher(_pub_topic_goal_init, PoseStamped, latch=True, queue_size=10)

        rospy.Subscriber(_sub_topic_completed, BoolStamped, callback=self.callback_1, queue_size=10)
        rospy.Subscriber(_sub_topic_time, Float64Stamped, callback=self.callback_2, queue_size=10)
        rospy.Subscriber(_pub_topic_start, BoolStamped, callback=self.callback_3, queue_size=10)
        self.result_time = None
        self.result_completed = None
        self.result_start = None
        self.counter = 0

    def test_expected(self):
        while self.counter < 2:
            self.counter +=1
            pub_msg = BoolStamped()
            pub_msg.data = True
            pub_msg.header.stamp = rospy.Time.now()

            # pub /signal/ui/start_test
            self.pub_start.publish(pub_msg)

            # wait for result
            while self.result_start is None:
                rospy.sleep(0.01)
            self.assertTrue(self.result_start, msg = 'Test {}: did not start'.format(self.counter))

            while self.result_time is None or self.result_completed is None:
                rospy.sleep(0.01)
            self.assertAlmostEqual(self.result_time, \
            self.result_time, \
            msg='Test {}: reached goal time: {}, expect {}'.format(self.counter, self.result_time, self.result_time), \
            delta= 0.01)

            self.assertNotAlmostEqual(self.result_time, \
            0, \
            msg = 'Test{}: reached goal time: {}, expect not ZERO'.format(self.counter, self.result_time), \
            delta= 0.0)

            self.assertTrue(self.result_completed, msg = 'Test {}: did not complete'.format(self.counter))

            rospy.sleep(1)

            # move robot away from goal
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"
            goal_msg.pose.position.x = -0.26
            goal_msg.pose.position.y = -1.6
            goal_msg.pose.orientation.w = 1
            self.pub_goal.publish(goal_msg)
            rospy.sleep(7)

            # pub /signal/ui/reset_test
            pub_msg.data = True
            pub_msg.header.stamp = rospy.Time.now()
            self.pub_reset.publish(pub_msg)
            rospy.sleep(1)

            self.assertFalse(self.result_completed, msg = 'Test {}: is not reset'.format(self.counter+1))

            self.result_time = None
            self.result_completed = None
            self.result_start = None
            
    def callback_1(self, msg):
        self.result_completed = msg.data

    def callback_2(self, msg):
        self.result_time = msg.data

    def callback_3(self, msg):
        self.result_start = msg.data

if __name__ == '__main__':
    pkg = 'turtlebot3_sim_tests'
    name = 'test4_with_action'
    rostest.rosrun(pkg, name, Test4WithAction)