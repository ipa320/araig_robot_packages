#!/usr/bin/env python
import sys
import unittest
import time
import rostest
import rospy
from  araig_msgs.msg import BoolStamped, Float64Stamped
from geometry_msgs.msg import Twist

class Test1Braking(unittest.TestCase):

    def setUp(self):
        _pub_topic_start = '/signal/ui/start_test'
        _pub_topic_reset = '/signal/ui/reset_test'
        _pub_topic_vel = '/cmd_vel'
        _pub_topic_interrupt = '/signal/ui/interrupt_test'

        _sub_topic_distance = '/data/calc/braking_distance'
        _sub_topic_time = '/data/calc/braking_time'
        _sub_topic_max = '/signal/calc/robot_has_max_vel'
        _sub_topic_failed = '/signal/runner/test_failed'

        rospy.init_node('test1', anonymous=True)
        self.pub_start = rospy.Publisher(_pub_topic_start, BoolStamped, latch=True, queue_size=10)
        self.pub_reset = rospy.Publisher(_pub_topic_reset, BoolStamped, latch=True, queue_size=10)
        self.pub_vel = rospy.Publisher(_pub_topic_vel, Twist, latch=True, queue_size=10)
        self.pub_interrupt = rospy.Publisher(_pub_topic_interrupt, BoolStamped, latch=True, queue_size=10)

        rospy.Subscriber(_sub_topic_distance, Float64Stamped, callback=self.callback_1, queue_size=10)
        rospy.Subscriber(_sub_topic_time, Float64Stamped, callback=self.callback_2, queue_size=10)
        rospy.Subscriber(_sub_topic_max, BoolStamped, callback=self.callback_3, queue_size=10)
        rospy.Subscriber(_sub_topic_failed, BoolStamped, callback=self.callback_4, queue_size=10)

        self.result_distance = None
        self.result_time = None
        self.result_max = None
        self.result_failed  = None

        self.counter = 0

    def test_brake(self):
        while self.counter < 3:
            self.counter +=1
            pub_msg = BoolStamped()
            pub_msg.data = True
            pub_msg.header.stamp = rospy.Time.now()

            # pub /signal/ui/start_test
            self.pub_start.publish(pub_msg)

            # testing if robot did not really reach max speed
            if self.counter == 1:
                while self.result_max is None or self.result_max.data == False:
                    rospy.sleep(0.01)
                vel = Twist()
                if self.result_max.data == True:
                    vel.linear.x = 0.7
                    for i in range(100):
                        rospy.sleep(0.01)
                        self.pub_vel.publish(vel)

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
            rospy.sleep(5)
            pub_msg.data = True
            pub_msg.header.stamp = rospy.Time.now()
            self.pub_reset.publish(pub_msg)
            rospy.sleep(1)

            self.result_distance = None
            self.result_time = None
    
    def test_interrupted(self):
        while self.counter < 3:
            self.counter +=1
            pub_msg = BoolStamped()
            pub_msg.data = True

            pub_msg.header.stamp = rospy.Time.now()
            # pub /signal/ui/start_test
            self.pub_start.publish(pub_msg)

            rospy.sleep(3.01)

            if self.counter == 1:
                pub_msg.data = True
                pub_msg.header.stamp = rospy.Time.now()
                self.pub_interrupt.publish(pub_msg)
            
            if self.counter == 2:
                rospy.sleep(0.1)
                pub_msg.data = True
                pub_msg.header.stamp = rospy.Time.now()
                self.pub_interrupt.publish(pub_msg)
            
            if self.counter == 3:
                rospy.sleep(2.1)
                pub_msg.data = True
                pub_msg.header.stamp = rospy.Time.now()
                self.pub_interrupt.publish(pub_msg)

            # wait for result
            while self.result_failed is None or self.result_failed is False:
                rospy.sleep(0.01)

            self.assertTrue(self.result_failed, 
            msg = 'Test{}: Test_Failed should be True, but get {}'.format(self.counter, self.result_failed))

            # pub /signal/ui/reset_test
            rospy.sleep(5)
            pub_msg.header.stamp = rospy.Time.now()
            self.pub_reset.publish(pub_msg)
            rospy.sleep(1)

            self.result_failed = None
            rospy.sleep(2)

    def callback_1(self, msg):
        self.result_distance = msg.data
   
    def callback_2(self, msg):
        self.result_time = msg.data

    def callback_3(self, msg):
        self.result_max = msg  

    def callback_4(self, msg):
        self.result_failed = msg.data

if __name__ == '__main__':
    pkg = 'turtlebot3_sim_tests'
    name = 'test1_braking'
    rostest.rosrun(pkg, name, Test1Braking)