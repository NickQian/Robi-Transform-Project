#!/usr/bin/env python

#
# 2017.12.22: init version.
#

from __future__ import print_function

import rospy, rostest
import sys, time
import unittest, rostest

import actionlib

import actprocess.msg


class TestWfw(unittest.TestCase):

    def __init__(self, *args):
        super(TestWfw, self).__init__(*args)
        self.success = False

    def callback(self, data):
        self.success = data.data and data.data.startswith('')

    def test_wfw(self):

        print ("@~@: In <test_wfw>")

        try:
            client = actionlib.SimpleActionClient('client_wfw_test', actprocess.msg.wfwAction)
            client.wait_for_server()

            print ("@~@: In <test_wfw>: wait_for_server sucess!")

            goal = actprocess.msg.wfwGoal(keep_balance = True)

            client.send_goal(goal)
            print ("@~@: In <test_wfw>: goal sent!")

            client.wait_for_result()
            result = client.get_result()

            print ("@~@: Result:", ', '.join( [str(n) for n in result.sequence] ) )

        except rospy.ROSInterruptException:
            print ("Program interrupted before completion", file=sys.stderr)

        timeout_t = time.time() + 10.0

        while (not rospy.is_shutdown() and
               not self.success and time.time() < timeout_t):
            time.sleep(0.1)
            print ('.')

        self.assert_(self.success)



def main():
    rospy.init_node('test_wfw', log_level=rospy.DEBUG)
    #rospy.Subscriber("",)
    print ("@~@: test_wfw: Node inited. ")

    rostest.rosrun("actprocess", "wfw_test", TestWfw, sys.argv)      #package_name, test_name, test_case_class, sys.argv



if __name__ == '__main__':
    main()

