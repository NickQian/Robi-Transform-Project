#!/usr/bin/env python
# 2016.7.15

import rospy
import sys
import actionlib
from actionlib_msgs.msg import GoalStatus
from bt_lib import Task

class MonitorTask(Task):
    """ a Ros subscriber """
    def __init__(self, name, topic, msg_type, msg_cb, wait_for_message=True, timeout=5):
        super(MonitorTask, self).__init__(name)

        self.topic = topic
        self.msg_type = msg_type
        self.timeout = timeout
        self.msg_cb = msg_cb

        rospy.loginfo("***** subscribing to topic " + topic )

        if wait_for_message:
            try:
                rospy.wait_for_message(topic, msg_type, timeout = self.timeout)
                rospy.loginfo("Connected.")
            except:
                rospy.loginfo("Timed out waiting for " + topic)
        rospy.Subscriber(self.topic, self.msg_type, self._msg_cb)

    def _msg_cb(self, msg):
        self.set_status(self.msg_cb(msg) )

    def run(self):
        return self.status

    def reset(self):
        pass

class ServiceTask(Task):
    """ turn ROS serviceProxy into task"""
    def __init__(self, name, service, service_type, request, result_cb = None, wait_for_service=True, timeout=5):
        super(ServiceTask, self).__init__(name)

        self.result = None
        self.request = request
        self.timeout = timeout
        self.result_cb = result_cb

        rospy.loginfo("Connecting tp service: " + service )

        if wait_for_service:
            rospy.loginfo("Waiting for service")
            rospy.wait_for_service(service, timeout = self.timeout)
            rospy.loginfo("Connected. ")

        #creat a service proxy
            self.service_proxy = rospy.ServiceProxy(service, service_type)

    def run(self):
        try:
            result = self.service_proxy(self.request)
            if self.result_cb is not None:
                self.result_cb(result)
            return TaskStatus.SUCCESS
        except:
            rospy.logerr(sys.exc_info() )
            return TaskStatus.FAILURE

    def reset(self):
        self.status = None


class SimpleActionTask(Task):
    """ client of ROS action """
    def __init__(self, name, action, action_type, goal, rate=5, connect_timeout=10, result_timeout=30, reset_after=False, active_cb=None, done_cb=None, feedback_cb=None):
        super(SimpleActionTask, self).__init__(name)
        
            




























    
        
