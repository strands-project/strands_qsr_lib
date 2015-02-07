#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""QSRlib ROS client sample.

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 22 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""

from __future__ import print_function, division
try:
    import cPickle as pickle
except:
    import pickle
import rospy
from qsr_lib.srv import *


class QSRlib_ROS_Client(object):
    def __init__(self, service_node_name="qsr_lib"):
        self.service_topic_names = {"request": service_node_name+"/request"}
        print("Waiting for service '" + self.service_topic_names["request"] + "' to come up", end="")
        rospy.wait_for_service(self.service_topic_names["request"])
        print("\tdone")

    def request_qsrs(self, req):
        print("Requesting QSRs...")
        try:
            proxy = rospy.ServiceProxy(self.service_topic_names["request"], RequestQSRs)
            res = proxy(req)
            return res
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

    def make_ros_request_message(self, qsrlib_request_message):
            # following line gives the following error if there is no rospy.init_node(),
            # which is in the class constructor now
            # rospy.exceptions.ROSInitException: time is not initialized. Have you called init_node()?
            req = RequestQSRsRequest()
            req.header.stamp = rospy.get_rostime()
            req.data = pickle.dumps(qsrlib_request_message)
            return req
