#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""QSRlib ROS server interface.

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 22 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""

from __future__ import print_function, division
import rospy
from qsrlib.qsrlib import QSRlib
from qsr_lib.srv import *
try:
    import cPickle as pickle
except:
    import pickle


class QSRlib_ROS_Server(object):
    def __init__(self, node_name="qsr_lib", active_qsrs=None):
        self.qsrlib = QSRlib(active_qsrs)
        self.node_name = node_name
        self.node = rospy.init_node(self.node_name)
        self.service_topic_names = {"request": self.node_name+"/request"}
        self.srv_qsrs_request = rospy.Service(self.service_topic_names["request"], RequestQSRs, self.handle_request_qsrs)
        print("QSRlib_ROS_Server up and running, listening to:", self.service_topic_names["request"])

    def handle_request_qsrs(self, req):
        print("Handling QSRs request made at", str(req.header.stamp.secs)+"."+str(req.header.stamp.nsecs))
        request_message = pickle.loads(req.data)
        qsrs_response_message = self.qsrlib.request_qsrs(request_message=request_message)
        res = RequestQSRsResponse()
        res.header.stamp = rospy.get_rostime()
        res.data = pickle.dumps(qsrs_response_message)
        print()
        return res


if __name__ == "__main__":
    srv = QSRlib_ROS_Server()
    rospy.spin()
