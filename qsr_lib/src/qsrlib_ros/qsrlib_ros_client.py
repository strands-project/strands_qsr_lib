#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
try:
    import cPickle as pickle
except:
    import pickle
import rospy
from qsr_lib.srv import *


class QSRlib_ROS_Client(object):
    """

    """
    def __init__(self, service_node_name="qsr_lib"):
        """Constructor.

        :param service_node_name: The node name of the service.
        :type service_node_name: str
        :return:
        """
        self.service_topic_names = {"request": service_node_name+"/request"}
        """dict: The topic names of the services."""

        rospy.logdebug("Waiting for service '" + self.service_topic_names["request"] + "' to come up")
        rospy.wait_for_service(self.service_topic_names["request"])
        rospy.logdebug("done")

    def request_qsrs(self, req):
        """Request to compute QSRs.

        :param req: The request message.
        :type req: qsr_lib.srv.RequestQSRsRequest
        :return: The ROS service response.
        :rtype: qsr_lib.srv.RequestQSRsResponse
        """
        rospy.logdebug("Requesting QSRs...")
        try:
            proxy = rospy.ServiceProxy(self.service_topic_names["request"], RequestQSRs)
            res = proxy(req)
            return res
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s"%e)

    def make_ros_request_message(self, qsrlib_request_message):
        """Make a QSRlib ROS service request message from standard QSRlib request message.

        :param qsrlib_request_message: The standard QSRlib request message.
        :type qsrlib_request_message: qsrlib.qsrlib.QSRlib_Request_Message
        :return: The ROS service request message.
        :rtype: qsr_lib.srv.RequestQSRsRequest
        """
        req = RequestQSRsRequest()
        req.header.stamp = rospy.get_rostime()
        req.data = pickle.dumps(qsrlib_request_message)
        return req
