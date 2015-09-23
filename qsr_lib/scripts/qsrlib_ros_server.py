#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import rospy
from qsrlib.qsrlib import QSRlib
from qsr_lib.srv import *
try:
    import cPickle as pickle
except:
    import pickle


class QSRlib_ROS_Server(object):
    """QSRlib ROS server."""

    def __init__(self, node_name="qsr_lib"):
        """Constructor.

        :param node_name: The QSRlib ROS server node name.
        :type node_name: str
        """
        self.qsrlib = QSRlib()
        """:class:`QSRlib <qsrlib.qsrlib.QSRlib>`: QSRlib main object."""

        self.node_name = node_name
        """str: QSRlib ROS server node name."""

        rospy.init_node(self.node_name)

        self.service_topic_names = {"request": self.node_name+"/request"}
        """dict: Holds the service topic names."""

        self.srv_qsrs_request = rospy.Service(self.service_topic_names["request"], RequestQSRs, self.handle_request_qsrs)
        """rospy.impl.tcpros_service.Service: QSRlib ROS service."""

        rospy.loginfo("QSRlib_ROS_Server up and running, listening to: %s" % self.service_topic_names["request"])

    def handle_request_qsrs(self, req):
        """Service handler.

        :param req: QSRlib ROS request.
        :type req: qsr_lib.srv.RequestQSRsRequest
        :return: SRlib ROS response message.
        :rtype: qsr_lib.srv.RequestQSRsResponse
        """
        rospy.logdebug("Handling QSRs request made at %i.%i" % (req.header.stamp.secs, req.header.stamp.nsecs))
        req_msg = pickle.loads(req.data)
        qsrlib_res_msg = self.qsrlib.request_qsrs(req_msg)
        ros_res_msg = RequestQSRsResponse()
        ros_res_msg.header.stamp = rospy.get_rostime()
        ros_res_msg.data = pickle.dumps(qsrlib_res_msg)
        return ros_res_msg


if __name__ == "__main__":
    srv = QSRlib_ROS_Server()
    rospy.spin()
