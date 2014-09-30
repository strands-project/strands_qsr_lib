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
import datetime
import os
import sys
import inspect
# http://stackoverflow.com/questions/279237/import-a-module-from-a-relative-path
add_folder = os.path.realpath(os.path.abspath(os.path.split(inspect.getfile( inspect.currentframe() ))[0]))
if add_folder not in sys.path:
    sys.path.insert(0, add_folder)
add_folder += "/../standalone"
if add_folder not in sys.path:
    sys.path.insert(0, add_folder)
from qsrlib import QSRlib
import rospy
from qsr_lib.msg import *
from qsr_lib.srv import *
from input_data import Input_Data_Block, Input_Data_One

class QSRlib_ROS_Server(object):
    def __init__(self, node_name="qsr_lib", active_qsrs=None):
        self.qsrlib = QSRlib(active_qsrs)
        self.node_name = node_name
        self.node = rospy.init_node(self.node_name)
        self.service_topic_names = {"request": self.node_name+"/request"}
        self.srv_qsrs_request = rospy.Service(self.service_topic_names["request"], QSRsRequest, self.handle_qsrs_request)
        print("QSRlib_ROS_Server up and running, listening to:", self.service_topic_names["request"])

    def handle_qsrs_request(self, req):
        print("Handling QSRs request made at", str(req.header.stamp.secs)+"."+str(req.header.stamp.nsecs))
        input_data = self.many_objects_data_to_input_data_block(req.input_data)
        qsrlib_res = self.qsrlib.request_qsrs(which_qsr=req.which_qsr, input_data=input_data, reset=req.reset)
        res = QSRsRequestResponse(self.output_data_to_ros_qsrs(qsrlib_res))
        # res = QSRsRequestResponse()
        return res

    def output_data_to_ros_qsrs(self, data):
        ret = QSRs()
        ret.qsr_type = data.qsr_type
        ret.all_possible_relations = data.all_possible_relations
        ret.timestamp_request_received = self.convert_pythondatetime_to_rostime(data.timestamp_request_received)
        ret.timestamp_qsrs_processed = self.convert_pythondatetime_to_rostime(data.timestamp_qsrs_processed)
        for ids, qsrs in zip(data.ids, data.data):
            ret.data.append(ObjectsQSRs(ids=ids, qsrs=qsrs))
        return ret

    def many_objects_data_to_input_data_block(self, data):
        datafoo = []
        for i in data.data:
            datafoo.append(Input_Data_One(i.id, i.data))
        ret = Input_Data_Block(data=datafoo, fields=data.fields, timesteps=data.timesteps, description=data.description)
        return ret

    def convert_pythondatetime_to_rostime(self, pythondatetime):
        tsecs = (pythondatetime - datetime.datetime.utcfromtimestamp(0)).total_seconds()
        return rospy.Time.from_sec(tsecs)

if __name__ == "__main__":
    srv = QSRlib_ROS_Server()
    srv.qsrlib.help()
    rospy.spin()
