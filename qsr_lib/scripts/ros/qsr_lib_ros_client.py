#!/usr/bin/env python

from __future__ import print_function, division
from docutils.nodes import description
import rospy
from qsr_lib.msg import *
from qsr_lib.srv import *
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
from input_data import Input_Data_Block, Input_Data_One

class QSR_Lib_ROS_Client(object):
    def __init__(self, service_node_name="qsr_lib"):
        self.client_node = rospy.init_node("qsr_lib_ros_client_example")  # needed for rospy.get_rostime() in the request method
        self.service_topic_names = {"request": service_node_name+"/request"}
        print("Waiting for service '" + self.service_topic_names["request"] + "' to come up", end="")
        rospy.wait_for_service(self.service_topic_names["request"])
        print("\tdone")

    def cln_qsrs_request(self, req):
        print("Requesting QSRs...")
        try:
            proxy = rospy.ServiceProxy(self.service_topic_names["request"], QSRsRequest)
            # following line gives the following error if there is no rospy.init_node(),
            # which is in the class constructor now
            # rospy.exceptions.ROSInitException: time is not initialized. Have you called init_node()?
            req.header.stamp = rospy.get_rostime()
            res = proxy(req)
            return res
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

    def input_data_block_to_many_objects_data(self, data):
        if isinstance(data, Input_Data_Block):
            ret = ManyObjectsData()
            ret.description = data.description
            ret.fields = data.fields
            ret.timesteps = data.timesteps
            for i in data.data:
                ret.data.append(ObjectData(i.id, i.data))
        else:
            print("Warning (qsr_lib_ros_client.py/dataBlock2ManyObjectsData): will use previous data")
            ret = ManyObjectsData(description="previous")
        return ret

    def make_request_message(self, which_qsr,
                             reset=False,
                             input_data=None,
                             header=None):
        print("Making request message...")
        req = QSRsRequestRequest()
        if header:
            req.header = header
        req.input_data = self.input_data_block_to_many_objects_data(input_data)
        req.which_qsr = which_qsr
        req.reset = reset
        return req


if __name__ == "__main__":
    cln = QSR_Lib_ROS_Client()
    input_data = Input_Data_Block(data=[Input_Data_One("1", [1.0, 1.0, 2.0, 2.0, 1.0, 1.0, 2.0, 2.0, 1.0, 1.0, 2.0, 2.0]),
                                        Input_Data_One("2", [11.0, 11.0, 22.0, 22.0, 1.5, 1.5, 22.0, 22.0, 1.5, 1.5, 1.5, 1.5])],
                                  fields=["x1", "y1", "x2", "y2"],
                                  timesteps=3,
                                  description="some 2d bounding boxes")
    req = cln.make_request_message(which_qsr="rcc3_rectangle_bounding_boxes_2d", input_data=input_data)
    res = cln.cln_qsrs_request(req)
    print("--------------")
    print("Response is:")
    print(res)
