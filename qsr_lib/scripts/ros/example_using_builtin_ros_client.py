#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Usage example of the ROS builtin example client.

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 22 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""

from __future__ import print_function, division
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
from input_data import Input_Data_One, Input_Data_Block
from qsrlib_ros_client import QSRlib_ROS_Client

if __name__ == "__main__":
    # define some dummy sample data
    input_data = Input_Data_Block(data=[Input_Data_One("1", [1.0, 1.0, 2.0, 2.0, 1.0, 1.0, 2.0, 2.0, 1.0, 1.0, 2.0, 2.0]),
                                        Input_Data_One("2", [11.0, 11.0, 22.0, 22.0, 1.5, 1.5, 22.0, 22.0, 1.5, 1.5, 1.5, 1.5])],
                                  fields=["x1", "y1", "x2", "y2"],
                                  timesteps=3,
                                  description="some 2d bounding boxes")
    # make a QSRlib ROS client object
    cln = QSRlib_ROS_Client()
    # make the request message part of the service
    req = cln.make_request_message(which_qsr="rcc3_rectangle_bounding_boxes_2d", input_data=input_data)
    # request qsrs
    res = cln.request_qsrs(req)
    # output the results (in QSRsRequestResponse message format)
    print("--------------")
    print("Response is:")
    print(res)
