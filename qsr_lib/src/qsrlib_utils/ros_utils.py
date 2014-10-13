# -*- coding: utf-8 -*-
"""Utilities functions in ROS

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 22 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""


from __future__ import print_function, division
import rospy
import datetime

def convert_pythondatetime_to_rostime(pythondatetime):
    tsecs = (pythondatetime - datetime.datetime.utcfromtimestamp(0)).total_seconds()
    return rospy.Time.from_sec(tsecs)
