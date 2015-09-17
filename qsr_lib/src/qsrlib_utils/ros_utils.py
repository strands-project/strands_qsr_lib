# -*- coding: utf-8 -*-
from __future__ import print_function, division
import rospy
import datetime


def convert_pythondatetime_to_rostime(pythondatetime):
    """Convert datetime from python format to ROS format.

    :param pythondatetime: Python format datetime.
    :type pythondatetime: datetime
    :return: ROS time.
    :rtype: rospy.Time
    """
    tsecs = (pythondatetime - datetime.datetime.utcfromtimestamp(0)).total_seconds()
    return rospy.Time.from_sec(tsecs)
