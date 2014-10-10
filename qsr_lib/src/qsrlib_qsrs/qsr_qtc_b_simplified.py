# -*- coding: utf-8 -*-
"""Example that shows how to implement QSR makers.

:Author: Christan Dondrup <>, Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Lincoln
:Date: 10 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
:Notes: future extension to handle polygons, to do that use matplotlib.path.Path.contains_points
        although might want to have a read on the following also...
        http://matplotlib.1069221.n5.nabble.com/How-to-properly-use-path-Path-contains-point-td40718.html
"""

from __future__ import print_function, division
from qsrlib_qsrs.qsr_abstractclass import QSR_Abstractclass
from qsrlib_io.qsr_trace import *
import math

class QSR_QTC_B_Simplified(QSR_Abstractclass):
    """Make default QSRs and provide an example for others"""
    def __init__(self):
        self.qsr_type = "qtc_b_simplified"  # must be the same that goes in the QSR_Lib.__const_qsrs_available
        self.all_possible_relations = ["0,0", "0,1", "1,0", "1,1", "0,-1", "-1,0", "-1,-1"]  # 0: no change, 1: moving away, -1: moving towards

    def custom_help(self):
        """Write your own help message function"""
        print("where,\nx1, y2: the xy-coords of the top-left corner of the rectangle\nx2, y2: the xy-coords of the bottom-right corner of the rectangle")

    def custom_checks(self, input_data):
        """Write your own custom checks on top of the default ones


        :return: error code, error message (integer, string), use 10 and above for error code as 1-9 are reserved by system
        """
        return 0, ""

    def make(self, *args, **kwargs):
        """Make the QSRs

        :param args: not used at the moment
        :param kwargs:
                        - input_data: World_Trace
        :return: World_QSR_Trace
        """
        input_data = kwargs["input_data"]
        ret = World_QSR_Trace(qsr_type=self.qsr_type)
        objects_names = sorted(input_data.trace[input_data.timestamps[0]].objects.keys())
        o1_name = objects_names[0]
        o2_name = objects_names[1]
        between = o1_name + "," + o2_name
        for t0, t1 in zip(input_data.timestamps, input_data.timestamps[1:]):
            timestamp = t1
            k = [input_data.trace[t0].objects[o1_name].x,
                 input_data.trace[t0].objects[o1_name].y,
                 input_data.trace[t1].objects[o1_name].x,
                 input_data.trace[t1].objects[o1_name].y]
            l = [input_data.trace[t0].objects[o2_name].x,
                 input_data.trace[t0].objects[o2_name].y,
                 input_data.trace[t1].objects[o2_name].x,
                 input_data.trace[t1].objects[o2_name].y]
            qsr = QSR(timestamp=timestamp, between=between, qsr=self.__compute_qsr(k ,l))
            ret.add_qsr_to_trace(qsr, timestamp)
        return ret

    def __compute_qsr(self, k, l):
        euclidean_initial = self.__euclidean(k[0:2], l[0:2])
        if k[0] == k[2] and k[1] == k[3]:
            qsr_k = "0"
        elif self.__euclidean(k[2:], l[0:2]) < euclidean_initial:
            qsr_k = "-1"
        else:
            qsr_k = "1"

        if l[0] == l[2] and l[1] == l[3]:
            qsr_l = "0"
        elif self.__euclidean(l[2:], k[0:2]) < euclidean_initial:
            qsr_l = "-1"
        else:
            qsr_l = "1"
        ret = qsr_k + "," + qsr_l
        return ret

    def __euclidean(self, a, b):
        euc = math.sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]))
        # print(euc)
        return euc
