# -*- coding: utf-8 -*-
"""Example that shows how to implement QSR makers.

:Author: Christan Dondrup <cdondrup@lincoln.ac.uk>, Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
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
from qsrlib_qsrs.qsr_qtc_simplified_abstractclass import QSR_QTC_Simplified_Abstractclass
from qsrlib_io.world_qsr_trace import *
import math
import sys


class QSR_QTC_B_Simplified(
        QSR_Abstractclass,
        QSR_QTC_Simplified_Abstractclass
        ):
    """Make default QSRs and provide an example for others"""
    def __init__(self):
        super(QSR_QTC_B_Simplified, self).__init__()
        self.qtc_type = "b"
        self.qsr_type = "qtc_b_simplified"  # must be the same that goes in the QSR_Lib.__const_qsrs_available
        self.all_possible_relations = self.return_all_possible_state_combinations()[0]

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
        timestamps = input_data.get_sorted_timestamps()
        objects_names = sorted(input_data.trace[timestamps[0]].objects.keys())
        o1_name = objects_names[0]
        o2_name = objects_names[1]
        between = o1_name + "," + o2_name
        timestamps = input_data.get_sorted_timestamps()
        for t0, t1 in zip(timestamps, timestamps[1:]):
            timestamp = t1
            try:
                k = [input_data.trace[t0].objects[o1_name].x,
                     input_data.trace[t0].objects[o1_name].y,
                     input_data.trace[t1].objects[o1_name].x,
                     input_data.trace[t1].objects[o1_name].y]
                l = [input_data.trace[t0].objects[o2_name].x,
                     input_data.trace[t0].objects[o2_name].y,
                     input_data.trace[t1].objects[o2_name].x,
                     input_data.trace[t1].objects[o2_name].y]
                qtc = self.create_qtc_representation(k, l, 0.)
                qtc = str(qtc[0]) + "," + str(qtc[1])
                qsr = QSR(
                    timestamp=timestamp,
                    between=between,
                    qsr=qtc
                )
                ret.add_qsr(qsr, timestamp)
            except KeyError:
                ret.add_empty_world_qsr_state(timestamp)
        return ret
