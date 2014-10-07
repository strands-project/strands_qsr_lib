# -*- coding: utf-8 -*-
"""Example that shows how to implement QSR makers.

:Author: Christan Dondrup <>
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
from datetime import datetime
from maker_qsr_abstractclass import Maker_QSR_Abstractclass
import os
import sys
import inspect
# http://stackoverflow.com/questions/279237/import-a-module-from-a-relative-path
add_folder = os.path.realpath(os.path.abspath(os.path.split(inspect.getfile( inspect.currentframe() ))[0]))
if add_folder not in sys.path:
    sys.path.insert(0, add_folder)
if "/makers" in add_folder:
        add_folder = add_folder[0:-len("makers")]
if add_folder not in sys.path:
    sys.path.insert(0, add_folder)
from output_data import Output_Data
import math

class Maker_QSR_QTC_B_Simplified(Maker_QSR_Abstractclass):
    """Make default QSRs and provide an example for others"""
    def __init__(self):
        self.qsr_type = "qtc_b_simplified"  # must be the same that goes in the QSR_Lib.__const_qsrs_available
        self.required_fields = ["x", "y"]
        self.all_possible_relations = ["0,0", "0,1", "1,0", "1,1", "0,-1", "-1,0", "-1,-1"]  # 0: no change, 1: moving away, -1: moving towards

    def custom_help(self):
        """Write your own help message function"""
        print("where,\nx1, y2: the xy-coords of the top-left corner of the rectangle\nx2, y2: the xy-coords of the bottom-right corner of the rectangle")

    def custom_checks(self):
        """Write your own custom checks on top of the default ones


        :return: error code, error message (integer, string), use 10 and above for error code as 1-9 are reserved by system
        """
        return 0, ""

    def make(self, *args, **kwargs):
        """Make the QSRs

        :param args: not used at the moment
        :param kwargs: a dictionary holding the input data; at the moment it has the following:
                        "input_data": Input_Data_Block object
        :return: Output_Data object
        """
        input_data = kwargs["input_data"]
        error_code, error_msg = self.check_input(input_data=input_data)
        if error_code > 0:
            print("ERROR:", error_msg)
            self.help()
            print("\nFailed to compute QSRs")
            return Output_Data(qsr_type="error")
        ids = []
        qsrs = []

        # begin of your code, feel free to write your own methods within the class to keep code neat
        ids.append([input_data.data[0].id, input_data.data[1].id])
        qsrs.append(self.__compute_qsrs(input_data.data[0].data, input_data.data[1].data))
        # end of your code

        ret = Output_Data(ids=ids,
                          data=qsrs,
                          qsr_type=self.qsr_type,
                          all_possible_relations=self.all_possible_relations,
                          timestamp_request_received=kwargs["timestamp_request_received"],
                          timestamp_qsrs_processed=datetime.now())
        return ret

    def __compute_qsrs(self, k, l):
        if len(k) != len(l):
            print("ERROR: lengths of trajectories for the two objects are not equal")
            return []
        timesteps = int(len(k) / len(self.required_fields))
        step = len(self.required_fields)
        ret = []
        for t in range(timesteps-1):
            i = t * step
            ret.append(self.__compute_qsr(k[i:i+2*step], l[i:i+2*step]))
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
        print(euc)
        return euc
