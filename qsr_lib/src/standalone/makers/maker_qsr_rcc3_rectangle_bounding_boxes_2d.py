# -*- coding: utf-8 -*-
"""Example that shows how to implement QSR makers.

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
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


class Maker_QSR_RCC3_Rectangle_Bounding_Boxes_2D(Maker_QSR_Abstractclass):
    """Make default QSRs and provide an example for others"""
    def __init__(self):
        self.qsr_type = "rcc3_rectangle_bounding_boxes_2d"  # must be the same that goes in the QSR_Lib.__const_qsrs_available
        self.required_fields = ["x1", "y1", "x2", "y2"]
        self.all_possible_relations = ["dc", "po", "o"]

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
        pairs = self.__return_all_possible_combinations(input_data)
        for p in pairs:
            ids.append([input_data.data[p[0]].id, input_data.data[p[1]].id])
            qsrs.append(self.__compute_pairwise_qsrs(input_data.data[p[0]].data, input_data.data[p[1]].data))
        # end of your code

        ret = Output_Data(ids=ids,
                          data=qsrs,
                          qsr_type=self.qsr_type,
                          all_possible_relations=self.all_possible_relations,
                          timestamp_request_received=kwargs["timestamp_request_received"],
                          timestamp_qsrs_processed=datetime.now())
        return ret

    # custom functions follow
    def __return_all_possible_combinations(self, input_data):
        ret = []
        for i in range(len(input_data.data)):
            for j in range(len(input_data.data)):
                if i != j:
                    ret.append([i, j])
        return ret

    def __compute_pairwise_qsrs(self, bb1, bb2):
        if len(bb1) != len(bb2):
            print("ERROR: lengths of bounding boxes for the two objects are not equal")
            return []
        timesteps = int(len(bb1) / len(self.required_fields))
        step = len(self.required_fields)
        ret = []
        for t in range(timesteps):
            i = t * step
            ret.append(self.__compute_qsr(bb1[i:i+step], bb2[i:i+step]))
        return ret

    def __compute_qsr(self, bb1, bb2):
        results = {0: "dc", 1: "po", 2: "o"}
        count_occluded_points = 0
        for i in range(0, len(bb2), 2):
            if self.__is_point_in_rectangle([bb2[i], bb2[i+1]], bb1):
                count_occluded_points += 1
        ret = results[count_occluded_points]
        return ret

    def __is_point_in_rectangle(self, p, r, d=0.):
        return p[0] >= r[0]-d and p[0] <= r[2]+d and p[1] >= r[0]-d and p[1] <= r[3]+d
