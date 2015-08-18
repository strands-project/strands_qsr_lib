# -*- coding: utf-8 -*-
"""Computes RCC2 relations: 'dc':disconnected, 'c': c

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
from qsrlib_qsrs.qsr_abstractclass import QSR_Abstractclass
from qsrlib_io.world_qsr_trace import *


class QSR_RCC2_Rectangle_Bounding_Boxes_2D(QSR_Abstractclass):
    """Make default QSRs and provide an example for others"""
    def __init__(self):
        super(QSR_RCC2_Rectangle_Bounding_Boxes_2D, self).__init__()
        self._unique_id = "rcc2"
        self.all_possible_relations = ["dc", "c"]

    def custom_set_from_config_file(self, document):
        pass

    def custom_help(self):
        """Write your own help message function"""
        print("where,\nx1, y2: the xy-coords of the top-left corner of the rectangle\nx2, y2: the xy-coords of the bottom-right corner of the rectangle")

    def custom_checks(self, input_data):
        """Write your own custom checks on top of the default ones


        :return: error code, error message (integer, string), use 10 and above for error code as 1-9 are reserved by system
        """
        return 0, ""

    def custom_checks_for_qsrs_for(self, qsrs_for, error_found):
        """qsrs_for must be tuples of two objects.

        :param qsrs_for: list of strings and/or tuples for which QSRs will be computed
        :param error_found: if an error was found in the qsrs_for that violates the QSR rules
        :return: qsrs_for, error_found
        """
        for p in list(qsrs_for):
            if (type(p) is not tuple) and (type(p) is not list) and (len(p) != 2):
                qsrs_for.remove(p)
                error_found = True
        return qsrs_for, error_found

    def make(self, *args, **kwargs):
        """Make the QSRs

        :param args: not used at the moment
        :param kwargs:
                        - input_data: World_Trace
        :return: World_QSR_Trace
        """
        input_data = kwargs["input_data"]
        include_missing_data = kwargs["include_missing_data"]
        ret = World_QSR_Trace(qsr_type=self._unique_id)
        try:
            quantisation_factor = float(kwargs["dynamic_args"][self._unique_id]["quantisation_factor"])
        except KeyError:
            try:
                quantisation_factor = float(kwargs["dynamic_args"]["quantisation_factor"])
                print("Warning: This feature is deprecated, use dynamic_args with the namespace '%s' on your request message instead" % self._unique_id)
            except (KeyError, TypeError):
                quantisation_factor = 0.0
        except TypeError:
            quantisation_factor = 0.0

        for t in input_data.get_sorted_timestamps():
            world_state = input_data.trace[t]
            timestamp = world_state.timestamp
            if kwargs["qsrs_for"]:
                qsrs_for, error_found = self.check_qsrs_for_data_exist(world_state.objects.keys(), kwargs["qsrs_for"])
            else:
                qsrs_for = self.__return_all_possible_combinations(world_state.objects.keys())
            if qsrs_for:
                for p in qsrs_for:
                    between = str(p[0]) + "," + str(p[1])
                    bb1 = world_state.objects[p[0]].return_bounding_box_2d()
                    bb2 = world_state.objects[p[1]].return_bounding_box_2d()
                    qsr = QSR(timestamp=timestamp, between=between,
                              qsr=self.handle_future(kwargs["future"], self.__compute_qsr(bb1, bb2, quantisation_factor), self._unique_id))
                    ret.add_qsr(qsr, timestamp)
            else:
                if include_missing_data:
                    ret.add_empty_world_qsr_state(timestamp)
        return ret

    # custom functions follow
    def __return_all_possible_combinations(self, objects_names):
        if len(objects_names) < 2:
            return []
        ret = []
        for i in objects_names:
            for j in objects_names:
                if i != j:
                    ret.append((i, j))
        return ret

    def __compute_qsr(self, bb1, bb2, q=0.0):
        """Return RCC2 relation

        :param bb1: diagonal points coordinates of first bounding box (x1, y1, x2, y2)
        :param bb2: diagonal points coordinates of second bounding box (x1, y1, x2, y2)
        :return: an RCC2 relation from the following: 'dc':disconnected, 'c': connected
        """
        return "dc" if (bb1[0] > bb2[2]+q) or (bb1[2] < bb2[0]-q) or (bb1[1] > bb2[3]+q) or (bb1[3] < bb2[1]-q) else "c"
