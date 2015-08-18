# -*- coding: utf-8 -*-
"""Computes MoS relations: 'm': moving, 's': stationary

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 10 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""

from __future__ import print_function, division
import numpy as np
from qsrlib_qsrs.qsr_abstractclass import QSR_Abstractclass
from qsrlib_io.world_qsr_trace import *


class QSR_Moving_or_Stationary(QSR_Abstractclass):
    def __init__(self):
        super(QSR_Moving_or_Stationary, self).__init__()
        self._unique_id = "mos"
        self.all_possible_relations = ["m", "s"]

    def custom_set_from_config_file(self, document):
        pass

    def custom_help(self):
        """Write your own help message function"""
        print("")

    def custom_checks(self, input_data):
        """Write your own custom checks on top of the default ones


        :return: error code, error message (integer, string), use 10 and above for error code as 1-9 are reserved by system
        """
        return 0, ""

    def custom_checks_for_qsrs_for(self, qsrs_for, error_found):
        """qsrs_for must be list of strings.

        :param qsrs_for:
        :param error_found:
        :return: qsrs_for, error_found
        """
        for p in list(qsrs_for):
            if type(p) is not str:
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
        quantisation_factor = 0.0
        try:
            quantisation_factor = float(kwargs["dynamic_args"]["quantisation_factor"])
            print("Warning: This feature is deprecated, use dynamic_args with the namespace '%s' on your request message instead" % self._unique_id)
        except:
            pass

        try:
            quantisation_factor = float(kwargs["dynamic_args"][self._unique_id]["quantisation_factor"])
        except:
            pass

        ret = World_QSR_Trace(qsr_type=self._unique_id)
        ts = input_data.get_sorted_timestamps()
        for t, tp in zip(ts[1:], ts):
            world_state_now = input_data.trace[t]
            world_state_previous = input_data.trace[tp]
            if kwargs["qsrs_for"]:
                qsrs_for, error_found = self.check_qsrs_for_data_exist(world_state_now.objects.keys(), kwargs["qsrs_for"])
            else:
                qsrs_for = world_state_now.objects.keys()
            if qsrs_for:
                for between in qsrs_for: # yes between is probably not the best name as it is simply object_name
                    point_now = (world_state_now.objects[between].x, world_state_now.objects[between].y)
                    point_previous = (world_state_previous.objects[between].x, world_state_previous.objects[between].y)
                    qsr = QSR(timestamp=t, between=between,
                              qsr=self.handle_future(kwargs["future"],
                                                     self.__compute_qsr(point_now, point_previous, quantisation_factor),
                                                     self._unique_id))
                    ret.add_qsr(qsr, t)
            else:
                if include_missing_data:
                    ret.add_empty_world_qsr_state(t)
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

    def __compute_qsr(self, p1, p2, quantisation_factor=0.0):
        """Return MOS relation

        :param p1: point (x1, y1)
        :param p2: point (x2, y2)
        :moving_thres: determines minimal displacement to be considered as a moving
        :return: a MoS relation from the following: 'm': moving, 'c': stationary
        """
        return "m" if np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2) > quantisation_factor else "s"
