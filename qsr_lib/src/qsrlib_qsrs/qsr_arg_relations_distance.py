# -*- coding: utf-8 -*-
"""Abstract class for unary dynamic categorical QSRs

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
"""

from __future__ import print_function, division
import numpy as np
import ConfigParser
from qsr_arg_relations_abstractclass import QSR_Arg_Relations_Abstractclass
from qsrlib_io.world_qsr_trace import *

class QSR_Arg_Relations_Distance(QSR_Arg_Relations_Abstractclass):
    def __init__(self, ini=None):
        super(QSR_Arg_Relations_Distance, self).__init__()
        self.qsr_type = "arg_relations_distance"
        self.qsr_keys = "argd"
        if ini:
            self.set_from_ini(ini=ini)

    def custom_set_from_ini(self, parser):
        try:
            relations_and_values = parser.get(self.qsr_type, "relations_and_values")
        except ConfigParser.NoSectionError, ConfigParser.NoOptionError:
            raise
        try:
            relations_and_values = eval(relations_and_values)
        except:
            raise ValueError
        self.set_qsr_relations_and_values(qsr_relations_and_values=relations_and_values)

    def custom_help(self):
        """Write your own help message function"""
        print("")

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
        # optional set from ini
        try:
            if kwargs["ini"]:
                self.set_from_ini(kwargs["ini"])
        except:
            pass
        # optional direct set, deprecated way
        try:
            if kwargs["qsr_relations_and_values"]:
                print("Warning: This feature is deprecated, use dynamic_args on your request message instead")
                self.set_qsr_relations_and_values(qsr_relations_and_values=kwargs["qsr_relations_and_values"])
        except:
            pass
        # optional direct set
        try:
            if kwargs["dynamic_args"]["qsr_relations_and_values"]:
                # print(">> dynamic args")  # dbg
                self.set_qsr_relations_and_values(qsr_relations_and_values=kwargs["dynamic_args"]["qsr_relations_and_values"])
        except:
            pass
        # print(self.qsr_relations_and_values)  # dbg
        if not self.qsr_relations_and_values:
            raise ValueError("qsr_relations_and_values is uninitialized,"
                             "use dynamic_args={'qsr_relations_and_values': <your dictionary of relations and values>"
                             "in the QSRlib_Request_Message")
        input_data = kwargs["input_data"]
        include_missing_data = kwargs["include_missing_data"]
        ret = World_QSR_Trace(qsr_type=self.qsr_type)
        for t in input_data.get_sorted_timestamps():
            world_state = input_data.trace[t]
            timestamp = world_state.timestamp
            if kwargs["qsrs_for"]:
                # print(kwargs["qsrs_for"])
                qsrs_for, error_found = self.check_qsrs_for_data_exist(world_state.objects.keys(), kwargs["qsrs_for"])
            else:
                qsrs_for = self.qsrs_for_default(world_state.objects.keys())
            if qsrs_for:
                for p in qsrs_for:
                    between = str(p[0]) + "," + str(p[1])
                    objs = (world_state.objects[p[0]], world_state.objects[p[1]])
                    qsr = QSR(timestamp=timestamp, between=between,
                              qsr=self.handle_future(kwargs["future"], self.__compute_qsr(objs), self.qsr_keys))
                    ret.add_qsr(qsr, timestamp)
            else:
                if include_missing_data:
                    ret.add_empty_world_qsr_state(timestamp)
        return ret

    def __compute_qsr(self, objs):
        if np.isnan(objs[0].z) or np.isnan(objs[1].z):
            d = np.sqrt(np.square(objs[0].x - objs[1].x) + np.square(objs[0].y - objs[1].y))
        else:
            d = np.sqrt(np.square(objs[0].x - objs[1].x) + np.square(objs[0].y - objs[1].y) + np.square(objs[0].z - objs[1].z))
        for thres, relation in zip(self.all_possible_values, self.all_possible_relations):
            if d <= thres:
                return relation
        return self.all_possible_relations[-1]

    def qsrs_for_default(self, objects_names):
        if len(objects_names) < 2:
            return []
        ret = []
        for i in sorted(objects_names):
            for j in objects_names:
                if i != j and (j, i) not in ret:
                    ret.append((i, j))
        return ret

