# -*- coding: utf-8 -*-
from __future__ import print_function, division
import numpy as np
import operator
from qsr_arg_relations_abstractclass import QSR_Arg_Relations_Abstractclass
from qsrlib_io.world_qsr_trace import *

class QSR_Arg_Relations_Distance(QSR_Arg_Relations_Abstractclass):
    def __init__(self, config=None):
        super(QSR_Arg_Relations_Distance, self).__init__()
        self._unique_id = "argd"
        self.allowed_value_types = (int, float)
        self.value_sort_key = operator.itemgetter(1) # Sort by threshold
        if config:
            self.set_from_config_file(config)

    def custom_set_from_config_file(self, document):
        try:
            relations_and_values = document[self._unique_id]["relations_and_values"]
        except:
            print("ERROR (qsr_arg_relations_distance.py/custom_set_from_config_file):"
                  "'%s' or 'relations_and_values' not found in config file" % self._unique_id)
            self.qsr_relations_and_values = None
            self.all_possible_relations = None
            self.all_possible_values = None
            raise LookupError
        self.set_qsr_relations_and_values(qsr_relations_and_values=relations_and_values)

    def custom_help(self):
        """Write your own help message function"""
        print("")

    def custom_checks(self, input_data):
        """Write your own custom checks on top of the default ones


        :return: error code, error message (integer, string), use 10 and above for error code as 1-9 are reserved by system
        """
        return 0, ""

    # todo IMPORTANT: qsr_relations_and_values should not be a member, but enforced to be passed everytime
    # todo this is incompatible with how the rest of the QSRs are obtaining their parameters
    def _process_qsr_parameters_from_request_parameters(self, req_params, **kwargs):
        # todo need to integrate the read from config file somewhere, or deprecate the config file on the server side completely; can then provide a client-side utility for it
        # try:
        #     if kwargs["config"]:
        #         self.set_from_config_file(kwargs["config"])
        # except:
        #     pass

        try:
            self.set_qsr_relations_and_values(qsr_relations_and_values=kwargs["dynamic_args"][self._unique_id]["qsr_relations_and_values"])
        except KeyError:
            raise KeyError("qsr_relations_and_values not set")
        return None

    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, **kwargs):
        ret = World_QSR_Trace(qsr_type=self._unique_id)
        for t in world_trace.get_sorted_timestamps():
            world_state = world_trace.trace[t]
            qsrs_for = self._process_qsrs_for(world_state.objects.keys(), kwargs)
            for p in qsrs_for:
                between = str(p[0]) + "," + str(p[1])
                objs = (world_state.objects[p[0]], world_state.objects[p[1]])
                ret.add_qsr(QSR(timestamp=t, between=between, qsr=self.format_qsr(self._compute_qsr(objs))),
                            t)
        return ret

    def _compute_qsr(self, objs):
        if np.isnan(objs[0].z) or np.isnan(objs[1].z):
            d = np.sqrt(np.square(objs[0].x - objs[1].x) + np.square(objs[0].y - objs[1].y))
        else:
            d = np.sqrt(np.square(objs[0].x - objs[1].x) + np.square(objs[0].y - objs[1].y) + np.square(objs[0].z - objs[1].z))
        for thres, relation in zip(self.all_possible_values, self.all_possible_relations):
            if d <= thres:
                return relation
        return self.all_possible_relations[-1]
