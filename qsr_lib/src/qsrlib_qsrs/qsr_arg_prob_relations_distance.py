# -*- coding: utf-8 -*-
"""Class for probabilistic distance QSR

:Author: Christian Dondrup <cdondrup@lincoln.ac.uk>
:Organization: University of Lincoln
"""

from __future__ import print_function, division
import numpy as np
from qsr_arg_relations_distance import QSR_Arg_Relations_Distance
from qsrlib_io.world_qsr_trace import *


class QSR_Arg_Prob_Relations_Distance(QSR_Arg_Relations_Distance):
    def __init__(self, config=None):
        super(QSR_Arg_Prob_Relations_Distance, self).__init__()
        self.qsr_type = "arg_prob_relations_distance"
        self.qsr_keys = "argprobd"
        self.allowed_value_types = (tuple,list)
        self.value_sort_key = lambda x: x[1][0] # Sort by first element in value tuple, i.e. mean
        if config:
            self.set_from_config_file(config)

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

    def __normpdf(self, x, mu, sigma):
        u = (x-mu)/np.abs(sigma)
        y = (1/(np.sqrt(2*np.pi)*np.abs(sigma)))*np.exp(-u*u/2)
        return y

    def _compute_qsr(self, objs):
        d = np.sqrt(np.square(objs[0].x - objs[1].x) + np.square(objs[0].y - objs[1].y))
        r = (None, 0.0)
        for values, relation in zip(self.all_possible_values, self.all_possible_relations):
            prob = self.__normpdf(d, mu=values[0], sigma=values[1])
            r = (relation, prob) if prob > r[1] else r
        return r[0] if r[0] else self.all_possible_relations[-1]
