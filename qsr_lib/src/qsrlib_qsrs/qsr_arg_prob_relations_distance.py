# -*- coding: utf-8 -*-
"""Class for probabilistic distance QSR

:Author: Christian Dondrup <cdondrup@lincoln.ac.uk>
:Organization: University of Lincoln
"""

from __future__ import print_function, division
import numpy as np
from qsr_arg_relations_distance import QSR_Arg_Relations_Distance
from random import uniform


class QSR_Arg_Prob_Relations_Distance(QSR_Arg_Relations_Distance):
    def __init__(self, config=None):
        super(QSR_Arg_Prob_Relations_Distance, self).__init__()
        self._unique_id = "argprobd"
        self.allowed_value_types = (tuple, list)
        self.value_sort_key = lambda x: x[1][0] # Sort by first element in value tuple, i.e. mean
        if config:
            self.set_from_config_file(config)

    def custom_checks(self, input_data):
        """Write your own custom checks on top of the default ones


        :return: error code, error message (integer, string), use 10 and above for error code as 1-9 are reserved by system
        """
        return 0, ""

    def __normpdf(self, x, mu, sigma):
        u = (x-mu)/np.abs(sigma)
        y = (1/(np.sqrt(2*np.pi)*np.abs(sigma)))*np.exp(-u*u/2)
        return np.around(y, decimals=3)

    def _compute_qsr(self, objs):
        d = np.sqrt(np.square(objs[0].x - objs[1].x) + np.square(objs[0].y - objs[1].y))
        r = (None, 0.0)
        for values, relation in zip(self.all_possible_values, self.all_possible_relations):
            prob = uniform(0.0, self.__normpdf(d, mu=values[0], sigma=values[1]))
            r = (relation, prob) if prob > r[1] else r
        return r[0] if r[0] else self.all_possible_relations[-1]
