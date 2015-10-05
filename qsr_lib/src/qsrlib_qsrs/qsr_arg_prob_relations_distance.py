# -*- coding: utf-8 -*-
from __future__ import print_function, division
import numpy as np
from qsr_arg_relations_distance import QSR_Arg_Relations_Distance
from random import uniform


class QSR_Arg_Prob_Relations_Distance(QSR_Arg_Relations_Distance):
    """Probabilistic ard-distances.

    Values of the abstract properties
        * **_unique_id** = "argprobd"
        * **_all_possible_relations** = depends on what user has passed
        * **_dtype** = "points"

    QSR Parameters (for `dynamic_args`)
        * **'qsr_relations_and_values'**: A dictionary with keys being the relations labels and values

    .. seealso:: For further details, refer to its :doc:`description. <../handwritten/qsrs/argprobd>`
    """

    def __init__(self):
        """Constructor."""
        super(QSR_Arg_Prob_Relations_Distance, self).__init__()

        self._unique_id = "argprobd"
        """str: Unique identifier of a QSR."""

        self.allowed_value_types = (tuple, list)
        """tuple of datatypes: ?"""

        self.value_sort_key = lambda x: x[1][0] # Sort by first element in value tuple, i.e. mean
        """?"""

    def __normpdf(self, x, mu, sigma):
        """

        :param x:
        :type x:
        :param mu:
        :type mu:
        :param sigma:
        :type sigma:
        :return:
        :rtype:
        """
        u = (x-mu)/np.abs(sigma)
        y = (1/(np.sqrt(2*np.pi)*np.abs(sigma)))*np.exp(-u*u/2)
        return np.around(y, decimals=3)

    def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
        """

        :param data1:
        :type data1:
        :param data2:
        :type data2:
        :param qsr_params:
        :type qsr_params:
        :param kwargs: kwargs arguments.
        :return:
        :rtype:
        """
        d = np.sqrt(np.square(data1.x - data2.x) + np.square(data1.y - data2.y))
        r = (None, 0.0)
        for values, relation in zip(self.all_possible_values, self._all_possible_relations):
            prob = uniform(0.0, self.__normpdf(d, mu=values[0], sigma=values[1]))
            r = (relation, prob) if prob > r[1] else r
        return r[0] if r[0] else self._all_possible_relations[-1]
