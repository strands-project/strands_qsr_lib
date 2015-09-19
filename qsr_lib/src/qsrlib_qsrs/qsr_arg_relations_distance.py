# -*- coding: utf-8 -*-
from __future__ import print_function, division
import numpy as np
import operator
from qsrlib_qsrs.qsr_arg_relations_abstractclass import QSR_Arg_Relations_Abstractclass
from qsrlib_io.world_qsr_trace import *


class QSR_Arg_Relations_Distance(QSR_Arg_Relations_Abstractclass):
    """Argument distance relations.

    Values of the abstract properties
        * **_unique_id** = "argd"
        * **_all_possible_relations** = depends on what user has passed
        * **_dtype** = "points"

    Some explanation about the QSR or better link to a separate webpage explaining it. Maybe a reference if it exists.
    """

    _unique_id = "argd"
    """str: Unique identifier of the QSR."""

    _all_possible_relations = ()
    """tuple: All possible relations of the QSR."""

    _dtype = "points"
    """str: Kind of data the QSR operates with, see self._dtype_map for possible values."""

    def __init__(self):
        """Constructor."""
        super(QSR_Arg_Relations_Distance, self).__init__()

        self.allowed_value_types = (int, float)
        """?"""

        self.value_sort_key = operator.itemgetter(1)  # Sort by threshold
        """?"""

    def _process_qsr_parameters_from_request_parameters(self, req_params, **kwargs):
        """

        :param req_params:
        :type req_params:
        :param kwargs: kwargs arguments.
        :raises: KeyError
        """
        try:
            self._set_qsr_relations_and_values(qsr_relations_and_values=req_params["dynamic_args"][self._unique_id]["qsr_relations_and_values"])
        except KeyError:
            raise KeyError("qsr_relations_and_values not set")

    def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
        """

        :param data1:
        :type data1:
        :param data2:
        :type data2:
        :param qsr_params:
        :type qsr_params:
        :param kwargs: kwargs arguments.
        :return: argd relation.
        :rtype: str
        """
        if np.isnan(data1.z) or np.isnan(data2.z):
            d = np.sqrt(np.square(data1.x - data2.x) + np.square(data1.y - data2.y))
        else:
            d = np.sqrt(np.square(data1.x - data2.x) + np.square(data1.y - data2.y) + np.square(data1.z - data2.z))
        for thres, relation in zip(self.all_possible_values, self._all_possible_relations):
            if d <= thres:
                return relation
        return self._all_possible_relations[-1]
