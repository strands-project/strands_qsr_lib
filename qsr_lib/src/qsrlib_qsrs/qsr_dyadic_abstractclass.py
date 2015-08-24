# -*- coding: utf-8 -*-
from __future__ import print_function, division
import abc
from qsrlib_qsrs.qsr_abstractclass import QSR_Abstractclass
from qsrlib_utils.combinations_and_permutations import *

class QSR_Dyadic_Abstractclass(QSR_Abstractclass):
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        super(QSR_Dyadic_Abstractclass, self).__init__()

    def _init_qsrs_for_default(self, objects_names_of_world_state, req_params, **kwargs):
        return possible_pairs(objects_names_of_world_state)

    def custom_checks_for_qsrs_for(self, qsrs_for):
        """qsrs_for must be tuples of two objects.

        :param qsrs_for: list of strings and/or tuples for which QSRs will be computed
        :return: A list of objects names pairs to make QSRs for
        :rtype: list
        """
        return [p for p in qsrs_for if isinstance(p, (list, tuple)) and (len(p) == 2)]
