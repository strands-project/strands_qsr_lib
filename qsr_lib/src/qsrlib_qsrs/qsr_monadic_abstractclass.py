# -*- coding: utf-8 -*-
from __future__ import print_function, division
import abc
from qsrlib_qsrs.qsr_abstractclass import QSR_Abstractclass

class QSR_Monadic_Abstractclass(QSR_Abstractclass):
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        super(QSR_Monadic_Abstractclass, self).__init__()

    def _init_qsrs_for_default(self, objects_names_of_world_state, req_params, **kwargs):
        return objects_names_of_world_state

    def _validate_qsrs_for(self, qsrs_for):
        """qsrs_for must be list of strings.

        :param qsrs_for:
        :type qsrs_for: list or tuple
        :return: A list of string objects names to make QSRs for
        :rtype: list
        """
        return [p for p in qsrs_for if isinstance(p, str)]



