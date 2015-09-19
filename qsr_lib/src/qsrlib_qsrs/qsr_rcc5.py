# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_rcc_abstractclass import QSR_RCC_Abstractclass


class QSR_RCC5(QSR_RCC_Abstractclass):
    """Computes symmetrical RCC5 relations

    """

    _unique_id = "rcc5"

    _all_possible_relations = ("dr", "po", "pp", "ppi", "eq")

    __mapping_from_rcc8 = {"dc": "dr",
                           "ec": "dr",
                           "po": "po",
                           "tpp": "pp",
                           "ntpp": "pp",
                           "tppi": "ppi",
                           "ntppi": "ppi",
                           "eq": "eq"}

    def __init__(self):
        super(QSR_RCC5, self).__init__()

    def _convert_to_requested_rcc_type(self, qsr):
        return self.__mapping_from_rcc8[qsr]
