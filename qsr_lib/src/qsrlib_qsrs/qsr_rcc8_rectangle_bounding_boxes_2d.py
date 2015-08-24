# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_rcc_abstractclass import QSR_RCC_Abstractclass


class QSR_RCC8_Rectangle_Bounding_Boxes_2D(QSR_RCC_Abstractclass):
    """RCC8

        # 'dc'     bb1 is disconnected from bb2
        # 'ec'     bb1 is externally connected with bb2
        # 'po'     bb1 partially overlaps bb2
        # 'eq'     bb1 equals bb2
        # 'tpp'    bb1 is a tangential proper part of bb2
        # 'ntpp'   bb1 is a non-tangential proper part of bb2
        # 'tppi'   bb2 is a tangential proper part of bb1
        # 'ntppi'  bb2 is a non-tangential proper part of bb1
    """
    def __init__(self):
        super(QSR_RCC8_Rectangle_Bounding_Boxes_2D, self).__init__()
        self._unique_id = "rcc8"
        self.all_possible_relations = ["dc", "ec", "po", "eq", "tpp", "ntpp", "tppi", "ntppi"]

    def _convert_to_current_rcc(self, qsr):
        return qsr
