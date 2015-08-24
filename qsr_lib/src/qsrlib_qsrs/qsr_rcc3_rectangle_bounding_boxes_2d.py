# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_rcc_abstractclass import QSR_RCC_Abstractclass


class QSR_RCC3_Rectangle_Bounding_Boxes_2D(QSR_RCC_Abstractclass):
    """Computes symmetrical RCC3 relations: 'dc':disconnected, 'po':partial overlap, 'o': occluded/part of

        # 'dc'     bb1 is disconnected from bb2
        # 'po'     bb1 partially overlaps bb2
        # 'o'      bb1 is occluded or part of bb2
    """
    def __init__(self):
        super(QSR_RCC3_Rectangle_Bounding_Boxes_2D, self).__init__()
        self._unique_id = "rcc3"
        self.all_possible_relations = ["dc", "po", "o"]

    def _convert_to_current_rcc(self, qsr):
        qsr = qsr.replace("ec", "po")
        return qsr if qsr in self.all_possible_relations else self.all_possible_relations[-1]
