# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_1t_Abstractclass


class QSR_MWE(QSR_Dyadic_1t_Abstractclass):
    def __init__(self):
        super(QSR_MWE, self).__init__()
        self._unique_id = "mwe"
        self._all_possible_relations = ("left", "together", "right")
        self._dtype = "points"

    def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
        return {
            data1.x < data2.x: "left",
            data1.x > data2.x: "right"
        }.get(True, "together")
