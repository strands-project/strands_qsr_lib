# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_rcc_abstractclass import QSR_RCC_Abstractclass


class QSR_RCC4(QSR_RCC_Abstractclass):
    """Symmetrical RCC4 relations.

    Values of the abstract properties
        * **_unique_id** = "rcc4"
        * **_all_possible_relations** = ("dc", "po", "pp", "ppi")
        * **_dtype** = "bounding_boxes_2d"

    QSR specific `dynamic_args`
        * **'quantisation_factor'** (*float*) = 0.0: Threshold that determines whether two rectangle regions are disconnected.

    .. seealso:: For further details about RCC4, refer to its :doc:`description. <../handwritten/qsrs/rcc4>`
    """
    _unique_id = "rcc4"

    _all_possible_relations = ("dc", "po", "pp", "ppi")

    __mapping_from_rcc8 = {"dc": "dc",
                           "ec": "po",
                           "po": "po",
                           "tpp": "pp",
                           "ntpp": "pp",
                           "eq": "pp",
                           "tppi": "ppi",
                           "ntppi": "ppi"}

    def __init__(self):
        super(QSR_RCC4, self).__init__()

    def _convert_to_requested_rcc_type(self, qsr):
        return self.__mapping_from_rcc8[qsr]
