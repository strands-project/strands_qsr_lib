# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_rcc_abstractclass import QSR_RCC_Abstractclass


class QSR_RCC5(QSR_RCC_Abstractclass):
    """Symmetrical RCC5 relations.

    Values of the abstract properties
        * **_unique_id** = "rcc5"
        * **_all_possible_relations** = ("dr", "po", "pp", "ppi", "eq")
        * **_dtype** = "bounding_boxes_2d"

    QSR specific `dynamic_args`
        * **'quantisation_factor'** (*float*) = 0.0: Threshold that determines whether two rectangle regions are disconnected.

    .. seealso:: For further details about RCC5, refer to its :doc:`description. <../handwritten/qsrs/rcc5>`
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
