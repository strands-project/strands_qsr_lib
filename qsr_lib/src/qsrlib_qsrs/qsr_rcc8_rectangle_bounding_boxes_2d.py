# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_rcc_abstractclass import QSR_RCC_Abstractclass


class QSR_RCC8_Rectangle_Bounding_Boxes_2D(QSR_RCC_Abstractclass):
    """RCC8 relations.

    Values of the abstract properties
        * **_unique_id** = "rcc8"
        * **_all_possible_relations** = ("dc", "ec", "po", "eq", "tpp", "ntpp", "tppi", "ntppi")
        * **_dtype** = "bounding_boxes_2d"

    .. seealso:: For further details about RCC8, refer to its :doc:`description. <../handwritten/qsrs/rcc8>`
    """

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

    _unique_id = "rcc8"
    """str: Unique identifier name of the QSR."""

    _all_possible_relations = ("dc", "ec", "po", "eq", "tpp", "ntpp", "tppi", "ntppi")
    """tuple: All possible relations of the QSR."""

    def __init__(self):
        """Constructor."""
        super(QSR_RCC8_Rectangle_Bounding_Boxes_2D, self).__init__()

    def _convert_to_requested_rcc_type(self, qsr):
        """No need for remapping.

        :param qsr: RCC8 value.
        :type qsr: str
        :return: RCC8 value.
        :rtype: str
        """
        return qsr
