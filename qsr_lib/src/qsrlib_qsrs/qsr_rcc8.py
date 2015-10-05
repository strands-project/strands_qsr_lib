# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_rcc_abstractclass import QSR_RCC_Abstractclass


class QSR_RCC8(QSR_RCC_Abstractclass):
    """Symmetrical RCC5 relations.

    Values of the abstract properties
        * **_unique_id** = "rcc8"
        * **_all_possible_relations** = ("dc", "ec", "po", "eq", "tpp", "ntpp", "tppi", "ntppi")
        * **_dtype** = "bounding_boxes_2d"

    QSR specific `dynamic_args`
        * **'quantisation_factor'** (*float*) = 0.0: Threshold that determines whether two rectangle regions are disconnected.

    .. seealso:: For further details about RCC8, refer to its :doc:`description. <../handwritten/qsrs/rcc8>`
    """

    _unique_id = "rcc8"
    """str: Unique identifier name of the QSR."""

    _all_possible_relations = ("dc", "ec", "po", "eq", "tpp", "ntpp", "tppi", "ntppi")
    """tuple: All possible relations of the QSR."""

    def __init__(self):
        """Constructor."""
        super(QSR_RCC8, self).__init__()

    def _convert_to_requested_rcc_type(self, qsr):
        """No need for remapping.

        :param qsr: RCC8 value.
        :type qsr: str
        :return: RCC8 value.
        :rtype: str
        """
        return qsr
