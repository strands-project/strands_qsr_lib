# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_rcc_abstractclass import QSR_RCC_Abstractclass


class QSR_RCC2(QSR_RCC_Abstractclass):
    """Symmetrical RCC2 relations.

    Values of the abstract properties
        * **_unique_id** = "rcc2"
        * **_all_possible_relations** = ("dc", "c")
        * **_dtype** = "bounding_boxes_2d"

    QSR specific `dynamic_args`
        * **'quantisation_factor'** (*float*) = 0.0: Threshold that determines whether two rectangle regions are disconnected.

    .. seealso:: For further details about RCC2, refer to its :doc:`description. <../handwritten/qsrs/rcc2>`
    """

    _unique_id = "rcc2"
    """str: Unique identifier name of the QSR."""

    _all_possible_relations = ("dc", "c")
    """tuple: All possible relations of the QSR."""

    def __init__(self):
        """Constructor."""
        super(QSR_RCC2, self).__init__()

    def _convert_to_requested_rcc_type(self, qsr):
        """Remap QSR values from RCC8 to RCC2.

        :param qsr: RCC8 value.
        :return: RCC2 value.
        :rtype: str
        """
        return qsr if qsr == "dc" else "c"
