# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_rcc_abstractclass import QSR_RCC_Abstractclass


class QSR_RCC3_Rectangle_Bounding_Boxes_2D(QSR_RCC_Abstractclass):
    """Symmetrical RCC5 relations.

    .. warning::
        RCC3 relations symbols are under consideration and might change in the near future.

    Values of the abstract properties
        * **_unique_id** = "rcc3"
        * **_all_possible_relations** = ("dc", "po", "o")
        * **_dtype** = "bounding_boxes_2d"

    QSR specific `dynamic_args`
        * **'quantisation_factor'** (*float*) = 0.0: Threshold that determines whether two rectangle regions are disconnected.

    .. seealso:: For further details about RCC3, refer to its :doc:`description. <../handwritten/qsrs/rcc3>`
    """

    """
        Following elsewhere.
        # 'dc'     bb1 is disconnected from bb2
        # 'po'     bb1 partially overlaps bb2
        # 'o'      bb1 is occluded or part of bb2
    """

    _unique_id = "rcc3"
    """str: Unique identifier name of the QSR."""

    _all_possible_relations = ("dc", "po", "o")
    """tuple: All possible relations of the QSR."""

    def __init__(self):
        """Constructor."""
        super(QSR_RCC3_Rectangle_Bounding_Boxes_2D, self).__init__()

    def _convert_to_requested_rcc_type(self, qsr):
        """Remap QSR values from RCC8 to RCC3.

        :param qsr: RCC8 value.
        :return: RCC3 value.
        :rtype: str
        """
        qsr = qsr.replace("ec", "po")
        return qsr if qsr in self._all_possible_relations else self._all_possible_relations[-1]
