# -*- coding: utf-8 -*-
from __future__ import print_function, division
from numpy import isnan
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_1t_Abstractclass


class QSR_RA(QSR_Dyadic_1t_Abstractclass):
    """Rectangle Algebra.

    Members:
        * _unique_id: "ra"
        * _all_possible_relations: ("<", ">", "m", "mi", "o", "oi", "s", "si", "d", "di", "f", "fi", "=")
        * _dtype: "bounding_boxes"

    .. seealso:: For further details about RA, refer to its :doc:`description. <../handwritten/qsrs/ra>`
    """

    _unique_id = "ra"
    """str: Unique identifier name of the QSR."""

    _all_possible_relations = ("<", ">", "m", "mi", "o", "oi", "s", "si", "d", "di", "f", "fi", "=")
    """tuple: All possible relations of the QSR."""

    _dtype = "bounding_boxes"
    """str: On what kind of data the QSR works with."""

    _inverse_map = {"<": ">", "m": "mi", "o": "oi", "s": "si", "d": "di", "f": "fi",
                    ">": "<", "mi": "m", "o1": "o", "si": "s", "di": "d", "fi": "f"}
    """dict: Inverse relations"""

    def __init__(self):
        """Constructor.

        :return:
        """
        super(QSR_RA, self).__init__()

    def _compute_qsr(self, bb1, bb2, qsr_params, **kwargs):
        """Compute QSR value.

        :param bb1: First object's bounding box.
        :type bb2: tuple or list
        :param bb2: Second object's bounding box.
        :type bb2: tuple or list
        :param qsr_params: QSR specific parameters passed in `dynamic_args`.
        :type qsr_params: dict
        :param kwargs: Optional further arguments.
        :return: The computed QSR value: two/three comma separated Allen relations for 2D/3D.
        :rtype: str
        """
        if len(bb1) == 4 and len(bb2) == 4:  # 2D version
            return ",".join([self.__allen((bb1[0], bb1[2]), (bb2[0], bb2[2])),
                             self.__allen((bb1[1], bb1[3]), (bb2[1], bb2[3]))])
        elif len(bb1) == 6 and len(bb2) == 6:  # 3D version
            return ",".join([self.__allen((bb1[0], bb1[3]), (bb2[0], bb2[3])),
                             self.__allen((bb1[1], bb1[4]), (bb2[1], bb2[4])),
                             self.__allen((bb1[2], bb1[5]), (bb2[2], bb2[5]))])
        else:
            raise ValueError("bb1 and bb2 must have length of 4 (2D) or 6 (3D)")

    def __allen(self, i1, i2):
        if isnan(i1).any() or isnan(i2).any():  # nan values cause dragons
            raise ValueError("illegal 'nan' values found")

        if i1[1] < i2[0]:
            return "<"
        if i1[1] == i2[0]:
            return "m"
        if i1[0] < i2[0] < i1[1] and i2[0] < i1[1] < i2[1]:
            return "o"
        if i1[0] == i2[0] and i1[1] < i2[1]:
            return "s"
        if i2[0] < i1[0] < i2[1] and i2[0] < i1[1] < i2[1]:
            return "d"
        if i2[0] < i1[0] < i2[1] and i1[1] == i2[1]:
            return "f"
        if i1[0] == i2[0] and i1[1] == i2[1]:
            return "="
        return self._inverse_map[self.__allen(i2, i1)]
