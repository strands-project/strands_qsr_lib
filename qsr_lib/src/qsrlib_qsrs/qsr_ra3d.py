# -*- coding: utf-8 -*-
from __future__ import print_function, division
import itertools
from qsr_ra_abstract import QSR_RA_Abstract


class QSR_RA3D(QSR_RA_Abstract):
    """Rectangle Algebra.

    Members:
        * **_unique_id** = "ra3d"
        * **_all_possible_relations** = quite long to list here (2197) but they are all possible triplets between the 13 Allen's relations
        * **_dtype** = "bounding_boxes_3d"

    QSR specific `dynamic_args`
        * **'qfactor'** (*int or float*) = 0.0: This factor provides some tolerance on the edges that are difficult
          when the variables are floats, e.g. in the 'meets' relation it is unlikely that the end of one
          segment will meet the beginning of the other to the decimal value.

    .. warning::
        Use of 'qfactor' might have strange and undesired results. Use it at your own risk.

        For further details, you might want to consult with the exact implementation of the method
        `_allen`_ in class `QSR_RA`.

    .. seealso:: For further details about RA, refer to its :doc:`description. <../handwritten/qsrs/ra>`

    .. _`_allen`: https://github.com/strands-project/strands_qsr_lib/blob/master/qsr_lib/src/qsrlib_qsrs/qsr_ra.py
    """

    _unique_id = "ra3d"
    """str: Unique identifier name of the QSR."""

    _dtype = "bounding_boxes_3d"
    """str: On what kind of data the QSR works with."""

    _all_possible_relations = tuple(itertools.product(("<", ">", "m", "mi", "o", "oi", "s", "si", "d", "di", "f", "fi", "="),
                                                      repeat=3))
    """tuple: All possible relations of the QSR."""

    def __init__(self):
        """Constructor."""
        super(QSR_RA3D, self).__init__()

    def _compute_qsr(self, bb1, bb2, qsr_params, **kwargs):
        """Compute QSR value.

        :param bb1: First object's 3D bounding box.
        :type bb2: tuple or list
        :param bb2: Second object's 3D bounding box.
        :type bb2: tuple or list
        :param qsr_params: QSR specific parameters passed in `dynamic_args`.
        :type qsr_params: dict
        :param kwargs: Optional further arguments.
        :return: The computed QSR value: two/three comma separated Allen relations for 2D/3D.
        :rtype: str
        """
        return ",".join([self._allen((bb1[0], bb1[3]), (bb2[0], bb2[3])),
                         self._allen((bb1[1], bb1[4]), (bb2[1], bb2[4])),
                         self._allen((bb1[2], bb1[5]), (bb2[2], bb2[5]))])
