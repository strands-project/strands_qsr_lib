# -*- coding: utf-8 -*-
from __future__ import print_function, division
import itertools
from qsrlib_qsrs.qsr_ra_abstract import QSR_RA_Abstract


class QSR_RA(QSR_RA_Abstract):
    """Rectangle Algebra.

    Members:
        * **_unique_id** = "ra"
        * **_all_possible_relations** = quite long to list here (169) but they are all possible pairs between the 13 Allen's relations
        * **_dtype** = "bounding_boxes_2d"

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

    _unique_id = "ra"
    """str: Unique identifier name of the QSR."""

    _dtype = "bounding_boxes_2d"
    """str: On what kind of data the QSR works with."""

    _all_possible_relations = tuple(itertools.product(("<", ">", "m", "mi", "o", "oi", "s", "si", "d", "di", "f", "fi", "="),
                                                      repeat=2))
    """tuple: All possible relations of the QSR."""

    def __init__(self):
        """Constructor."""
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
        return ",".join([self._allen((bb1[0], bb1[2]), (bb2[0], bb2[2]), qsr_params),
                         self._allen((bb1[1], bb1[3]), (bb2[1], bb2[3]), qsr_params)])
