# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_1t_Abstractclass


class QSR_MWE(QSR_Dyadic_1t_Abstractclass):
    """Minimal Working Example (MWE) of making a new QSR.

    Values of the abstract properties
        * **_unique_id** = "mwe"
        * **_all_possible_relations** = ("left", "together", "right")
        * **_dtype** = "points"

    .. seealso:: For further details about MWE, refer to its :doc:`description. <../handwritten/qsrs/mwe>`
    """

    _unique_id = "mwe"
    """str: Unique identifier name of the QSR."""

    _all_possible_relations = ("left", "together", "right")
    """tuple: All possible relations of the QSR."""

    _dtype = "points"
    """str: On what kind of data the QSR works with."""

    def __init__(self):
        """Constructor."""
        super(QSR_MWE, self).__init__()

    def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
        """Compute QSR value.

        :param data1: First object data.
        :type data1: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param data2: Second object data.
        :type data2: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param qsr_params: QSR specific parameters passed in `dynamic_args`.
        :type qsr_params: dict
        :param kwargs: kwargs arguments.
        :return: Computed QSR value.
        :rtype: str
        """
        return {
            data1.x < data2.x: "left",
            data1.x > data2.x: "right"
        }.get(True, "together")
