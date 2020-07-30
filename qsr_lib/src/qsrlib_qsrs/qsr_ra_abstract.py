# -*- coding: utf-8 -*-
from __future__ import print_function, division
from abc import ABCMeta, abstractmethod
from numpy import isnan, abs
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_1t_Abstractclass


class QSR_RA_Abstract(QSR_Dyadic_1t_Abstractclass):
    """Rectangle Algebra abstract class.

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

    __metaclass__ = ABCMeta

    __inverse_map = {"<": ">", "m": "mi", "o": "oi", "s": "si", "d": "di", "f": "fi",
                     ">": "<", "mi": "m", "o1": "o", "si": "s", "di": "d", "fi": "f"}
    """dict: Inverse relations"""

    __qsr_params_defaults = {"qfactor": 0.0}
    """dict: Default values of the QSR parameters."""

    def __init__(self):
        """Constructor."""
        super(QSR_RA_Abstract, self).__init__()

    def _process_qsr_parameters_from_request_parameters(self, req_params, **kwargs):
        """Extract QSR specific parameters from the QSRlib request call parameters.

        :param req_params: QSRlib request call parameters.
        :type req_params: dict
        :param kwargs: kwargs arguments.
        :return: QSR specific parameter settings.
        :rtype: dict
        """
        qsr_params = self.__qsr_params_defaults.copy()
        try:
            qsr_params["qfactor"] = float(req_params["dynamic_args"][self._unique_id]["qfactor"])
            if not isinstance(qsr_params["qfactor"], (int, float)) or qsr_params["qfactor"] < 0:
                raise ValueError("qfactor must be positive numeric")
        except (KeyError, TypeError):
            pass
        return qsr_params

    @abstractmethod
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

    def _allen(self, i1, i2, qsr_params):
        if isnan(i1).any() or isnan(i2).any():  # nan values cause dragons
            raise ValueError("illegal 'nan' values found")

        if not qsr_params["qfactor"]:  # normal
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

        else:  # qfactor compensation, can produce strange and undesired results
            if i1[1]+qsr_params["qfactor"] < i2[0]:
                return "<"
            if abs(i1[1]-i2[0]) <= qsr_params["qfactor"]:
                return "m"
            if abs(i1[0]-i2[0]) <= qsr_params["qfactor"] and i1[1] < i2[1]:
                return "s"
            if i2[0] < i1[0] < i2[1] and abs(i1[1]-i2[1]) <= qsr_params["qfactor"]:
                return "f"
            if abs(i1[0]-i2[0]) <= qsr_params["qfactor"] and abs(i1[1]-i2[1]) <= qsr_params["qfactor"]:
                return "="
            if i1[0] < i2[0] < i1[1] and i2[0] < i1[1] < i2[1]:
                return "o"
            if i2[0] < i1[0] < i2[1] and i2[0] < i1[1] < i2[1]:
                return "d"

        return self.__inverse_map[self._allen(i2, i1, qsr_params)]
