# -*- coding: utf-8 -*-
from __future__ import print_function, division
import numpy as np
from qsrlib_qsrs.qsr_monadic_abstractclass import QSR_Monadic_2t_Abstractclass


class QSR_Moving_or_Stationary(QSR_Monadic_2t_Abstractclass):
    """Computes moving or stationary relations.

    Values of the abstract properties
        * **_unique_id** = "mos"
        * **_all_possible_relations** = ("m", "s")
        * **_dtype** = "points"

    QSR specific `dynamic_args`
        * **'quantisation_factor'** (*float*) = 0.0: Threshold that determines minimal Euclidean distance to be considered as moving.

    Some explanation about the QSR or better link to a separate webpage explaining it. Maybe a reference if it exists.
    """

    _unique_id = "mos"
    """str: Unique identifier name of the QSR."""

    _all_possible_relations = ("m", "s")
    """tuple: All possible relations of the QSR."""

    _dtype = "points"
    """str: On what kind of data the QSR works with."""

    def __init__(self):
        """Constructor."""
        super(QSR_Moving_or_Stationary, self).__init__()

        self.__qsr_params_defaults = {"quantisation_factor": 0.0}
        """?"""

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
            qsr_params["quantisation_factor"] = float(req_params["dynamic_args"][self._unique_id]["quantisation_factor"])
        except (KeyError, TypeError):
            try:
                qsr_params["quantisation_factor"] = float(req_params["dynamic_args"]["for_all_qsrs"]["quantisation_factor"])
            except (TypeError, KeyError):
                pass
        return qsr_params

    def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
        """Return QSR value.

        :param data1: Object data at first timestamp.
        :type data1: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param data2: Object data at second timestamp.
        :type data2: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param qsr_params: QSR specific parameters passed in `dynamic_args`.
        :type qsr_params: dict
        :param kwargs: kwargs arguments
        :return: Computed QSR value.
        :rtype: str
        """
        # print(data1.x, data1.y, data2.x, data2.y, np.sqrt((data1.x-data2.x)**2 + (data1.y-data2.y**2)))
        # print(np.sqrt((data1.x-data2.x)**2 + (data1.y-data2.y**2)))
        return "m" if np.sqrt((data1.x-data2.x)**2 + (data1.y-data2.y)**2) > qsr_params["quantisation_factor"] else "s"
