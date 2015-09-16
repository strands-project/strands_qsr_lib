# -*- coding: utf-8 -*-
from __future__ import print_function, division
import numpy as np
from qsrlib_qsrs.qsr_monadic_abstractclass import QSR_Monadic_2t_Abstractclass


class QSR_Moving_or_Stationary(QSR_Monadic_2t_Abstractclass):
    """Computes moving or stationary relations: 'm': moving, 's': stationary

    """

    _unique_id = "mos"
    _all_possible_relations = ("m", "s")
    _dtype = "points"

    def __init__(self):
        super(QSR_Moving_or_Stationary, self).__init__()
        self.__qsr_params_defaults = {"quantisation_factor": 0.0}

    def _process_qsr_parameters_from_request_parameters(self, req_params, **kwargs):
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
        """Return MOS relation

        :param data1: point
        :param data2: point
        :param qsr_params: "quantisation_factor": determines minimal displacement to be considered as a moving
        :return: a MoS relation from the following: 'm': moving, 'c': stationary
        """
        # print(data1.x, data1.y, data2.x, data2.y, np.sqrt((data1.x-data2.x)**2 + (data1.y-data2.y**2)))
        # print(np.sqrt((data1.x-data2.x)**2 + (data1.y-data2.y**2)))
        return "m" if np.sqrt((data1.x-data2.x)**2 + (data1.y-data2.y)**2) > qsr_params["quantisation_factor"] else "s"
