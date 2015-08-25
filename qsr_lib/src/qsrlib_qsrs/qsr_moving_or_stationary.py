# -*- coding: utf-8 -*-
"""Computes MoS relations: 'm': moving, 's': stationary

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 10 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""

from __future__ import print_function, division
import numpy as np
from qsrlib_qsrs.qsr_monadic_abstractclass import QSR_Monadic_Abstractclass
from qsrlib_io.world_qsr_trace import *


class QSR_Moving_or_Stationary(QSR_Monadic_Abstractclass):
    """Computes moving or stationary relations: 'm': moving, 's': stationary

    """
    def __init__(self):
        super(QSR_Moving_or_Stationary, self).__init__()
        self._unique_id = "mos"
        self.all_possible_relations = ["m", "s"]

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

    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, **kwargs):
        ret = World_QSR_Trace(qsr_type=self._unique_id)
        for t, tp in zip(timestamps[1:], timestamps):
            world_state_now = world_trace.trace[t]
            world_state_previous = world_trace.trace[tp]
            qsrs_for = self._process_qsrs_for(world_state_now.objects.keys(), kwargs)
            for object_name in qsrs_for: # yes between is probably not the best name as it is simply object_name
                point_now = (world_state_now.objects[object_name].x, world_state_now.objects[object_name].y)
                point_previous = (world_state_previous.objects[object_name].x, world_state_previous.objects[object_name].y)
                ret.add_qsr(QSR(timestamp=t, between=object_name,
                                qsr=self._format_qsr(self.__compute_qsr(point_now, point_previous,
                                                                       qsr_params["quantisation_factor"]))),
                            t)
        return ret

    def __compute_qsr(self, p1, p2, quantisation_factor=0.0):
        """Return MOS relation

        :param p1: point (x1, y1)
        :param p2: point (x2, y2)
        :moving_thres: determines minimal displacement to be considered as a moving
        :return: a MoS relation from the following: 'm': moving, 'c': stationary
        """
        return "m" if np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2) > quantisation_factor else "s"
