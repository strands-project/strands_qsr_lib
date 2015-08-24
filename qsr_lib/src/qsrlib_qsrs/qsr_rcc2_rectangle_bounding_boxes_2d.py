# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_Abstractclass
from qsrlib_io.world_qsr_trace import *


class QSR_RCC2_Rectangle_Bounding_Boxes_2D(QSR_Dyadic_Abstractclass):
    """Computes RCC2 relations: 'dc': disconnected, 'c': connected

    """
    def __init__(self):
        super(QSR_RCC2_Rectangle_Bounding_Boxes_2D, self).__init__()
        self._unique_id = "rcc2"
        self.all_possible_relations = ["dc", "c"]

        self.__qsr_params_defaults = {"quantisation_factor": 0.0}

    def custom_set_from_config_file(self, document):
        pass

    def custom_help(self):
        """Write your own help message function"""
        print("where,\nx1, y2: the xy-coords of the top-left corner of the rectangle\nx2, y2: the xy-coords of the bottom-right corner of the rectangle")

    #todo possibly no longer needed
    def custom_checks(self, input_data):
        """Write your own custom checks on top of the default ones


        :return: error code, error message (integer, string), use 10 and above for error code as 1-9 are reserved by system
        """
        return 0, ""

    def _process_qsr_parameters_from_request_parameters(self, req_params, **kwargs):
        qsr_params = self.__qsr_params_defaults.copy()
        try:
            qsr_params["quantisation_factor"] = float(req_params["dynamic_args"][self._unique_id]["quantisation_factor"])
        except KeyError:
            try:
                qsr_params["quantisation_factor"] = float(req_params["dynamic_args"]["for_all_qsrs"]["quantisation_factor"])
            except TypeError:
                pass
        except TypeError:
            pass
        return qsr_params

    def _postprocess_world_qsr_trace(self, world_qsr_trace, world_trace, world_trace_timestamps, req_params, qsr_params):
        return world_qsr_trace

    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, **kwargs):
        ret = World_QSR_Trace(qsr_type=self._unique_id)
        for t in timestamps:
            world_state = world_trace.trace[t]
            qsrs_for = self._process_qsrs_for(world_state.objects.keys(), kwargs)
            for p in qsrs_for:
                between = ",".join(p)
                bb1 = world_state.objects[p[0]].return_bounding_box_2d()
                bb2 = world_state.objects[p[1]].return_bounding_box_2d()
                ret.add_qsr(QSR(timestamp=t, between=between,
                                qsr=self.format_qsr(self.__compute_qsr(bb1, bb2, qsr_params["quantisation_factor"]))),
                            t)
        return ret

    def __compute_qsr(self, bb1, bb2, q=0.0):
        """Return RCC2 relation

        :param bb1: diagonal points coordinates of first bounding box (x1, y1, x2, y2)
        :param bb2: diagonal points coordinates of second bounding box (x1, y1, x2, y2)
        :return: an RCC2 relation from the following: 'dc':disconnected, 'c': connected
        """
        return "dc" if (bb1[0] > bb2[2]+q) or (bb1[2] < bb2[0]-q) or (bb1[1] > bb2[3]+q) or (bb1[3] < bb2[1]-q) else "c"
