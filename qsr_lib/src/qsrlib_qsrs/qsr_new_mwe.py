# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_Abstractclass
from qsrlib_io.world_qsr_trace import *


class QSR_MWE(QSR_Dyadic_Abstractclass):
    def __init__(self):
        super(QSR_MWE, self).__init__()
        self._unique_id = "mwe"
        self.all_possible_relations = ["left", "together", "right"]

    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, req_params, **kwargs):
        ret = World_QSR_Trace(qsr_type=self._unique_id)
        # this is what you need to implement
        for t in timestamps:
            world_state = world_trace.trace[t]
            qsrs_for = self._process_qsrs_for(world_state.objects.keys(), req_params["dynamic_args"])
            for p in qsrs_for:
                ret.add_qsr(QSR(timestamp=t, between=",".join(p),
                                qsr=self._format_qsr(self.__compute_qsr(world_state.objects[p[0]],
                                                                        world_state.objects[p[1]]))),
                            t)
        # end of what you need to implement
        return ret

    def __compute_qsr(self, p1, p2):
        if p1.x < p2.x:
            return "left"
        elif p1.x > p2.x:
            return "right"
        else:
            return "together"
