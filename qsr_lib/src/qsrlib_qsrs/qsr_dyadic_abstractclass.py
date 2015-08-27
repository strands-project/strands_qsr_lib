# -*- coding: utf-8 -*-
from __future__ import print_function, division
from abc import ABCMeta, abstractmethod
from qsrlib_qsrs.qsr_abstractclass import QSR_Abstractclass
from qsrlib_utils.combinations_and_permutations import *
from qsrlib_io.world_qsr_trace import *

class QSR_Dyadic_Abstractclass(QSR_Abstractclass):
    __metaclass__ = ABCMeta

    def __init__(self):
        super(QSR_Dyadic_Abstractclass, self).__init__()
        self._allowed_dtype = {"points": self.__return_points,
                               "bounding_boxes": self.__return_bounding_boxes}

    def _init_qsrs_for_default(self, objects_names_of_world_state):
        return possible_pairs(objects_names_of_world_state)

    def _validate_qsrs_for(self, qsrs_for):
        """qsrs_for must be tuples of two objects.

        :param qsrs_for: list of strings and/or tuples for which QSRs will be computed
        :return: A list of objects names pairs to make QSRs for
        :rtype: list
        """
        return [p for p in qsrs_for if isinstance(p, (list, tuple)) and (len(p) == 2)]

    def __return_points(self, data1, data2):
        return data1, data2

    def __return_bounding_boxes(self, data1, data2):
        return data1.return_bounding_box_2d(), data2.return_bounding_box_2d()


class QSR_Dyadic_1t_Abstractclass(QSR_Dyadic_Abstractclass):
    __metaclass__ = ABCMeta

    def __init__(self):
        super(QSR_Dyadic_1t_Abstractclass, self).__init__()

    @abstractmethod
    def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
        return

    def _make_world_qsr_trace(self, world_trace, timestamps, qsr_params, req_params, dtype, **kwargs):
        """

        :param world_trace:
        :param timestamps:
        :param qsr_params:
        :param req_params:
        :param dtype: value from self.__allowed_dtype
        :param kwargs:
        :return:
        """
        ret = World_QSR_Trace(qsr_type=self._unique_id)
        for t in timestamps:
            world_state = world_trace.trace[t]
            qsrs_for = self._process_qsrs_for(world_state.objects.keys(), req_params["dynamic_args"])
            for p in qsrs_for:
                between = ",".join(p)
                try:
                    data1, data2 = self._allowed_dtype[dtype](world_state.objects[p[0]], world_state.objects[p[1]])
                except KeyError:
                    raise KeyError("%s is not a valid value, should be one of %s" % (dtype, self._allowed_dtype.keys()))
                ret.add_qsr(QSR(timestamp=t, between=between,
                                qsr=self._format_qsr(self._compute_qsr(data1, data2, qsr_params, **kwargs))),
                            t)
        return ret
