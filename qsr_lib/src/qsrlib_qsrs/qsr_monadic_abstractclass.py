# -*- coding: utf-8 -*-
from __future__ import print_function, division
from abc import ABCMeta, abstractmethod
from qsrlib_qsrs.qsr_abstractclass import QSR_Abstractclass
from qsrlib_io.world_qsr_trace import *

class QSR_Monadic_Abstractclass(QSR_Abstractclass):
    __metaclass__ = ABCMeta

    def __init__(self):
        super(QSR_Monadic_Abstractclass, self).__init__()
        self._allowed_dtype = {"points": self.__return_points,
                               "bounding_boxes": self.__return_bounding_boxes}

    def _init_qsrs_for_default(self, objects_names_of_world_state):
        return objects_names_of_world_state

    def _validate_qsrs_for(self, qsrs_for):
        """qsrs_for must be list of strings.

        :param qsrs_for:
        :type qsrs_for: list or tuple
        :return: A list of string objects names to make QSRs for
        :rtype: list
        """
        return [p for p in qsrs_for if isinstance(p, str)]

    def __return_points(self, data1, data2):
        return data1, data2

    def __return_bounding_boxes(self, data1, data2):
        raise data1.return_bounding_box_2d(), data2.return_bounding_box_2d()

class QSR_Monadic_2t_Abstractclass(QSR_Monadic_Abstractclass):
    __metaclass__ = ABCMeta

    def __init__(self):
        super(QSR_Monadic_2t_Abstractclass, self).__init__()

    @abstractmethod
    def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
        return

    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, req_params, **kwargs):
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
        for t, tp in zip(timestamps[1:], timestamps):
            world_state_now = world_trace.trace[t]
            world_state_previous = world_trace.trace[tp]
            qsrs_for = self._process_qsrs_for([world_state_previous.objects.keys(), world_state_now.objects.keys()],
                                              req_params["dynamic_args"])
            for object_name in qsrs_for:
                try:
                    data1, data2 = self._allowed_dtype[self._dtype](world_state_now.objects[object_name],
                                                              world_state_previous.objects[object_name])
                except KeyError:
                    raise KeyError("%s is not a valid value, should be one of %s" % (self._dtype, self._allowed_dtype.keys()))
                ret.add_qsr(QSR(timestamp=t, between=object_name,
                                qsr=self._format_qsr(self._compute_qsr(data1, data2, qsr_params, **kwargs))),
                            t)
        return ret
