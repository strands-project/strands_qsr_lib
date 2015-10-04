# -*- coding: utf-8 -*-
from __future__ import print_function, division
from abc import ABCMeta, abstractmethod
from qsrlib_qsrs.qsr_abstractclass import QSR_Abstractclass
from qsrlib_io.world_qsr_trace import *


class QSR_Monadic_Abstractclass(QSR_Abstractclass):
    """Abstract class of monadic QSRs, i.e. QSRs that are computed over a single object."""

    __metaclass__ = ABCMeta

    def __init__(self):
        """Constructor."""
        super(QSR_Monadic_Abstractclass, self).__init__()

    def _init_qsrs_for_default(self, objects_names_of_world_state):
        """Default list of entities for which QSRs are to be computed for.

        :param objects_names_of_world_state: Objects names at a world state.
        :type objects_names_of_world_state: list
        :return: The object names in the world state, i.e. return the argument as passed.
        :rtype: list of str
        """
        return objects_names_of_world_state

    def _validate_qsrs_for(self, qsrs_for):
        """Validate `qsrs_for` which must be a list of strings.

        :param qsrs_for: The original `qsrs_for` that needs validation.
        :type qsrs_for: list
        :return: A list of string objects names to make QSRs, which might be the same as the argument `qsrs_for` or a
        subset of it with elements that passed the validation test, i.e. the elements of the list must be strings.
        :rtype: list
        """
        return [p for p in qsrs_for if isinstance(p, str)]

    def _return_points(self, data1, data2):
        """Return the arguments as they are in their point form.

        :param data1: Object data at first timestamp.
        :type data1: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param data2: Object data at second timestamp.
        :type data2: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :return: `data1`, `data2`
        :rtype: two :class:`Object_State <qsrlib_io.world_trace.Object_State>` objects
        """
        return data1, data2

    def _return_bounding_boxes_2d(self, data1, data2):
        """Return the 2D bounding boxes of the arguments.

        :param data1: Object data at first timestamp.
        :type data1: qsrlib_io.world_trace.Object_State
        :param data2: Object data at second timestamp.
        :type data2: qsrlib_io.world_trace.Object_State
        :return: `bbox1`, `bbox2`
        :rtype: two lists of floats
        """
        raise data1.return_bounding_box_2d(), data2.return_bounding_box_2d()


class QSR_Monadic_2t_Abstractclass(QSR_Monadic_Abstractclass):
    """Special case abstract class of monadic QSRs. Works with monadic QSRs that require data over two timestamps."""

    __metaclass__ = ABCMeta

    def __init__(self):
        """Constructor."""
        super(QSR_Monadic_2t_Abstractclass, self).__init__()

    @abstractmethod
    def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
        """Compute QSR value.

        :param data1: Object data at first timestamp.
        :type data1: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param data2: Object data at second timestamp.
        :type data2: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param qsr_params: QSR specific parameters passed in `dynamic_args`.
        :type qsr_params: dict
        :param kwargs: kwargs arguments.
        :return: Computed QSR value.
        :rtype: str
        """
        return

    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, req_params, **kwargs):
        """Compute the world QSR trace from the arguments.

        :param world_trace: Input data.
        :type world_trace: :class:`World_Trace <qsrlib_io.world_trace.World_Trace>`
        :param timestamps: List of sorted timestamps of `world_trace`.
        :type timestamps: list
        :param qsr_params: QSR specific parameters passed in `dynamic_args`.
        :type qsr_params: dict
        :param req_params: Request parameters.
        :type req_params: dict
        :param kwargs: kwargs arguments.
        :return: Computed world QSR trace.
        :rtype: :class:`World_QSR_Trace <qsrlib_io.world_qsr_trace.World_QSR_Trace>`
        """
        ret = World_QSR_Trace(qsr_type=self._unique_id)
        for t, tp in zip(timestamps[1:], timestamps):
            world_state_now = world_trace.trace[t]
            world_state_previous = world_trace.trace[tp]
            qsrs_for = self._process_qsrs_for([world_state_previous.objects.keys(), world_state_now.objects.keys()],
                                              req_params["dynamic_args"])
            for object_name in qsrs_for:
                try:
                    data1, data2 = self._dtype_map[self._dtype](world_state_now.objects[object_name],
                                                                world_state_previous.objects[object_name])
                except KeyError:
                    raise KeyError("%s is not a valid value, should be one of %s" % (self._dtype, self._dtype_map.keys()))
                ret.add_qsr(QSR(timestamp=t, between=object_name,
                                qsr=self._format_qsr(self._compute_qsr(data1, data2, qsr_params, **kwargs))),
                            t)
        return ret
