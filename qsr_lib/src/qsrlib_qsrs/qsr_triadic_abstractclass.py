# -*- coding: utf-8 -*-
from __future__ import print_function, division
from abc import ABCMeta, abstractmethod
from qsrlib_qsrs.qsr_abstractclass import QSR_Abstractclass
from qsrlib_utils.combinations_and_permutations import *
from qsrlib_io.world_qsr_trace import *

class QSR_Triadic_Abstractclass(QSR_Abstractclass):
    """Abstract class of triadic QSRs, i.e. QSRs that are computed over three objects."""

    __metaclass__ = ABCMeta

    def __init__(self):
        """Constructor."""
        super(QSR_Triadic_Abstractclass, self).__init__()

    def _init_qsrs_for_default(self, objects_names_of_world_state):
        """Default list of entities for which QSRs are to be computed for.

        :param objects_names_of_world_state: Objects names at a world state.
        :type objects_names_of_world_state: list of str
        :return: The permutations, i.e. all possible pairs including mirrors, of the list of names passed in the
        arguments. E.g. for `objects_names_of_world_state = ['a', 'b']` return `[('a', 'b'), ('b', 'a')]`.
        :rtype: list of tuples of str
        """
        return possible_triplets(objects_names_of_world_state)

    def _validate_qsrs_for(self, qsrs_for):
        """Validate `qsrs_for` which must be a list of tuples of three objects names.

        :param qsrs_for: The original `qsrs_for` that needs validation.
        :type qsrs_for: list
        :return: List of string objects names to make QSRs, which might be the same as the argument `qsrs_for` or a
        subset of it with elements that passed the validation test, i.e. the elements of the list must be tuples of
        three strings.
        :rtype: list
        """
        return [p for p in qsrs_for if isinstance(p, (list, tuple)) and (len(p) == 3)]

    def _return_points(self, data1, data2, data3):
        """Return the arguments as they are in their point form.

        :param data1: First object data.
        :type data1: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param data2: Second object data.
        :type data2: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param data3: Third object data.
        :type data3: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :return: `data1`, `data2`, `data3`
        :rtype: three :class:`Object_State <qsrlib_io.world_trace.Object_State>` objects
        """
        return data1, data2, data3

    def _return_bounding_boxes_2d(self, data1, data2, data3):
        """Return the 2D bounding boxes of the arguments.

        :param data1: First object data.
        :type data1: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param data2: Second object data.
        :type data2: :class:`Object_State <qsrlib_io.world_trace.Object_State>` 
        :param data3: Third object data.
        :type data3: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :return: `bbox1`, `bbox2`, `bbox3`
        :rtype: three lists of floats
        """
        return data1.return_bounding_box_2d(), data2.return_bounding_box_2d(), data3.return_bounding_box_2d()


class QSR_Triadic_1t_Abstractclass(QSR_Triadic_Abstractclass):
    """Special case abstract class of triadic QSRs. Works with triadic QSRs that require data over one timestamp."""

    __metaclass__ = ABCMeta

    def __init__(self):
        """Constructor."""
        super(QSR_Triadic_1t_Abstractclass, self).__init__()

    @abstractmethod
    def _compute_qsr(self, data1, data2, data3, qsr_params, **kwargs):
        """Compute QSR value.

        :param data1: First object data.
        :type data1: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param data2: Second object data.
        :type data2: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param data3: Third object data.
        :type data3: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
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
        for t in timestamps:
            world_state = world_trace.trace[t]
            qsrs_for = self._process_qsrs_for(world_state.objects.keys(), req_params["dynamic_args"])
            for p in qsrs_for:
                between = ",".join(p)
                try:
                    data1, data2, data3 = self._dtype_map[self._dtype](world_state.objects[p[0]], world_state.objects[p[1]], world_state.objects[p[2]])
                except KeyError:
                    raise KeyError("%s is not a valid value, should be one of %s" % (self._dtype, self._dtype_map.keys()))
                ret.add_qsr(QSR(timestamp=t, between=between,
                                qsr=self._format_qsr(self._compute_qsr(data1, data2, data3, qsr_params, **kwargs))),
                            t)
        return ret
