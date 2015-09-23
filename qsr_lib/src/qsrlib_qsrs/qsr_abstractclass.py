# -*- coding: utf-8 -*-

from __future__ import print_function, division
from abc import ABCMeta, abstractmethod, abstractproperty


class QSR_Abstractclass(object):
    """Root abstract class of the QSR implementators.

    Abstract properties
        * **_unique_id** (*str*): Unique identifier of a QSR.
        * **_all_possible_relations** (*tuple*): All possible relations of a QSR.
        * **_dtype** (*str*): Kind of data the QSR operates with, see self._dtype_map for possible values.

    Members
        * **_dtype_map** (*dict*): Mapping of _dtype to methods. It contains:

            * "points": self._return_points
            * "bounding_boxes": self._return_bounding_boxes_2d
            * "bounding_boxes_2d": self._return_bounding_boxes_2d

    """

    __metaclass__ = ABCMeta

    _common_dynamic_args = ["qsrs_for"]
    """tuple: Common across all QSRs arguments of `dynamic_args`."""

    def __init__(self):
        """Constructor."""
        self._dtype_map = {"points": self._return_points,
                           "bounding_boxes": self._return_bounding_boxes_2d,  # todo this is to handle 2D/3D and needs its own function
                           "bounding_boxes_2d": self._return_bounding_boxes_2d}
        """dict: Mapping of _dtype to methods."""

    @abstractproperty
    def _unique_id(self):
        """str: Unique identifier of a QSR."""
        pass

    @abstractproperty
    def _all_possible_relations(self):
        """tuple: All possible relations of a QSR."""
        pass

    @abstractproperty
    def _dtype(self):
        """str: Kind of data the QSR operates with, see self._dtype_map for possible values."""
        pass

    @abstractmethod
    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, req_params, **kwargs):
        """The main function that generates the world QSR trace.

        * QSR classes inheriting from the general purpose meta-abstract classes \
        (e.g. :class:`QSR_Monadic_Abstractclass <qsrlib_qsrs.qsr_monadic_abstractclass.QSR_Monadic_Abstractclass>`, \
        :class:`QSR_Dyadic_Abstractclass <qsrlib_qsrs.qsr_dyadic_abstractclass.QSR_Dyadic_Abstractclass>` , etc.) \
        need to provide this function.
        * When inheriting from one of the special case meta-abstract classes \
        (e.g. :class:`QSR_Monadic_2t_Abstractclass <qsrlib_qsrs.qsr_monadic_abstractclass.QSR_Monadic_2t_Abstractclass>`, \
        :class:`QSR_Dyadic_1t_Abstractclass <qsrlib_qsrs.qsr_dyadic_abstractclass.QSR_Dyadic_1t_Abstractclass>`, etc.) \
        then usually there is no need to do so; check with the documentation of these special cases to see if they \
        already implement one.

        :param world_trace: Input data.
        :type world_trace: :class:`World_Trace <qsrlib_io.world_trace.World_Trace>`
        :param timestamps: List of sorted timestamps of `world_trace`.
        :type timestamps: list
        :param qsr_params: QSR specific parameters passed in `dynamic_args`.
        :type qsr_params: dict
        :param dynamic_args: Dynamic arguments passed with the request.
        :type dynamic_args: dict
        :param kwargs: kwargs arguments.
        :return: Computed world QSR trace.
        :rtype: :class:`World_QSR_Trace <qsrlib_io.world_qsr_trace.World_QSR_Trace>`
        """
        return

    @abstractmethod
    def _init_qsrs_for_default(self, objects_names_of_world_state, **kwargs):
        """The default list of entities for which QSRs are to be computed for, given typically the list of objects names
        that exist at a world state.

        Usually this is provided by a parent abstract class and there is no need for the QSRs to implement it,
        unless they want to override the parent's default method.

        :param objects_names_of_world_state: The objects names at a world state.
        :type objects_names_of_world_state: list
        :param kwargs: Optional extra arguments.
        :return: The list of entities for which QSRs will be computed for at that world state for which the objects names
        were given.
        :rtype: list
        """
        return

    @abstractmethod
    def _validate_qsrs_for(self, qsrs_for):
        """Custom checks of the qsrs_for field.

        Usually this is provided by a parent abstract class and there is no need for the QSRs to implement it,
        unless they want to override the parent's default method.

        :param qsrs_for: List of entities to be validated and processed.
        :type qsrs_for: list
        :return: Validated list.
        :rtype: list
        """
        return qsrs_for

    @abstractmethod
    def _return_points(self):
        """Return the arguments as they are in their point form.

        :return: The arguments as they are in their point form.
        """
        return

    @abstractmethod
    def _return_bounding_boxes_2d(self):
        """Return the 2D bounding boxes of the arguments.

        :return: 2D bounding boxes of the arguments.
        """
        return

    @property
    def unique_id(self):
        """Getter for the unique identifier of a QSR.

        :return: `self._unique_id`
        :rtype: str
        """
        return self._unique_id

    @property
    def all_possible_relations(self):
        """Getter for all the possible relations of a QSR.

        :return: `self._all_possible_relations`
        :rtype: tuple
        """
        return self._all_possible_relations

    def get_qsrs(self, **req_params):
        """Compute the QSRs.

        This method is called from QSRlib so no need to call it directly from anywhere else.

        :param req_params: Request parameters.
        :type req_params: dict
        :return: Computed world qsr trace.
        :rtype: :class:`World_QSR_Trace <qsrlib_io.world_qsr_trace.World_QSR_Trace>`
        """
        qsr_params = self._process_qsr_parameters_from_request_parameters(req_params)
        world_trace, timestamps = self._set_input_world_trace(req_params["input_data"], qsr_params)
        world_qsr_trace = self.make_world_qsr_trace(world_trace, timestamps, qsr_params, req_params)
        world_qsr_trace = self._postprocess_world_qsr_trace(world_qsr_trace, world_trace, timestamps, qsr_params, req_params)
        return world_qsr_trace

    def _custom_checks_world_trace(self, world_trace, qsr_params):
        """Customs checks of the input data.

        If a QSR needs to overwrite this then it should raise appropriate exceptions.

        :param world_trace: The input data.
        :type world_trace: :class:`World_Trace <qsrlib_io.world_trace.World_Trace>`
        :return: False for no problems.
        :rtype: bool
        :raises: Depends on the QSR.
        """
        return False

    def _set_input_world_trace(self, world_trace, qsr_params):
        """Check the input data and return both the input data and the sorted timestamps it.

        :param world_trace: The input data.
        :type world_trace: :class:`World_Trace <qsrlib_io.world_trace.World_Trace>`
        :param qsr_params: QSR specific parameters.
        :type qsr_params: dict
        :return: The input data and its sorted timestamps if all well.
        :rtype: (:class:`World_Trace <qsrlib_io.world_trace.World_Trace>`, list)
        :raises: Depends on `self._custom_checks_world_trace` method.
        """
        self._custom_checks_world_trace(world_trace, qsr_params)
        return world_trace, world_trace.get_sorted_timestamps()

    def _process_qsrs_for(self, objects_names_of_world_state, dynamic_args, **kwargs):
        """Parse `dynamic_args` and generate valid `qsrs_for`.

        It parses the `dynamic_args` to see if the user has specified `qsrs_for` and then validates them, or uses
        default `qsrs_for` generation if user has not specified anything.

        :param objects_names_of_world_state: The object names in a world state.
        :param dynamic_args: The dynamic arguments passed with the request.
        :type dynamic_args: dict
        :param kwargs: Optional extra arguments.
        :return: Valid `qsrs_for`.
        :rtype: list
        :raises: TypeError: When no valid `qsrs_for` can be generated.
        """
        if isinstance(objects_names_of_world_state[0], str):
            try:
                return self.__check_qsrs_for_data_exist_at_world_state(objects_names_of_world_state,
                                                                       dynamic_args[self._unique_id]["qsrs_for"])
            except KeyError:
                try:
                    return self.__check_qsrs_for_data_exist_at_world_state(objects_names_of_world_state,
                                                                           dynamic_args["for_all_qsrs"]["qsrs_for"])
                except KeyError:
                    return self._init_qsrs_for_default(objects_names_of_world_state)
        elif isinstance(objects_names_of_world_state[0], (list, tuple)):
            qsrs_for_list = []
            for objects_names_i in objects_names_of_world_state:
                try:
                    qsrs_for_list.append(self.__check_qsrs_for_data_exist_at_world_state(objects_names_i,
                                                                                         dynamic_args[self._unique_id]["qsrs_for"]))
                except KeyError:
                    try:
                        qsrs_for_list.append(self.__check_qsrs_for_data_exist_at_world_state(objects_names_i,
                                                                                             dynamic_args["for_all_qsrs"]["qsrs_for"]))
                    except KeyError:
                        qsrs_for_list.append(self._init_qsrs_for_default(objects_names_i))

            return list(set(qsrs_for_list[0]).intersection(*qsrs_for_list))
        else:
            raise TypeError("the objects names must be a list of str or list of lists")

    def __check_qsrs_for_data_exist_at_world_state(self, objects_names_of_world_state, qsrs_for):
        """Check that the entities in `qsrs_for` exist in the world state.

        :param objects_names_of_world_state: The objects names that exist in a world state.
        :type objects_names_of_world_state: list or tuple
        :param qsrs_for:
        :return: A list of entities that exist in the world state and QSRs are to be computed for. Entities with objects
        that did not exist will have been removed.
        :rtype: list
        """
        if len(objects_names_of_world_state) == 0:
            return []
        if not isinstance(qsrs_for, (list, tuple)):
            raise TypeError("qsrs_for must be list or tuple")
        qsrs_for_ret = []
        for p in qsrs_for:
            if isinstance(p, str):
                if p in objects_names_of_world_state:
                    qsrs_for_ret.append(p)
            elif isinstance(p, (list, tuple)):
                tuple_data_exists = True
                for o in p:
                    if o not in objects_names_of_world_state:
                        tuple_data_exists = False
                        break
                if tuple_data_exists:
                    qsrs_for_ret.append(p)
            else:
                raise TypeError("Elements of qsrs_for must be strings and/or tuples")
        qsrs_for_ret = self._validate_qsrs_for(qsrs_for_ret)
        return qsrs_for_ret

    def _process_qsr_parameters_from_request_parameters(self, req_params, **kwargs):
        """Get the QSR specific parameters from the request parameters.

        Overwrite as needed.

        :param req_params: Request parameters.
        :type req_params: dict
        :param kwargs: kwargs arguments.
        :return: QSR specific parameters.
        :rtype: dict
        """
        return {}

    def _postprocess_world_qsr_trace(self, world_qsr_trace, world_trace, world_trace_timestamps, qsr_params, req_params, **kwargs):
        """Post-process the computed world QSR trace.

        Overwrite as needed.

        :param world_qsr_trace: The world QSR trace.
        :type world_qsr_trace: qsrlib_io.world_qsr_trace.World_QSR_Trace
        :param world_trace: The input data world trace.
        :type world_trace: qsrlib_io.world_trace.World_Trace
        :param world_trace_timestamps: The sorted timestamps of the world trace.
        :type world_trace_timestamps: list
        :param qsr_params: The QSR specific parameters.
        :type qsr_params: dict
        :return: Post-processed world QSR trace.
        :rtype: qsrlib_io.world_qsr_trace.World_QSR_Trace
        """
        return world_qsr_trace

    def _format_qsr(self, v):
        """Format the value of the QSR to the QSRlib standard format storage.

        :param v: QSR value.
        :type v: str
        :return: The format
        :rtype: dict
        """
        return {self._unique_id: v}
