# -*- coding: utf-8 -*-
from __future__ import print_function, division
from abc import ABCMeta, abstractmethod
import yaml
import os


class QSR_Abstractclass(object):
    """Abstract class for the QSR makers"""
    __metaclass__ = ABCMeta

    def __init__(self):
        self._unique_id = ""  # must be the same that goes in the QSRlib.__qsrs_registration
        self._all_possible_relations = []
        self._allowed_parameters = ["qsrs_for"]
        self._dtype = ""

    @abstractmethod
    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, req_params, **kwargs):
        """The main function that makes the returned World_QSR_Trace and each QSR has to implement.

        :param world_trace:
        :param timestamps:
        :param qsr_params:
        :param dynamic_args:
        :param kwargs:
        :return:
        """
        return

    @abstractmethod
    def _init_qsrs_for_default(self, objects_names_of_world_state, **kwargs):
        """The default list of entities at each time-step (i.e. World_State.objects.keys() for which QSRs are computed for.

        Usually this is provided by a parent class and there is no need for the QSRs to implement it, unless they want
        to override the parent default method.

        :param objects_names_of_world_state:
        :param req_params:
        :param kwargs:
        :return:
        """
        return

    @abstractmethod
    def _validate_qsrs_for(self, qsrs_for):
        """Custom checks of the qsrs_for field.

        Usually this is provided by a parent class and there is no need for the QSRs to implement it, unless they want
        to override the parent default method.

        :param qsrs_for: list of strings and/or tuples for which QSRs will be computed
        :return: qsrs_for
        """
        return qsrs_for

    def get_unique_id(self):
        return self._unique_id

    def get_all_possible_relations(self):
        return self._all_possible_relations

    def get_qsrs(self, **req_params):
        """

        :param req_params: The request args
        :return: Computed World_QSR_Trace
        :rtype: World_QSR_Trace
        """
        world_trace, timestamps = self._set_input_world_trace(req_params["input_data"])
        qsr_params = self._process_qsr_parameters_from_request_parameters(req_params)
        world_qsr_trace = self.make_world_qsr_trace(world_trace, timestamps, qsr_params, req_params)
        world_qsr_trace = self._postprocess_world_qsr_trace(world_qsr_trace, world_trace, timestamps, qsr_params, req_params)
        return world_qsr_trace

    def _custom_checks_world_trace(self, world_trace):
        """Customs checks of the input data.

        :param world_trace: The input data.
        :type world_trace: World_Trace
        :return: False for no problems.
        :rtype: bool
        """
        return False

    def _set_input_world_trace(self, world_trace):
        self._custom_checks_world_trace(world_trace)
        return world_trace, world_trace.get_sorted_timestamps()

    def _process_qsrs_for(self, objects_names, dynamic_args, **kwargs):
        if isinstance(objects_names[0], str):
            try:
                return self.__check_qsrs_for_data_exist_at_world_state(objects_names,
                                                                       dynamic_args[self._unique_id]["qsrs_for"])
            except KeyError:
                try:
                    return self.__check_qsrs_for_data_exist_at_world_state(objects_names,
                                                                           dynamic_args["for_all_qsrs"]["qsrs_for"])
                except KeyError:
                    return self._init_qsrs_for_default(objects_names)
        elif isinstance(objects_names[0], (list, tuple)):
            qsrs_for_list = []
            for objects_names_i in objects_names:
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
            raise TypeError("objects_names must be a list of str or list of lists")

    def __check_qsrs_for_data_exist_at_world_state(self, objects_names_of_world_state, qsrs_for):
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
        """

        Overwrite as needed.

        :param req_params:
        :param kwargs:
        :return:
        """
        return {}

    def _postprocess_world_qsr_trace(self, world_qsr_trace, world_trace, world_trace_timestamps, qsr_params, req_params, **kwargs):
        """

        Overwrite as needed.

        :param world_qsr_trace:
        :param world_trace:
        :param world_trace_timestamps:
        :param qsr_params:
        :return:
        """
        return world_qsr_trace

    def _format_qsr(self, v):
        return {self._unique_id: v}
