# -*- coding: utf-8 -*-
"""Provides the abstract class of the QSR makers.

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 10 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""

from __future__ import print_function, division
import abc
import yaml
import os

import sys


class QSR_Abstractclass(object):
    """Abstract class for the QSR makers"""
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self._unique_id = ""  # must be the same that goes in the QSRlib.__qsrs_registration
        self.all_possible_relations = []

    def help(self):
        self.custom_help()

    # todo can be simplified a bit, also custom_checks possibly not needed anymore here
    def _set_input_world_trace(self, req_kwargs):
        try:
            world_trace = req_kwargs["input_data"]
            error_code, error_msg = self.custom_checks(input_data=world_trace)
            if error_code > 0:
                raise RuntimeError("Something wrong with the input data", error_code, error_msg)
        except KeyError:
            raise KeyError("No input data found.")
        timestamps = world_trace.get_sorted_timestamps()
        return world_trace, timestamps

    # todo rename get to a more meaningful name
    def get(self, *args, **kwargs):
        world_trace, timestamps = self._set_input_world_trace(req_kwargs=kwargs)
        qsr_params = self._process_qsr_parameters_from_request_parameters(kwargs)
        world_qsr_trace = self.make_world_qsr_trace(world_trace, timestamps, qsr_params)
        world_qsr_trace = self._postprocess_world_qsr_trace(world_qsr_trace, world_trace, timestamps, kwargs, qsr_params)
        return world_qsr_trace

    def _process_qsrs_for(self, objects_names_of_world_state, req_params, **kwargs):
        try:
            return self.__check_qsrs_for_data_exist_at_world_state(objects_names_of_world_state,
                                                                   req_params["dynamic_args"][self._unique_id]["qsrs_for"])
        except KeyError:
            try:
                return self.__check_qsrs_for_data_exist_at_world_state(objects_names_of_world_state,
                                                                       req_params["dynamic_args"]["for_all_qsrs"]["qsrs_for"])
            except KeyError:
                return self._init_qsrs_for_default(objects_names_of_world_state, req_params, **kwargs)

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
        qsrs_for_ret = self.custom_checks_for_qsrs_for(qsrs_for_ret)
        return qsrs_for_ret

    @abc.abstractmethod
    def _process_qsr_parameters_from_request_parameters(self, req_params, **kwargs):
        return

    @abc.abstractmethod
    def _postprocess_world_qsr_trace(self, world_qsr_trace, world_trace, world_trace_timestamps, req_params, qsr_params):
        return

    @abc.abstractmethod
    def _init_qsrs_for_default(self, objects_names_of_world_state, req_params, **kwargs):
        return

    @abc.abstractmethod
    def custom_help(self):
        return

    @abc.abstractmethod
    def custom_checks(self, input_data):
        return 0, ""

    @abc.abstractmethod
    def custom_checks_for_qsrs_for(self, qsrs_for, error_found):
        """Custom checks of the qsrs_for field.
        Hint: If you have to iterate over the qsrs_for make sure you do it on a copy of it or there might be dragons,
        e.g.:
         for p in list(qsrs_for):
            if p is not valid:
                qsrs_for.remove(p)
                error_found = True

        :param qsrs_for: list of strings and/or tuples for which QSRs will be computed
        :param error_found: if an error was found in the qsrs_for that violates the QSR rules
        :return: qsrs_for, error_found
        """

        return qsrs_for, error_found

    @abc.abstractmethod
    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, **kwargs):
        return

    def set_from_config_file(self, path):
        try:
            import rospkg
        except ImportError:
            raise ImportError("Module rospkg not found; setting from config file works for now only within the ROS eco-system")
        if path is None:
            path = os.path.join(rospkg.RosPack().get_path("qsr_lib"), "cfg/defaults.yml")
        else:
            path_ext = os.path.splitext(path)[1]
            if path_ext != ".yml" and path_ext != ".yaml":
                print("ERROR (qsr_abstractclass.py/set_from_config_file): Only yaml files are supported")
                raise ValueError
        with open(path, "r") as f:
            document = yaml.load(f)
        self.custom_set_from_config_file(document)

    @abc.abstractmethod
    def custom_set_from_config_file(self, document):
        return

    def handle_future(self, future, v, k):
        raise DeprecationWarning("future is default now, use format_qsr instead")
        # return {k: v} if future else v

    def format_qsr(self, v):
        return {self._unique_id: v}
