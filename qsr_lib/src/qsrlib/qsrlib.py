# -*- coding: utf-8 -*-
from __future__ import print_function, division
from datetime import datetime
from qsrlib_io.world_trace import World_Trace
from qsrlib_utils.utils import merge_world_qsr_traces
from qsrlib_qsrs import *


class QSRlib_Response_Message(object):
    def __init__(self, qsrs, timestamp_request_made, timestamp_request_received, timestamp_qsrs_computed):
        self.qsrs = qsrs
        self.timestamp_request_made = timestamp_request_made
        self.timestamp_request_received = timestamp_request_received
        self.timestamp_qsrs_computed = timestamp_qsrs_computed

class QSRlib_Request_Message(object):
    def __init__(self, which_qsr, input_data, dynamic_args={}, timestamp_request_made=None, config=None):
        self.which_qsr = which_qsr
        if isinstance(input_data, World_Trace):
            self.input_data = input_data
        else:
            raise TypeError("input_data must be of type 'World_Trace'")
        self.dynamic_args = dynamic_args
        self.timestamp_request_made = datetime.now() if timestamp_request_made is None else timestamp_request_made
        self.config = config

class QSRlib(object):
    """The LIB
    """
    def __init__(self, help=False):
        self.__qsrs = self.__check_and_activate_qsrs(qsrs_registry)
        if help:
            self.help()

    @staticmethod
    def __check_and_activate_qsrs(qsrs_registration):
        """Checks for uniqueness of the QSRs _unique_id and their corresponding class names and then return a dictionary
        with the unique IDs as keys and their corresponding objects.

        :param qsrs_registration: The dictionary where the QSRs are registered (see constructor source).
            :type qsrs_registration: tuple
        :return: A dictionary with the QSRs _unique_id as keys and an object of their corresponding classes
        :rtype: dict
        """
        if len(set(qsrs_registration)) != len(qsrs_registration):
            raise KeyError("Repeated class name found")
        qsrs = {}
        for class_name in qsrs_registration:
            o = class_name()
            if o._unique_id in qsrs:
                raise KeyError("Non unique QSR ID <%s> found while processing class <%s> which was mapped to class <%s>"
                               % (o._unique_id, o.__class__.__name__, qsrs[o._unique_id].__class__.__name__))
            else:
                qsrs[o._unique_id] = o
        return qsrs

    def help(self):
        self.__print_qsrs_available()

    def __print_qsrs_available(self):
        print("Supported QSRs are:")
        for i in sorted(self.__qsrs):
            print("-", i)

    def request_qsrs(self, request_message):
        """

        :param request_message: QSRlib_Request_Message, default=None
        :return: QSRlib_Response_Message
        """
        world_qsr_traces = []
        timestamp_request_received = datetime.now()

        # which_qsrs should always be iterable, even it is only a string, to enable the loop
        which_qsrs = request_message.which_qsr if isinstance(request_message.which_qsr, (list, tuple)) else [request_message.which_qsr]
        for which_qsr in which_qsrs:
            world_qsr_traces.append(self.__qsrs[which_qsr].get_qsrs(input_data=request_message.input_data,
                                                                    timestamp_request_received=timestamp_request_received,
                                                                    config=request_message.config,
                                                                    dynamic_args=request_message.dynamic_args))
        if world_qsr_traces:
            # If the input was a list of QSRs, merge the results
            if isinstance(request_message.which_qsr, (list, tuple)):
                world_qsr_trace = merge_world_qsr_traces(world_qsr_traces, ",".join(request_message.which_qsr))
            elif len(world_qsr_traces) == 1:  # Just take the first because the list will only contain that one element
                world_qsr_trace = world_qsr_traces[0]
            else:
                raise RuntimeError("this should never have occured; file an issue for the developers to fix")
        else:
            world_qsr_trace = None

        qsrlib_response = QSRlib_Response_Message(qsrs=world_qsr_trace,
                                                  timestamp_request_made=request_message.timestamp_request_made,
                                                  timestamp_request_received=timestamp_request_received,
                                                  timestamp_qsrs_computed=datetime.now())

        return qsrlib_response
