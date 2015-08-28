# -*- coding: utf-8 -*-
from __future__ import print_function, division
from datetime import datetime
from qsrlib_io.world_trace import World_Trace
from qsrlib_utils.utils import merge_world_qsr_traces
from qsrlib_qsrs import *


class QSRlib_Response_Message(object):
    def __init__(self, qsrs, req_made_at, req_received_at, req_finished_at):
        self.qsrs = qsrs
        self.req_made_at = req_made_at
        self.req_received_at = req_received_at
        self.req_finished_at = req_finished_at

class QSRlib_Request_Message(object):
    def __init__(self, which_qsr, input_data, dynamic_args={}, req_made_at=None, config=None):
        self.which_qsr = which_qsr
        if isinstance(input_data, World_Trace):
            self.input_data = input_data
        else:
            raise TypeError("input_data must be of type 'World_Trace'")
        self.dynamic_args = dynamic_args
        self.made_at = req_made_at if req_made_at else datetime.now()
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

    def get_qsrs_registry(self):
        return self.__qsrs

    def help(self):
        self.__print_qsrs_available()

    def __print_qsrs_available(self):
        print("Supported QSRs are:")
        for i in sorted(self.__qsrs):
            print("-", i)

    def request_qsrs(self, req_msg):
        """

        :param req_msg: QSRlib_Request_Message, default=None
        :return: QSRlib_Response_Message
        """
        req_received_at = datetime.now()
        world_qsr_traces = []

        # which_qsrs should always be iterable, even it is only a string, to enable the loop
        which_qsrs = req_msg.which_qsr if isinstance(req_msg.which_qsr, (list, tuple)) else [req_msg.which_qsr]
        for which_qsr in which_qsrs:
            world_qsr_traces.append(self.__qsrs[which_qsr].get_qsrs(input_data=req_msg.input_data,
                                                                    timestamp_request_received=req_received_at,
                                                                    config=req_msg.config,
                                                                    dynamic_args=req_msg.dynamic_args))
        if world_qsr_traces:
            # If the input was a list of QSRs, merge the results
            if isinstance(req_msg.which_qsr, (list, tuple)):
                world_qsr_trace = merge_world_qsr_traces(world_qsr_traces, ",".join(req_msg.which_qsr))
            elif len(world_qsr_traces) == 1:  # Just take the first because the list will only contain that one element
                world_qsr_trace = world_qsr_traces[0]
            else:
                raise RuntimeError("this should never have occured; file an issue for the developers to fix")
        else:
            world_qsr_trace = None

        qsrlib_response = QSRlib_Response_Message(qsrs=world_qsr_trace,
                                                  req_made_at=req_msg.made_at,
                                                  req_received_at=req_received_at,
                                                  req_finished_at=datetime.now())

        return qsrlib_response
