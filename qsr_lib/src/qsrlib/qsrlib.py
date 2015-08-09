# -*- coding: utf-8 -*-
"""Provides a class that wraps QSR makers making requests and returns transparent and easy.

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 10 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""

from __future__ import print_function, division
from datetime import datetime
from qsrlib_io.world_trace import World_Trace
from qsrlib_utils.utils import merge_world_qsr_traces

# Import implemented makers
from qsrlib_qsrs.qsr_rcc2_rectangle_bounding_boxes_2d import QSR_RCC2_Rectangle_Bounding_Boxes_2D
from qsrlib_qsrs.qsr_rcc3_rectangle_bounding_boxes_2d import QSR_RCC3_Rectangle_Bounding_Boxes_2D
from qsrlib_qsrs.qsr_rcc8_rectangle_bounding_boxes_2d import QSR_RCC8_Rectangle_Bounding_Boxes_2D
from qsrlib_qsrs.qsr_cone_direction_bounding_boxes_centroid_2d import QSR_Cone_Direction_Bounding_Boxes_Centroid_2D
from qsrlib_qsrs.qsr_qtc_b_simplified import QSR_QTC_B_Simplified
from qsrlib_qsrs.qsr_qtc_c_simplified import QSR_QTC_C_Simplified
from qsrlib_qsrs.qsr_qtc_bc_simplified import QSR_QTC_BC_Simplified
from qsrlib_qsrs.qsr_arg_relations_distance import QSR_Arg_Relations_Distance
from qsrlib_qsrs.qsr_arg_prob_relations_distance import QSR_Arg_Prob_Relations_Distance
from qsrlib_qsrs.qsr_moving_or_stationary import QSR_Moving_or_Stationary


class QSRlib_Response_Message(object):
    def __init__(self, qsrs, timestamp_request_made, timestamp_request_received, timestamp_qsrs_computed):
        self.qsrs = qsrs
        self.timestamp_request_made = timestamp_request_made
        self.timestamp_request_received = timestamp_request_received
        self.timestamp_qsrs_computed = timestamp_qsrs_computed

class QSRlib_Request_Message(object):
    def __init__(self, which_qsr="", input_data=None, qsrs_for=[], timestamp_request_made=None,
                 start=0, finish=-1, objects_names=[], include_missing_data=True, qsr_relations_and_values={},
                 future=False, config=None, dynamic_args=None):
        self.future = future
        self.which_qsr = which_qsr
        self.input_data = None
        self.set_input_data(input_data=input_data, start=start, finish=finish, objects_names=objects_names)
        self.qsrs_for = qsrs_for
        self.timestamp_request_made = datetime.now() if timestamp_request_made is None else timestamp_request_made
        self.include_missing_data = include_missing_data
        self.qsr_relations_and_values = qsr_relations_and_values # should be more dynamic
        self.config = config
        self.dynamic_args = dynamic_args

    def make(self, which_qsr, input_data, qsrs_for=[], timestamp_request_made=None, future=None, ini=None,
             dynamic_args=None):
        if future:
            self.future = future
        self.which_qsr = which_qsr
        self.input_data = self.set_input_data(input_data)
        self.qsrs_for = qsrs_for
        self.timestamp_request_made = datetime.now() if timestamp_request_made is None else timestamp_request_made
        self.ini = None
        self.dynamic_args = None

    def set_input_data(self, input_data, start=0, finish=-1, objects_names=[]):
        # todo this function needs updating, still should work but has redundant code (e.g. "reusing previous input data" should never occur
        error = False
        if input_data is None:
            input_data = World_Trace()
        if isinstance(input_data, World_Trace):
            if len(input_data.trace) > 0:
                self.input_data = input_data
            else:
                if self.input_data is not None and len(self.input_data.trace) > 0:
                    print("Reusing previous input data")
                else:
                    print("Warning (QSRlib_Request_Message.set_input_data): It seems you are trying to reuse previous data, but previous data is empty")
                    self.input_data = World_Trace(description="error")
                    error = True
        else:
            print("ERROR (QSRLib_.set_input_data): input data has incorrect type, must be of type 'World_Trace'")
            self.input_data = World_Trace(description="error")
            error = True
        if not error:
            if finish >= 0:
                self.input_data = self.input_data.get_at_timestamp_range(start=start, finish=finish)
            if len(objects_names) > 0:
                self.input_data = self.input_data.get_for_objects(objects_names=objects_names)

class QSRlib(object):
    """The LIB
    """
    def __init__(self, help=True):
        self.__qsrs = {"rcc2_rectangle_bounding_boxes_2d": QSR_RCC2_Rectangle_Bounding_Boxes_2D(),
                       "rcc3_rectangle_bounding_boxes_2d": QSR_RCC3_Rectangle_Bounding_Boxes_2D(),
                       "rcc8_rectangle_bounding_boxes_2d": QSR_RCC8_Rectangle_Bounding_Boxes_2D(),
                       "cone_direction_bounding_boxes_centroid_2d": QSR_Cone_Direction_Bounding_Boxes_Centroid_2D(),
                       "qtc_b_simplified": QSR_QTC_B_Simplified(),
                       "qtc_c_simplified": QSR_QTC_C_Simplified(),
                       "qtc_bc_simplified": QSR_QTC_BC_Simplified(),
                       "arg_relations_distance": QSR_Arg_Relations_Distance(),
                       "arg_prob_relations_distance": QSR_Arg_Prob_Relations_Distance(),
                       "moving_or_stationary": QSR_Moving_or_Stationary()}

        if help:
            self.help()

        # register short names
        for k, v in self.__qsrs.items():
            if v.qsr_keys not in self.__qsrs:
                self.__qsrs[v.qsr_keys] = self.__qsrs[k]
            else:
                raise KeyError("Non-unique shortname <%s> found for <%s> while registered to <%s>. Shortnames must also be unique keys."
                               % (v.qsr_keys, v.qsr_type, self.__qsrs[v.qsr_keys].qsr_type))

    def help(self):
        self.print_qsrs_available()

    def print_qsrs_available(self):
        l = sorted(self.__qsrs)
        print("Types of QSRs that have been included so far in the lib are the following:")
        for i in l:
            print("-", i, "("+self.__qsrs[i].qsr_keys+")")

    def request_qsrs(self, request_message):
        """

        :param request_message: QSRlib_Request_Message, default=None
        :return: QSRlib_Response_Message
        """
        world_qsr_traces = []
        timestamp_request_received = datetime.now()

        # Checking if future is True when a list of QSRs is given.
        # If not, print error as the string results do not support multiple
        # QSRs at the same time
        if not request_message.future and isinstance(request_message.which_qsr, (list, tuple)):
            raise RuntimeError("(QSR_Lib.request_qsrs): Using a", type(request_message.which_qsr), "of qsrs:", request_message.which_qsr, "is only supported when using future: future = True")
        else:
            # which_qsrs should always be iterable, even it is only a string, to enable the loop
            which_qsrs = request_message.which_qsr if isinstance(request_message.which_qsr, (list, tuple)) else [request_message.which_qsr]
            for which_qsr in which_qsrs:
                try:
                    world_qsr_traces.append(self.__qsrs[which_qsr].get(input_data=request_message.input_data,
                                                                       include_missing_data=request_message.include_missing_data,
                                                                       timestamp_request_received=timestamp_request_received,
                                                                       qsrs_for=request_message.qsrs_for,
                                                                       qsr_relations_and_values=request_message.qsr_relations_and_values,
                                                                       future=request_message.future,
                                                                       config=request_message.config,
                                                                       dynamic_args=request_message.dynamic_args))
                except KeyError:
                    raise KeyError("(QSR_Lib.request_qsrs): it seems that the QSR you requested (" + which_qsr + ") is not implemented yet or has not been activated")

        if world_qsr_traces:
            # If the input was a list of QSRs, merge the results
            if request_message.future and isinstance(request_message.which_qsr, (list, tuple)):
                world_qsr_trace = merge_world_qsr_traces(world_qsr_traces, ",".join(request_message.which_qsr))
            elif len(world_qsr_traces) == 1:  # Just take the first because the list will only contain that one element
                world_qsr_trace = world_qsr_traces[0]
            else:
                raise RuntimeError("(QSR_Lib.request_qsrs): this should never have occured; file an issue for the developers to fix")
        else:
            # If something went wrong, world_qsr_traces will be False.
            # Setting world_qsr_trace to the same value to preserve previous behaviour.
            world_qsr_trace = world_qsr_traces

        qsrlib_response = QSRlib_Response_Message(qsrs=world_qsr_trace,
                                                  timestamp_request_made=request_message.timestamp_request_made,
                                                  timestamp_request_received=timestamp_request_received,
                                                  timestamp_qsrs_computed=datetime.now())

        return qsrlib_response
