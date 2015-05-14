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

# Import implemented makers
from qsrlib_qsrs.qsr_rcc2_rectangle_bounding_boxes_2d import QSR_RCC2_Rectangle_Bounding_Boxes_2D
from qsrlib_qsrs.qsr_rcc3_rectangle_bounding_boxes_2d import QSR_RCC3_Rectangle_Bounding_Boxes_2D
from qsrlib_qsrs.qsr_rcc8_rectangle_bounding_boxes_2d import QSR_RCC8_Rectangle_Bounding_Boxes_2D
from qsrlib_qsrs.qsr_cone_direction_bounding_boxes_centroid_2d import QSR_Cone_Direction_Bounding_Boxes_Centroid_2D
from qsrlib_qsrs.qsr_qtc_b_simplified import QSR_QTC_B_Simplified
from qsrlib_qsrs.qsr_qtc_c_simplified import QSR_QTC_C_Simplified
from qsrlib_qsrs.qsr_qtc_bc_simplified import QSR_QTC_BC_Simplified
from qsrlib_qsrs.qsr_arg_relations_distance import QSR_Arg_Relations_Distance

class QSRlib_Response_Message(object):
    def __init__(self, qsrs, timestamp_request_made, timestamp_request_received, timestamp_qsrs_computed):
        self.qsrs = qsrs
        self.timestamp_request_made = timestamp_request_made
        self.timestamp_request_received = timestamp_request_received
        self.timestamp_qsrs_computed = timestamp_qsrs_computed

class QSRlib_Request_Message(object):
    def __init__(self, which_qsr="", input_data=None, qsrs_for=[], timestamp_request_made=None,
                 start=0, finish=-1, objects_names=[], include_missing_data=True, qsr_relations_and_values={},
                 future=False, ini=None, dynamic_args=None):
        self.future = future
        self.which_qsr = which_qsr
        self.input_data = None
        self.set_input_data(input_data=input_data, start=start, finish=finish, objects_names=objects_names)
        self.qsrs_for = qsrs_for
        self.timestamp_request_made = datetime.now() if timestamp_request_made is None else timestamp_request_made
        self.include_missing_data = include_missing_data
        self.qsr_relations_and_values = qsr_relations_and_values # should be more dynamic
        self.ini = ini
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
    def __init__(self, qsrs_active=None, print_messages=True, help=True, request_message=None):
        self.__const_qsrs_available = {"rcc2_rectangle_bounding_boxes_2d": QSR_RCC2_Rectangle_Bounding_Boxes_2D,
                                       "rcc3_rectangle_bounding_boxes_2d": QSR_RCC3_Rectangle_Bounding_Boxes_2D,
                                       "rcc8_rectangle_bounding_boxes_2d": QSR_RCC8_Rectangle_Bounding_Boxes_2D,
                                       "cone_direction_bounding_boxes_centroid_2d": QSR_Cone_Direction_Bounding_Boxes_Centroid_2D,
                                       "qtc_b_simplified": QSR_QTC_B_Simplified,
                                       "qtc_c_simplified": QSR_QTC_C_Simplified,
                                       "qtc_bc_simplified": QSR_QTC_BC_Simplified,
                                       "arg_relations_distance": QSR_Arg_Relations_Distance}
        self.__qsrs_active = {}
        self.__set_qsrs_active(qsrs_active)
        if help:
            self.help()
        self.request_message = request_message
        self.timestamp_request_received = None

        # these are the droids you are not looking for
        self.__out = print_messages

    def help(self):
        self.print_qsrs_available()
        print()
        self.print_qsrs_active()
        print()

    def set_out(self, b):
        self.__out = b

    def print_qsrs_available(self):
        l = sorted(self.__const_qsrs_available)
        print("Types of QSRs that have been included so far in the lib are the following:")
        for i in l:
            print("-", i)

    def print_qsrs_active(self):
        l = sorted(self.__qsrs_active)
        print("Types of QSRs that you enabled are:")
        for i in l:
            print("-", i)

    def __set_qsrs_active(self, qsrs_active):
        if qsrs_active is None:
            for qsr_type, class_name in zip(self.__const_qsrs_available.keys(), self.__const_qsrs_available.values()):
                self.__qsrs_active[qsr_type] = class_name()
        else:
            for qsr_type in qsrs_active:
                try:
                    self.__qsrs_active[qsr_type] = self.__const_qsrs_available[qsr_type]()
                except KeyError:
                    print("ERROR (QSR_Lib.__set_qsrs_active): it seems that this QSR type '" + qsr_type + "' has not been implemented yet; or maybe a typo?")

    def request_qsrs(self, request_message, reset=True):
        """

        :param request_message: QSRlib_Request_Message, default=None
        :param reset: Boolean, if to reset the self.request_message, default=True
        :return: QSRlib_Response_Message
        """
        self.timestamp_request_received = datetime.now()
        self.request_message = request_message
        try:
            world_qsr_trace = self.__qsrs_active[self.request_message.which_qsr].get(input_data=self.request_message.input_data,
                                                                                     include_missing_data=self.request_message.include_missing_data,
                                                                                     timestamp_request_received=self.timestamp_request_received,
                                                                                     qsrs_for=self.request_message.qsrs_for,
                                                                                     qsr_relations_and_values=self.request_message.qsr_relations_and_values,
                                                                                     future=self.request_message.future,
                                                                                     ini=self.request_message.ini,
                                                                                     dynamic_args=self.request_message.dynamic_args)
        except KeyError:
            print("ERROR (QSR_Lib.request_qsrs): it seems that the QSR you requested (" + self.request_message.which_qsr + ") is not implemented yet or has not been activated")
            world_qsr_trace = False
        if reset:
            self.timestamp_request_received = None
            self.request_message = None
        else:
            if self.__out: print("QSRlib data not resetted, working with previous held data. Pass the data as this feature is planned to be deprecated.")
        qsrlib_response = QSRlib_Response_Message(qsrs=world_qsr_trace,
                                                  timestamp_request_made=request_message.timestamp_request_made,
                                                  timestamp_request_received=self.timestamp_request_received,
                                                  timestamp_qsrs_computed=datetime.now())

        return qsrlib_response
