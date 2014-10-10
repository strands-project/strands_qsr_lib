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
from qsrlib_legacy.input_data import Input_Data_One, Input_Data_Block
from qsrlib_legacy.output_data import Output_Data

# Import implemented makers
from qsrlib_legacy.maker_qsr_rcc3_rectangle_bounding_boxes_2d import Maker_QSR_RCC3_Rectangle_Bounding_Boxes_2D
from qsrlib_legacy.maker_qsr_qtc_b_simplified import Maker_QSR_QTC_B_Simplified

class QSRlib(object):
    """The LIB

    At the moment the data data variables bounding_boxes_2d & 3d and trajectories_2d & 3d
    are pretty loose in terms of definition... it can be pretty much anything... this will create
    problems eventually to the makers... as such I think a "strict" prototype should be suggested...
    I think it should be a list of dictionaries with each dictionary keeping an id and
    a list of the data data, e.g.
    self.bounding_boxes_2d = [{"id": "some_id", "data":[x10, y10, w1, h1, x20, y20, w2, h2, ...]}]
    or define a new class probably for input_data
    """
    def __init__(self, qsrs_active=None, print_messages=True, help=False):
        self.__const_qsrs_available = {"rcc3_rectangle_bounding_boxes_2d": Maker_QSR_RCC3_Rectangle_Bounding_Boxes_2D,
                                       "qtc_b_simplified": Maker_QSR_QTC_B_Simplified}
        self.__qsrs_active = {}
        self.__set_qsrs_active(qsrs_active)
        if help:
            self.help()
        self.timestamp_request_received = None
        self.which_qsr = None
        self.input_data = None

        # these are the droids you are not looking for
        self.__out = print_messages

    def help(self):
        self.print_qsrs_available()
        print()
        self.print_qsrs_active()
        print()

    def set_out(self, b):
        self.__out = b

    def reset_all(self):
        self.timestamp_request_received = None
        self.input_data = None
        self.which_qsr = None

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

    def set_which_qsr(self, which_qsr):
        code = 0
        if which_qsr is None:
            if self.__out: print("ERROR (QSR_Lib.set_which_qsr): argument is empty, please specify which qsr you would like computed")
            code = 1
        self.which_qsr = which_qsr
        return code

    def get_which_qsr(self):
        return self.which_qsr

    def set_input_data(self, input_data):
        if input_data is None:
            input_data = Input_Data_Block()
        if isinstance(input_data, Input_Data_Block):
            if len(input_data.data) > 0:
                self.input_data = input_data
            else:
                if self.__out:
                    if len(self.input_data.data) > 0:
                        print("Reusing previous input data.")
                    else:
                        print("Warning (QSRlib.set_input_data): It seems you are trying to reuse previous data, but previous data is empty")
        else:
            if self.__out: print("ERROR (QSR_Lib.set_input_data): input data has incorrect type, must be of type 'Input_Data_Block'")
            self.input_data = Input_Data_Block(description="error: input data has incorrect type")

    def get_input_data(self):
        return self.input_data

    def request_qsrs(self, which_qsr, input_data=None, reset=False):
        self.timestamp_request_received = datetime.now()
        self.set_which_qsr(which_qsr)
        self.set_input_data(input_data=input_data)
        # if not isinstance(input_data, Input_Data_Block):
        #     if self.__out: print("ERROR (QSR_Lib.request_qsrs): input data has incorrect type, must be of type 'Input_Data_Block'")
        #     ret = Output_Data(qsr_type="error: input data has incorrect type")
        # else:
        try:
            ret = self.__qsrs_active[self.which_qsr].make(input_data=self.input_data,
                                                          timestamp_request_received=self.timestamp_request_received)
        except KeyError:
            print("ERROR (QSR_Lib.request): it seems that the QSR you requested (" + which_qsr + ") is not implemented yet or has not been activated")
            ret = Output_Data(qsr_type="error", timestamp_request_received=self.timestamp_request_received)
        if reset:
            if self.__out: print("Resetting self.input_data in QSRlib")
            self.reset_all()
        return ret


if __name__ == "__main__":
    # define some dummy sample data
    input_data = Input_Data_Block(data=[Input_Data_One("1", [1.0, 1.0, 2.0, 2.0, 1.0, 1.0, 2.0, 2.0, 1.0, 1.0, 2.0, 2.0]),
                                        Input_Data_One("2", [11.0, 11.0, 22.0, 22.0, 1.5, 1.5, 22.0, 22.0, 1.5, 1.5, 1.5, 1.5])],
                                  fields=["x1", "y1", "x2", "y2"],
                                  timesteps=3,
                                  description="some 2d bounding boxes")
    # make a QSRlib object
    qsrlib = QSRlib()
    # request QSRs
    out = qsrlib.request_qsrs(which_qsr="rcc3_rectangle_bounding_boxes_2d", input_data=input_data)
    # print the timestamps, ids and qsrs
    print("Request was received at", out.timestamp_request_received, "and finished processing at", out.timestamp_qsrs_processed)
    print("Objects:", out.ids)
    print("QSRs:", out.data)
