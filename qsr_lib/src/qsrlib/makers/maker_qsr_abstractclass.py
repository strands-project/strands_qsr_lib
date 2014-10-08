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
from qsrlib.input_data import Input_Data_Block

class Maker_QSR_Abstractclass(object):
    """Abstract class for the QSR makers"""
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.qsr_type = ""  # must be the same that goes in the QSR_Lib.__const_qsrs_available
        self.required_fields = []
        self.all_possible_relations = []

    def help(self):
        print("QSR requires the following input fields:", self.required_fields)
        self.custom_help()
        print("Please provide the appropriate fields and format your input data accordingly.")

    def check_input(self, input_data):
        if not isinstance(input_data, Input_Data_Block):
            return 1, "Input data has incorrect type"
        for f in self.required_fields:
            if f not in input_data.fields:
                msg = "Required field (" + f + ") not found"
                return 2, msg
        if len(input_data.fields) != len(self.required_fields):
            return 3, "Incorrect number of input fields"
        if len(self.required_fields) * input_data.timesteps != len(input_data.data[0].data):
            return 4, "Length of input data mismatch with what was expected"
        error, msg = self.custom_checks()
        return error, msg

    @abc.abstractmethod
    def custom_help(self):
        return

    @abc.abstractmethod
    def custom_checks(self):
        return 0, ""

    @abc.abstractmethod
    def make(self, *args, **kwargs):
        """Abstract method that needs to be implemented by the QSR makers

        :param args: not really used at the moment
        :param kwargs:
                    - "input_data": Input_Data_Block
        :return:
        """
        return
