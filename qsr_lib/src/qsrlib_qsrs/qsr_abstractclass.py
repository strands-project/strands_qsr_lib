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

class QSR_Abstractclass(object):
    """Abstract class for the QSR makers"""
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.qsr_type = ""  # must be the same that goes in the QSR_Lib.__const_qsrs_available
        self.all_possible_relations = []

    def help(self):
        self.custom_help()

    def check_input(self, input_data):
        error, msg = self.custom_checks(input_data)
        return error, msg

    def get(self, *args, **kwargs):
        error_code, error_msg = self.check_input(input_data=kwargs["input_data"])
        if error_code > 0:
            print("ERROR:", error_msg)
            self.help()
            print("\nFailed to compute QSRs")
            return False
        return self.make(*args, **kwargs)

    @abc.abstractmethod
    def custom_help(self):
        return

    @abc.abstractmethod
    def custom_checks(self, input_data):
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
