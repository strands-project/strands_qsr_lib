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
import ConfigParser
import rospkg
import os


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

    def check_qsrs_for_data_exist(self, objects_names, qsrs_for):
        if len(objects_names) == 0:
            error_found = True if qsrs_for else False
            return [], error_found

        if type(qsrs_for) is not list:
            raise ValueError("qsrs_for must be a list of strings and/or tuples of strings")

        qsrs_for_ret = []
        error_found = False

        for p in qsrs_for:
            if type(p) is str:
                if p in objects_names:
                    qsrs_for_ret.append(p)
            elif (type(p) is tuple) or (type(p) is list):
                tuple_data_exists = True
                for o in p:
                    if o not in objects_names:
                        tuple_data_exists = False
                        break
                if tuple_data_exists:
                    qsrs_for_ret.append(p)
            else:
                raise ValueError("Elements of qsrs_for must be strings and/or tuples")

        qsrs_for_ret, error_found = self.custom_checks_for_qsrs_for(qsrs_for_ret, error_found)
        return qsrs_for_ret, error_found

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
    def make(self, *args, **kwargs):
        """Abstract method that needs to be implemented by the QSR makers

        :param args: not really used at the moment
        :param kwargs:
                    - "input_data": Input_Data_Block
        :return:
        """
        return

    def set_from_ini(self, ini):
        if ini is None:
            ini = os.path.join(rospkg.RosPack().get_path("qsr_lib"), "cfg/defaults.ini")
        parser = ConfigParser.SafeConfigParser()
        if len(parser.read(ini)) == 0:
            raise IOError
        self.custom_set_from_ini(parser)

    @abc.abstractmethod
    def custom_set_from_ini(self, parser):
        return

    def handle_future(self, future, v, k=None):
        if future:
            if k is None:
                raise ValueError("None qsr key")
            return {k: v}
        else:
            return v
