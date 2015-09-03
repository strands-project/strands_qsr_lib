#!/usr/bin/env python
from __future__ import print_function, division
import unittest
from abc import ABCMeta, abstractmethod
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from unittests_data_loaders import *
from unittests_utils import *

class AbstractClass_UnitTest(unittest.TestCase):
    __metaclass__ = ABCMeta

    def __init__(self, *args):
        super(AbstractClass_UnitTest, self).__init__(*args)
        self._unique_id = ""
        self._worlds = {"data1": load_input_data1(),
                        "data2": load_input_data2(),
                        "data3": load_input_data3(),
                        "data4": load_input_data4()}
        self._qsrlib = QSRlib()

    @abstractmethod
    def qsrs_for_global_namespace(self, world_name, gt_filename):
        return

    @abstractmethod
    def qsrs_for_qsr_namespace(self, world_name, gt_filename):
        return

    @abstractmethod
    def qsrs_for_qsr_namespace_over_global_namespace(self, world_name, gt_filename):
        return

    def defaults(self, world_name, gt_filename):
        expected = unittest_read_qsrs_as_one_long_list(find_resource(PKG, gt_filename)[0])
        req_msg = QSRlib_Request_Message(self._unique_id, self._worlds[world_name])
        actual = unittest_get_qsrs_as_one_long_list(self._qsrlib.request_qsrs(req_msg).qsrs)
        return expected, actual

    def q_factor(self, world_name, gt_filename, q_factor):
        expected = unittest_read_qsrs_as_one_long_list(find_resource(PKG, gt_filename)[0])
        req_msg = QSRlib_Request_Message(self._unique_id, self._worlds[world_name],
                                         {self._unique_id: {"quantisation_factor": q_factor}})
        actual = unittest_get_qsrs_as_one_long_list(self._qsrlib.request_qsrs(req_msg).qsrs)
        return expected, actual

    def custom(self, world_name, gt_filename, dynamic_args):
        expected = unittest_read_qsrs_as_one_long_list(find_resource(PKG, gt_filename)[0])
        req_msg = QSRlib_Request_Message(self._unique_id, self._worlds[world_name], dynamic_args)
        actual = unittest_get_qsrs_as_one_long_list(self._qsrlib.request_qsrs(req_msg).qsrs)
        return expected, actual

