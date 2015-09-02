#!/usr/bin/env python
from __future__ import print_function, division
import unittest
from abc import ABCMeta, abstractmethod
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from tests_utils import *

class AbstractClass_Data1_UnitTest(unittest.TestCase):
    __metaclass__ = ABCMeta

    def __init__(self, *args):
        super(AbstractClass_Data1_UnitTest, self).__init__(*args)
        self._unique_id = ""
        self._world = None
        self._qsrlib = QSRlib()

    @abstractmethod
    def qsrs_for_global_namespace(self, gt_filename):
        return

    @abstractmethod
    def qsrs_for_qsr_namespace(self, gt_filename):
        return

    @abstractmethod
    def qsrs_for_qsr_namespace_over_global_namespace(self, gt_filename):
        return

    def defaults(self, gt_filename):
        gt_qsrs = unittest_read_qsrs_as_one_long_list(find_resource(PKG, gt_filename)[0])
        req_msg = QSRlib_Request_Message(self._unique_id, self._world)
        qsrs = unittest_get_qsrs_as_one_long_list(self._qsrlib.request_qsrs(req_msg).qsrs)
        return qsrs, gt_qsrs

    def q_factor(self, gt_filename, q_factor):
        gt_qsrs = unittest_read_qsrs_as_one_long_list(find_resource(PKG, gt_filename)[0])
        req_msg = QSRlib_Request_Message(self._unique_id, self._world,
                                         {self._unique_id: {"quantisation_factor": q_factor}})
        qsrs = unittest_get_qsrs_as_one_long_list(self._qsrlib.request_qsrs(req_msg).qsrs)
        return qsrs, gt_qsrs

    def custom(self, gt_filename, dynamic_args):
        gt_qsrs = unittest_read_qsrs_as_one_long_list(find_resource(PKG, gt_filename)[0])
        req_msg = QSRlib_Request_Message(self._unique_id, self._world, dynamic_args)
        qsrs = unittest_get_qsrs_as_one_long_list(self._qsrlib.request_qsrs(req_msg).qsrs)
        return qsrs, gt_qsrs
