#!/usr/bin/env python
from __future__ import print_function, division
from abc import ABCMeta
from abstractclass_unittest_data1 import AbstractClass_Data1_UnitTest
from qsrlib.qsrlib import QSRlib_Request_Message
from tests_utils import *

class Abstractclass_Unittest_Data1_Monadic(AbstractClass_Data1_UnitTest):
    __metaclass__ = ABCMeta

    def __init__(self, *args):
        super(Abstractclass_Unittest_Data1_Monadic, self).__init__(*args)

    def qsrs_for_global_namespace(self, gt_filename):
        gt_qsrs = unittest_read_qsrs_as_one_long_list(find_resource(PKG, gt_filename)[0])
        req_msg = QSRlib_Request_Message(self._unique_id, self._world,
                                         {"for_all_qsrs": {"qsrs_for": ["o2"]}})
        qsrs = unittest_get_qsrs_as_one_long_list(self._qsrlib.request_qsrs(req_msg).qsrs)
        return qsrs, gt_qsrs

    def qsrs_for_qsr_namespace(self, gt_filename):
        gt_qsrs = unittest_read_qsrs_as_one_long_list(find_resource(PKG, gt_filename)[0])
        req_msg = QSRlib_Request_Message(self._unique_id, self._world,
                                         {self._unique_id: {"qsrs_for": ["o1"]}})
        qsrs = unittest_get_qsrs_as_one_long_list(self._qsrlib.request_qsrs(req_msg).qsrs)
        return qsrs, gt_qsrs

    def qsrs_for_qsr_namespace_over_global_namespace(self, gt_filename):
        gt_qsrs = unittest_read_qsrs_as_one_long_list(find_resource(PKG, gt_filename)[0])
        req_msg = QSRlib_Request_Message(self._unique_id, self._world,
                                         {"for_all_qsrs": {"qsrs_for": ["o2"]},
                                          self._unique_id: {"qsrs_for": ["o1"]}})
        qsrs = unittest_get_qsrs_as_one_long_list(self._qsrlib.request_qsrs(req_msg).qsrs)
        return qsrs, gt_qsrs
