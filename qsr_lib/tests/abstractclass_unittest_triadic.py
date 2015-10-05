#!/usr/bin/env python
from __future__ import print_function, division
from abc import ABCMeta
from abstractclass_unittest import AbstractClass_UnitTest
from qsrlib.qsrlib import QSRlib_Request_Message
from unittests_utils import *

class Abstractclass_Unittest_Triadic(AbstractClass_UnitTest):
    __metaclass__ = ABCMeta

    def __init__(self, *args):
        super(Abstractclass_Unittest_Triadic, self).__init__(*args)

    def qsrs_for_global_namespace(self, world_name, gt_filename):
        expected = unittest_read_qsrs_as_one_long_list(find_resource(PKG, gt_filename)[0])
        req_msg = QSRlib_Request_Message(self._unique_id, self._worlds[world_name],
                                         {"for_all_qsrs": {"qsrs_for": [("o3", "o2", "o1")]}})
        actual = unittest_get_qsrs_as_one_long_list(self._qsrlib.request_qsrs(req_msg).qsrs)
        return expected, actual

    def qsrs_for_qsr_namespace(self, world_name, gt_filename):
        expected = unittest_read_qsrs_as_one_long_list(find_resource(PKG, gt_filename)[0])
        req_msg = QSRlib_Request_Message(self._unique_id, self._worlds[world_name],
                                         {self._unique_id: {"qsrs_for": [("o1", "o2", "o3")]}})
        actual = unittest_get_qsrs_as_one_long_list(self._qsrlib.request_qsrs(req_msg).qsrs)
        return expected, actual

    def qsrs_for_qsr_namespace_over_global_namespace(self, world_name, gt_filename):
        expected = unittest_read_qsrs_as_one_long_list(find_resource(PKG, gt_filename)[0])
        req_msg = QSRlib_Request_Message(self._unique_id, self._worlds[world_name],
                                         {"for_all_qsrs": {"qsrs_for": [("o3", "o2", "o1")]},
                                          self._unique_id: {"qsrs_for": [("o1", "o2", "o3")]}})
        actual = unittest_get_qsrs_as_one_long_list(self._qsrlib.request_qsrs(req_msg).qsrs)
        return actual, expected
