#!/usr/bin/env python
from __future__ import print_function, division
import sys
import random
from copy import deepcopy
from abstractclass_unittest import AbstractClass_UnitTest
from qsrlib.qsrlib import QSRlib_Request_Message
from unittests_utils import *


class Multiple_Test(AbstractClass_UnitTest):
    def __init__(self, *args):
        super(Multiple_Test, self).__init__(*args)
        self._unique_id = "multiple"
        self.__which_qsr = sorted(self._qsrlib.qsrs_registry.keys())
        self.__seed = 100
        self.__dynamic_args = {"argd": {"qsr_relations_and_values": {"close": 10.0, "near": 20.0,
                                                                     "far": 30.0, "veryfar": 40.0}},
                               "argprobd": {"qsr_relations_and_values": {"close": (10, 10/2), "near": (20, 20/2),
                                                                         "far": (30, 30/2), "veryfar": (40, 40/2)}},
                               "qtcbs": {"validate": False, "no_collapse": True},
                               "qtccs": {"validate": False, "no_collapse": True},
                               "qtcbcs": {"validate": False, "no_collapse": True}}

    def test_defaults(self):
        random.seed(self.__seed)
        self.assertItemsEqual(*self.defaults("data1", "data1_multiple_defaults.txt"))
        # with bounding boxes
        random.seed(self.__seed)
        self.assertItemsEqual(*self.defaults("data2_first100", "data2_first100_multiple_defaults.txt"))
        # without bounding boxes
        random.seed(self.__seed)
        self.assertItemsEqual(*self.defaults("data3_first100", "data3_first100_multiple_defaults.txt"))
        # floats
        random.seed(self.__seed)
        self.assertItemsEqual(*self.defaults("data4_first100", "data4_first100_multiple_defaults.txt"))

    def test_qsrs_for_global_namespace(self):
        random.seed(self.__seed)
        self.assertItemsEqual(*self.qsrs_for_global_namespace("data2_first100", "data2_first100_multiple_qsrs_for_global_namespace.txt"))

    def test_qsrs_for_qsr_namespace(self):
        random.seed(self.__seed)
        self.assertItemsEqual(*self.qsrs_for_qsr_namespace("data2_first100", "data2_first100_multiple_qsrs_for_qsr_namespace.txt"))

    def test_qsrs_for_qsr_namespace_over_global_namespace(self):
        random.seed(self.__seed)
        self.assertItemsEqual(*self.qsrs_for_qsr_namespace_over_global_namespace("data2_first100",
                                                                                 "data2_first100_multiple_qsrs_for_qsr_namespace_over_global.txt"))

    # overwrites parent method
    def defaults(self, world_name, gt_filename):
        expected = unittest_read_qsrs_as_one_long_list(find_resource(PKG, gt_filename)[0])
        actual = unittest_get_multiple_qsrs_as_one_long_list(self._qsrlib.request_qsrs(QSRlib_Request_Message(self.__which_qsr,
                                                                                                              self._worlds[world_name],
                                                                                                              self.__dynamic_args)).qsrs,
                                                             self.__which_qsr)
        return expected, actual

    # overwrites parent method
    def custom(self, world_name, gt_filename, dynamic_args):
        expected = unittest_read_qsrs_as_one_long_list(find_resource(PKG, gt_filename)[0])
        actual = unittest_get_multiple_qsrs_as_one_long_list(self._qsrlib.request_qsrs(QSRlib_Request_Message(self.__which_qsr,
                                                                                                              self._worlds[world_name],
                                                                                                              dynamic_args)).qsrs,
                                                             self.__which_qsr)
        return expected, actual

    # *** abstractmethods
    def qsrs_for_global_namespace(self, world_name, gt_filename):
        dynamic_args = deepcopy(self.__dynamic_args)
        dynamic_args["for_all_qsrs"] = {"qsrs_for": [("o3", "o2", "o1"), ("o2", "o1"), "o2"]}
        return self.custom(world_name, gt_filename, dynamic_args)

    def qsrs_for_qsr_namespace(self, world_name, gt_filename):
        dynamic_args = deepcopy(self.__dynamic_args)
        # cherry pick some of the qsrs
        dynamic_args["rcc2"] = {"qsrs_for": [("o1", "o2")]}
        dynamic_args["qtcbs"]["qsrs_for"] = [("o1", "o2")]
        dynamic_args["mwe"] = {"qsrs_for": [("o1", "o2")]}
        dynamic_args["mos"] = {"qsrs_for": ["o1"]}
        dynamic_args["tpcc"] = {"qsrs_for": [("o1", "o2", "o3")]}
        return self.custom(world_name, gt_filename, dynamic_args)

    def qsrs_for_qsr_namespace_over_global_namespace(self, world_name, gt_filename):
        dynamic_args = deepcopy(self.__dynamic_args)
        dynamic_args["for_all_qsrs"] = {"qsrs_for": [("o3", "o2", "o1"), ("o2", "o1"), "o2"]}
        # cherry pick some of the qsrs
        dynamic_args["rcc2"] = {"qsrs_for": [("o1", "o2")]}
        dynamic_args["qtcbs"]["qsrs_for"] = [("o1", "o2")]
        dynamic_args["mwe"] = {"qsrs_for": [("o1", "o2")]}
        dynamic_args["mos"] = {"qsrs_for": ["o1"]}
        dynamic_args["tpcc"] = {"qsrs_for": [("o1", "o2", "o3")]}
        return self.custom(world_name, gt_filename, dynamic_args)
    # *** eof abstractmethods


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("qsr_lib", "multiple_test", Multiple_Test, sys.argv)
