#!/usr/bin/env python
from __future__ import print_function, division
import sys
from abstractclass_unittest_dyadic import Abstractclass_Unittest_Dyadic


class ArgD_Test(Abstractclass_Unittest_Dyadic):
    def __init__(self, *args):
        super(ArgD_Test, self).__init__(*args)
        self._unique_id = "argd"
        self.__params = {"qsr_relations_and_values": {"close": 10.0, "near": 20.0, "far": 30.0, "veryfar": 40.0}}
        self.__custom = {self._unique_id: {"qsrs_for": [("o1", "o2")],
                                           "qsr_relations_and_values": self.__params["qsr_relations_and_values"]}}

    def test_defaults(self):
        # with bounding boxes
        self.assertItemsEqual(*self.custom("data2", "data2_argd_defaults.txt", {self._unique_id: self.__params}))
        # without bounding boxes
        self.assertItemsEqual(*self.custom("data3", "data3_argd_defaults.txt", {self._unique_id: self.__params}))
        # floats
        self.assertItemsEqual(*self.custom("data4", "data4_argd_defaults.txt", {self._unique_id: self.__params}))

    def test_qsrs_for_global_namespace(self):
        self.assertItemsEqual(*self.custom("data2", "data2_argd_qsrs_for_global_namespace.txt",
                                           {"for_all_qsrs": {"qsrs_for": [("o2", "o1")]}, self._unique_id: self.__params}))

    def test_qsrs_for_qsr_namespace(self):
        self.assertItemsEqual(*self.custom("data2", "data2_argd_qsrs_for_qsr_namespace.txt",
                                           {self._unique_id: {"qsrs_for": [("o1", "o2")],
                                                              "qsr_relations_and_values": self.__params["qsr_relations_and_values"]}}))
        self.assertItemsEqual(*self.custom("data2", "data2_argd_qsrs_for_qsr_namespace.txt",
                                           {"for_all_qsrs": {"qsrs_for": [("o2", "o1")]},
                                            self._unique_id: {"qsrs_for": [("o1", "o2")],
                                                              "qsr_relations_and_values": self.__params["qsr_relations_and_values"]}}))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("qsr_lib", "argd_test", ArgD_Test, sys.argv)
