#!/usr/bin/env python
from __future__ import print_function, division
import sys
import random
from abstractclass_unittest_dyadic import Abstractclass_Unittest_Dyadic


class ArgProbD_Test(Abstractclass_Unittest_Dyadic):
    def __init__(self, *args):
        super(ArgProbD_Test, self).__init__(*args)
        self._unique_id = "argprobd"
        self.__params = {"qsr_relations_and_values": {"close": (10, 10/2), "near": (20, 20/2),
                                                      "far": (30, 30/2), "veryfar": (40, 40/2)}}
        self.__seed = 100

    def test_defaults(self):
        # with bounding boxes
        random.seed(self.__seed)
        self.assertItemsEqual(*self.custom("data2", "data2_argprobd_defaults.txt", {self._unique_id: self.__params}))
        # without bounding boxes
        random.seed(self.__seed)
        self.assertItemsEqual(*self.custom("data3", "data3_argprobd_defaults.txt", {self._unique_id: self.__params}))
        # floats
        random.seed(self.__seed)
        self.assertItemsEqual(*self.custom("data4", "data4_argprobd_defaults.txt", {self._unique_id: self.__params}))

    def test_qsrs_for_global_namespace(self):
        random.seed(self.__seed)
        self.assertItemsEqual(*self.custom("data2", "data2_argprobd_qsrs_for_global_namespace.txt",
                                           {"for_all_qsrs": {"qsrs_for": [("o2", "o1")]}, self._unique_id: self.__params}))

    def test_qsrs_for_qsr_namespace(self):
        random.seed(self.__seed)
        self.assertItemsEqual(*self.custom("data2", "data2_argprobd_qsrs_for_qsr_namespace.txt",
                                           {self._unique_id: {"qsrs_for": [("o1", "o2")],
                                                              "qsr_relations_and_values": self.__params["qsr_relations_and_values"]}}))
        random.seed(self.__seed)
        self.assertItemsEqual(*self.custom("data2", "data2_argprobd_qsrs_for_qsr_namespace.txt",
                                           {"for_all_qsrs": {"qsrs_for": [("o2", "o1")]},
                                            self._unique_id: {"qsrs_for": [("o1", "o2")],
                                                              "qsr_relations_and_values": self.__params["qsr_relations_and_values"]}}))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("qsr_lib", "argprobd_test", ArgProbD_Test, sys.argv)
