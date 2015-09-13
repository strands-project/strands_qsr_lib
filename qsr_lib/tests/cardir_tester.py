#!/usr/bin/env python
from __future__ import print_function, division
import sys
from abstractclass_unittest_dyadic import Abstractclass_Unittest_Dyadic


class cardir_Test(Abstractclass_Unittest_Dyadic):
    def __init__(self, *args):
        super(cardir_Test, self).__init__(*args)
        self._unique_id = "cardir"

    def test_defaults(self):
        self.assertItemsEqual(*self.defaults("data1", "data1_cardir_defaults.txt"))

    def test_qsrs_for_global_namespace(self):
        self.assertItemsEqual(*self.qsrs_for_global_namespace("data1", "data1_cardir_qsrs_for_global_namespace.txt"))

    def test_qsrs_for_qsr_namespace(self):
        self.assertItemsEqual(*self.qsrs_for_qsr_namespace("data1", "data1_cardir_qsrs_for_qsr_namespace.txt"))
        # precedes "for_all_qsrs" namespace
        self.assertItemsEqual(*self.qsrs_for_qsr_namespace_over_global_namespace("data1",
                                                                            "data1_cardir_qsrs_for_qsr_namespace.txt"))

    def test_with_bounding_boxes(self):
        self.assertItemsEqual(*self.defaults("data2", "data2_cardir_defaults.txt"))

    def test_without_bounding_boxes(self):
        self.assertItemsEqual(*self.defaults("data3", "data3_cardir_defaults.txt"))

    def test_floats(self):
        self.assertItemsEqual(*self.defaults("data4", "data4_cardir_defaults.txt"))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("qsr_lib", "cardir_test", cardir_Test, sys.argv)
