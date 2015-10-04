#!/usr/bin/env python
from __future__ import print_function, division
import sys
from abstractclass_unittest_triadic import Abstractclass_Unittest_Triadic


class TPCC_Test(Abstractclass_Unittest_Triadic):
    def __init__(self, *args):
        super(TPCC_Test, self).__init__(*args)
        self._unique_id = "tpcc"

    # data1 contains data for only 2 objects so the default tests has to be with data2
    # would be same as test_without_bounding_boxes in other QSRs tests
    def test_defaults(self):
        self.assertItemsEqual(*self.defaults("data3", "data3_tpcc_defaults.txt"))

    def test_qsrs_for_global_namespace(self):
        self.assertItemsEqual(*self.qsrs_for_global_namespace("data3", "data3_tpcc_qsrs_for_global_namespace.txt"))

    def test_qsrs_for_qsr_namespace(self):
        self.assertItemsEqual(*self.qsrs_for_qsr_namespace("data3", "data3_tpcc_qsrs_for_qsr_namespace.txt"))
        # precedes "for_all_qsrs" namespace
        self.assertItemsEqual(*self.qsrs_for_qsr_namespace_over_global_namespace("data3",
                                                                                 "data3_tpcc_qsrs_for_qsr_namespace.txt"))

    def test_with_bounding_boxes(self):
        self.assertItemsEqual(*self.defaults("data2", "data2_tpcc_defaults.txt"))

    def test_floats(self):
        self.assertItemsEqual(*self.defaults("data4", "data4_tpcc_defaults.txt"))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("qsr_lib", "tpcc_test", TPCC_Test, sys.argv)
