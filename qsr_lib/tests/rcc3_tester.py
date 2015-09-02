#!/usr/bin/env python
from __future__ import print_function, division
import sys
from abstractclass_unittest_data1_dyadic import Abstractclass_Unittest_Data1_Dyadic
from tests_utils import *


class RCC3_Test(Abstractclass_Unittest_Data1_Dyadic):
    def __init__(self, *args):
        super(RCC3_Test, self).__init__(*args)
        self._unique_id = "rcc3"
        self._world = load_input_data1()
        self.__params = {"quantisation_factor": 2.0}
        self.__custom = {self._unique_id: {"qsrs_for": [("o1", "o2")],
                                           "quantisation_factor": 2.0}}

    def test_defaults(self):
        self.assertEqual(*self.defaults('data1_rcc3_defaults.txt'))

    def test_qsrs_for_global_namespace(self):
        self.assertEqual(*self.qsrs_for_global_namespace('data1_rcc3_qsrs_for_global_namespace.txt'))

    def test_qsrs_for_qsr_namespace(self):
        self.assertEqual(*self.qsrs_for_qsr_namespace('data1_rcc3_qsrs_for_qsr_namespace.txt'))
        # precedes 'for_all_qsrs' namespace
        self.assertEqual(*self.qsrs_for_qsr_namespace_over_global_namespace('data1_rcc3_qsrs_for_qsr_namespace.txt'))

    def test_q_factor(self):
        self.assertEqual(*self.q_factor("data1_rcc3_q_factor_2p0.txt", self.__params["quantisation_factor"]))

    def test_custom(self):
        self.assertEqual(*self.custom("data1_rcc3_custom.txt", self.__custom))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("qsr_lib", "rcc3_test", RCC3_Test, sys.argv)
