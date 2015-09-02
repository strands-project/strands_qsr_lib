#!/usr/bin/env python
from __future__ import print_function, division
import sys
from abstractclass_unittest_data1_dyadic import Abstractclass_Unittest_Data1_Dyadic
from tests_utils import *


class MWE_Test(Abstractclass_Unittest_Data1_Dyadic):
    def __init__(self, *args):
        super(MWE_Test, self).__init__(*args)
        self._unique_id = "mwe"
        self._world = load_input_data1()

    def test_defaults(self):
        self.assertEqual(*self.defaults('data1_mwe_defaults.txt'))

    def test_qsrs_for_global_namespace(self):
        self.assertEqual(*self.qsrs_for_global_namespace('data1_mwe_qsrs_for_global_namespace.txt'))

    def test_qsrs_for_qsr_namespace(self):
        self.assertEqual(*self.qsrs_for_qsr_namespace('data1_mwe_qsrs_for_qsr_namespace.txt'))
        self.assertEqual(*self.qsrs_for_qsr_namespace_over_global_namespace('data1_mwe_qsrs_for_qsr_namespace.txt'))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("qsr_lib", "mwe_test", MWE_Test, sys.argv)
