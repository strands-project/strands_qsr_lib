#!/usr/bin/env python
from __future__ import print_function, division
import sys
from abstractclass_unittest_dyadic import Abstractclass_Unittest_Dyadic


class QTCBS_Test(Abstractclass_Unittest_Dyadic):
    def __init__(self, *args):
        super(QTCBS_Test, self).__init__(*args)
        self._unique_id = "qtcbs"
        self.__params = {"quantisation_factor": 2.0}

    def test_defaults(self):
        self.assertItemsEqual(*self.defaults("data1", "data1_qtcbs_defaults.txt"))
        self.assertItemsEqual(*self.defaults("data2", "data2_qtcbs_defaults.txt"))
        self.assertItemsEqual(*self.defaults("data3", "data3_qtcbs_defaults.txt"))
        self.assertItemsEqual(*self.defaults("data4", "data4_qtcbs_defaults.txt"))

    def test_qsrs_for_global_namespace(self):
        self.assertItemsEqual(*self.qsrs_for_global_namespace("data1", "data1_qtcbs_qsrs_for_global_namespace.txt"))

    def test_qsrs_for_qsr_namespace(self):
        self.assertItemsEqual(*self.qsrs_for_qsr_namespace("data1", "data1_qtcbs_qsrs_for_qsr_namespace.txt"))
        # precedes "for_all_qsrs" namespace
        self.assertItemsEqual(*self.qsrs_for_qsr_namespace_over_global_namespace("data1",
                                                                                 "data1_qtcbs_qsrs_for_qsr_namespace.txt"))

    def test_q_factor(self):
        self.assertItemsEqual(*self.q_factor("data1", "data1_qtcbs_q_factor_2p0.txt",
                                             self.__params["quantisation_factor"]))
        q_factor_results, defaults_results = self.q_factor_data_notequal_defaults("data1_qtcbs_q_factor_2p0.txt",
                                                                                  "data1_qtcbs_defaults.txt")
        self.assertFalse(q_factor_results == defaults_results)

    def test_parameters_combinations(self):
        dynamic_args = {self._unique_id: {"validate": True, "no_collapse": True}}
        self.assertItemsEqual(*self.custom("data1", "data1_qtcbs_validate_nocollapse.txt", dynamic_args))
        dynamic_args = {self._unique_id: {"validate": True, "no_collapse": False}}
        self.assertItemsEqual(*self.custom("data1", "data1_qtcbs_validate_nonocollapse.txt", dynamic_args))
        dynamic_args = {self._unique_id: {"validate": False, "no_collapse": True}}
        self.assertItemsEqual(*self.custom("data1", "data1_qtcbs_novalidate_nocollapse.txt", dynamic_args))
        dynamic_args = {self._unique_id: {"validate": False, "no_collapse": False}}
        self.assertItemsEqual(*self.custom("data1", "data1_qtcbs_novalidate_nonocollapse.txt", dynamic_args))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("qsr_lib", "qtcbs_test", QTCBS_Test, sys.argv)
