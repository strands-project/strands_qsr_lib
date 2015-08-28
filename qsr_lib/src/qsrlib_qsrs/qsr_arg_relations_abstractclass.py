# -*- coding: utf-8 -*-
from __future__ import print_function, division
import abc
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_1t_Abstractclass


class QSR_Arg_Relations_Abstractclass(QSR_Dyadic_1t_Abstractclass):
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        super(QSR_Arg_Relations_Abstractclass, self).__init__()
        self.qsr_relations_and_values = None
        self.all_possible_relations = None
        self.all_possible_values = None
        self.allowed_value_types = None
        self.value_sort_key = None
        self.__qsr_params_defaults = {"qsr_relations_and_values": None}

    def __populate_possible_relations_and_values(self):
        ret_relations = []
        ret_values = []
        sorted_by_v = sorted(self.qsr_relations_and_values.items(), key=self.value_sort_key)
        for i in sorted_by_v:
            ret_relations.append(i[0])
            ret_values.append(i[1])
        return ret_relations, ret_values

    def __check_validity_of_qsr_relations_and_values(self, qsr_relations_and_values):
        if type(qsr_relations_and_values) is not dict:
            raise ValueError("qsr_relations_and_values must be a dict")
        for k, v in qsr_relations_and_values.items():
            if not isinstance(k, str) or not isinstance(v, self.allowed_value_types):
                try:
                    raise ValueError("qsr_relations_and_values must be a dict of str:%s" % '|'.join(x.__name__ for x in self.allowed_value_types))
                except TypeError:
                    raise ValueError("qsr_relations_and_values must be a dict of str:%s" % self.allowed_value_types.__name__)
        return True

    def _set_qsr_relations_and_values(self, qsr_relations_and_values):
        if self.__check_validity_of_qsr_relations_and_values(qsr_relations_and_values):
            self.qsr_relations_and_values = qsr_relations_and_values
        self.all_possible_relations, self.all_possible_values = self.__populate_possible_relations_and_values()
