# -*- coding: utf-8 -*-
"""Abstract class for dynamic categorical QSRs

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
"""

from __future__ import print_function, division
import abc
import operator
from qsrlib_qsrs.qsr_abstractclass import QSR_Abstractclass
# from qsrlib_io.world_qsr_trace import *


class QSR_Arg_Relations_Abstractclass(QSR_Abstractclass):
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.qsr_relations_and_values = None
        self.all_possible_relations = None
        self.all_possible_values = None
        self.qsr_keys = "argd"

    def __populate_possible_relations_and_values(self):
        ret_relations = []
        ret_values = []
        sorted_by_v = sorted(self.qsr_relations_and_values.items(), key=operator.itemgetter(1))
        for i in sorted_by_v:
            ret_relations.append(i[0])
            ret_values.append(i[1])
        return ret_relations, ret_values

    def __check_validity_of_qsr_relations_and_values(self, qsr_relations_and_values):
        if type(qsr_relations_and_values) is not dict:
            raise ValueError("qsr_relations_and_values must be a dict")
        for k, v in qsr_relations_and_values.items():
            if (type(k) is not str) or ((type(v) is not float) and (type(v) is not int)):
                raise ValueError("qsr_relations_and_values must be a dict of str:float|int")
        return True

    def set_qsr_relations_and_values(self, qsr_relations_and_values):
        if self.__check_validity_of_qsr_relations_and_values(qsr_relations_and_values):
            self.qsr_relations_and_values = qsr_relations_and_values
        self.all_possible_relations, self.all_possible_values = self.__populate_possible_relations_and_values()
