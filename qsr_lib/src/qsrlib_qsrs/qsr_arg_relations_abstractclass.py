# -*- coding: utf-8 -*-
from __future__ import print_function, division
import abc
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_1t_Abstractclass


class QSR_Arg_Relations_Abstractclass(QSR_Dyadic_1t_Abstractclass):
    """Abstract class of argument relations."""

    __metaclass__ = abc.ABCMeta

    def __init__(self):
        """Constructor"""
        # todo seems to me like some of the members could be re-organised as properties, etc.
        super(QSR_Arg_Relations_Abstractclass, self).__init__()

        self.qsr_relations_and_values = None # todo private abstractproperty maybe?
        """dict: Holds the passed `qsr_relations_and_values` dict in `dynamic_args`."""

        self._all_possible_relations = None
        """tuple: All possible relations, basically alphabetically sorted list of the keys of `qsr_relations_and_values`."""

        self.all_possible_values = None
        """tuple: List of distance thresholds from `qsr_relations_and_values`, corresponding to the order of `self._all_possible_relations`."""

        self.allowed_value_types = None  # todo private abstractproperty maybe?
        """tuple: Allowed types of the thresholds."""

        # todo should be private abstractproperty
        self.value_sort_key = None
        """type depends on implementation: The method that the QSR labels are sorted based on their values."""

        self.__qsr_params_defaults = {"qsr_relations_and_values": None}
        """dict: Default values of the QSR parameters."""

    def __populate_possible_relations_and_values(self):
        """Populate the internal variables from `qsr_relations_and_values` passed in `dynamic_args`.

        :return: Relations labels, Relation thresholds. Sorted according to `self.value_sort_key`.
        :rtype: tuple, tuple
        """
        ret_relations = []
        ret_values = []
        sorted_by_v = sorted(self.qsr_relations_and_values.items(), key=self.value_sort_key)
        for i in sorted_by_v:
            ret_relations.append(i[0])
            ret_values.append(i[1])
        return ret_relations, ret_values

    def __check_validity_of_qsr_relations_and_values(self, qsr_relations_and_values):
        """Check that there are no type/value errors in the passed parameters.

        :param qsr_relations_and_values: User specified relation labels and values.
        :type qsr_relations_and_values: dict
        :return: True if all good, else it should have already raised an exception.
        :rtype: bool
        """
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
        """Validate and set the internal variables from `qsr_relations_and_values` passed in `dynamic_args`.

        :param qsr_relations_and_values: Holds the labels and corresponding values as passed in `dynamic_args`.
        :type qsr_relations_and_values: dict
        """
        if self.__check_validity_of_qsr_relations_and_values(qsr_relations_and_values):
            self.qsr_relations_and_values = qsr_relations_and_values
        self._all_possible_relations, self.all_possible_values = self.__populate_possible_relations_and_values()
