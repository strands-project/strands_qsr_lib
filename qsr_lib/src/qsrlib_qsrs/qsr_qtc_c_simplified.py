# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_qtc_simplified_abstractclass import QSR_QTC_Simplified_Abstractclass


class QSR_QTC_C_Simplified(QSR_QTC_Simplified_Abstractclass):
    """QTCB simplified relations.

    Values of the abstract properties
        * **_unique_id** = "qtccs"
        * **_all_possible_relations** = ?
        * **_dtype** = "points"

    Some explanation about the QSR or better link to a separate webpage explaining it. Maybe a reference if it exists.
    """

    def __init__(self):
        """Constructor"""
        super(QSR_QTC_C_Simplified, self).__init__()

        self._unique_id = "qtccs"
        """str: Unique identifier name of the QSR."""

        self.qtc_type = "c"
        """str: QTC specific type."""

        self._all_possible_relations = tuple(self.return_all_possible_state_combinations()[0])
        """tuple: All possible relations of the QSR."""

    def qtc_to_output_format(self, qtc):
        """Return QTCCS.

        :param qtc: Full QTCC tuple [q1,q2,q4,q5].
        :type qtc: list or tuple
        :return: {"qtccs": "q1,q2,q4,q5"}
        :rtype: dict
        """
        return self._format_qsr(self.create_qtc_string(qtc))
