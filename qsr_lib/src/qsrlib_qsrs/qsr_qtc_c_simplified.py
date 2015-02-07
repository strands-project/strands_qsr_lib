# -*- coding: utf-8 -*-
"""Example that shows how to implement QSR makers.

:Author: Christan Dondrup <cdondrup@lincoln.ac.uk>
:Organization: University of Lincoln
:Date: 10 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
:Notes: future extension to handle polygons, to do that use matplotlib.path.Path.contains_points
        although might want to have a read on the following also...
        http://matplotlib.1069221.n5.nabble.com/How-to-properly-use-path-Path-contains-point-td40718.html
"""

from __future__ import print_function, division
from qsrlib_qsrs.qsr_qtc_simplified_abstractclass import QSR_QTC_Simplified_Abstractclass
import math
import sys


class QSR_QTC_C_Simplified(QSR_QTC_Simplified_Abstractclass):
    """Make default QSRs and provide an example for others"""
    def __init__(self):
        super(QSR_QTC_C_Simplified, self).__init__()
        self.qtc_type = "c"
        self.qsr_type = "qtc_c_simplified"  # must be the same that goes in the QSR_Lib.__const_qsrs_available
        self.all_possible_relations = self.return_all_possible_state_combinations()[1]

    def qtc_to_string(self, qtc):
        """Overwrite this for the different QTC veriants to select only the parts
        from the QTCC tuple that you would like to return.
        Example for QTCB: return qtc[0:2]
        
        :param qtc: The full QTCC tuple [q1,q2,q4,q5]
        
        :return: q1,q2,q4,q5 converted to a comma separated string
        """
        return qtc
