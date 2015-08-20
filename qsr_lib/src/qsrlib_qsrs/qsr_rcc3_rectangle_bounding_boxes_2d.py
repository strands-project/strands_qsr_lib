# -*- coding: utf-8 -*-
"""RCC3, based upon RCC Abstract Class
:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Author: Peter Lightbody <plightbody@lincoln.ac.uk>
:Organization: University of Leeds
:Organization: University of Lincoln
:Date: 10 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""

from __future__ import print_function, division
from qsrlib_qsrs.qsr_rcc_abstractclass import QSR_RCC_Abstractclass
from qsrlib_io.world_qsr_trace import *

class QSR_RCC3_Rectangle_Bounding_Boxes_2D(QSR_RCC_Abstractclass):
    """Make default QSRs and provide an example for others"""
    def __init__(self):
        self._unique_id = "rcc3"
        # 'dc'     bb1 is disconnected from bb2
        # 'po'     bb1 partially overlaps bb2
        # 'o'      bb1 is occluded or part of bb2
        self.all_possible_relations = ["dc", "po", "o"]

    def custom_set_from_config_file(self, document):
        pass

    def custom_help(self):
        """Write your own help message function"""
        print("where,\nx1, y2: the xy-coords of the top-left corner of the rectangle\nx2, y2: the xy-coords of the bottom-right corner of the rectangle")

    def custom_checks(self, input_data):
        """Write your own custom checks on top of the default ones
        :return: error code, error message (integer, string), use 10 and above for error code as 1-9 are reserved by system
        """
        return 0, ""

    def convert_to_current_rcc(self, qsr):
        qsr = qsr.replace("ec", "po")
        return qsr if qsr in self.all_possible_relations else self.all_possible_relations[-1]