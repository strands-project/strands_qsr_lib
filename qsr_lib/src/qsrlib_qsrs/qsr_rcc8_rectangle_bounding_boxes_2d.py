# -*- coding: utf-8 -*-
"""Example that shows how to implement QSR makers.

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 10 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
:Notes: future extension to handle polygons, to do that use matplotlib.path.Path.contains_points
        although might want to have a read on the following also...
        http://matplotlib.1069221.n5.nabble.com/How-to-properly-use-path-Path-contains-point-td40718.html
"""

from __future__ import print_function, division
from qsrlib_qsrs.qsr_abstractclass import QSR_Abstractclass
from qsrlib_io.world_qsr_trace import *


class QSR_RCC8_Rectangle_Bounding_Boxes_2D(QSR_Abstractclass):
    """Make default QSRs and provide an example for others"""
    def __init__(self):
        self.qsr_type = "rcc8_rectangle_bounding_boxes_2d"  # must be the same that goes in the QSR_Lib.__const_qsrs_available
#         {0:'dc'}      x is disconnected from y
#         {1:'ec'}      x is externally connected with y
#         {2:'po'}      x partially overlaps y
#         {3:'eq'}      x equals y
#         {4:'tpp'}     x is a tangential proper part of y
#         {5:'ntpp'}    y is a non-tangential proper part of x
#         {6:'tppi'}    y is a tangential proper part of x
#         {7:'ntppi'}   y is a non-tangential proper part of x
        self.all_possible_relations = ["dc", "ec", "po", "eq", "tpp", "ntpp", "tppi", "ntppi"]

    def custom_help(self):
        """Write your own help message function"""
        print("where,\nx1, y2: the xy-coords of the top-left corner of the rectangle\nx2, y2: the xy-coords of the bottom-right corner of the rectangle")

    def custom_checks(self, input_data):
        """Write your own custom checks on top of the default ones


        :return: error code, error message (integer, string), use 10 and above for error code as 1-9 are reserved by system
        """
        return 0, ""

    def custom_checks_for_qsrs_for(self, qsrs_for, error_found):
        """qsrs_for must be tuples of two objects.

        :param qsrs_for: list of strings and/or tuples for which QSRs will be computed
        :param error_found: if an error was found in the qsrs_for that violates the QSR rules
        :return: qsrs_for, error_found
        """
        for p in list(qsrs_for):
            if (type(p) is not tuple) and (type(p) is not list) and (len(p) != 2):
                qsrs_for.remove(p)
                error_found = True
        return qsrs_for, error_found

    def make(self, *args, **kwargs):
        """Make the QSRs

        :param args: not used at the moment
        :param kwargs:
                        - input_data: World_Trace
        :return: World_QSR_Trace
        """
        input_data = kwargs["input_data"]
        include_missing_data = kwargs["include_missing_data"]
        ret = World_QSR_Trace(qsr_type=self.qsr_type)
        for t in input_data.get_sorted_timestamps():
            world_state = input_data.trace[t]
            timestamp = world_state.timestamp
            if kwargs["qsrs_for"]:
                qsrs_for, error_found = self.check_qsrs_for_data_exist(world_state.objects.keys(), kwargs["qsrs_for"])
            else:
                qsrs_for = self.__return_all_possible_combinations(world_state.objects.keys())
            if qsrs_for:
                for p in qsrs_for:
                    between = str(p[0]) + "," + str(p[1])
                    bb1 = world_state.objects[p[0]].return_bounding_box_2d()
                    bb2 = world_state.objects[p[1]].return_bounding_box_2d()
                    qsr = QSR(timestamp=timestamp, between=between, qsr=self.__compute_qsr(bb1, bb2))
                    ret.add_qsr(qsr, timestamp)
            else:
                if include_missing_data:
                    ret.add_empty_world_qsr_state(timestamp)
        return ret

    # custom functions follow
    def __return_all_possible_combinations(self, objects_names):
        if len(objects_names) < 2:
            return []
        ret = []
        for i in objects_names:
            for j in objects_names:
                if i != j:
                    ret.append((i, j))
        return ret

    def __compute_qsr(self, bb1, bb2):
        """Return symmetrical RCC3 relation
            :param bb1: diagonal points coordinates of first bounding box (x1, y1, x2, y2)
            :param bb2: diagonal points coordinates of second bounding box (x1, y1, x2, y2)
            :return: an RCC8 relation from the following:
             {0:'dc'}      x is disconnected from y
             {1:'ec'}      x is externally connected with y
             {2:'po'}      x partially overlaps y
             {3:'eq'}      x equals y
             {4:'tpp'}     x is a tangential proper part of y
             {5:'ntpp'}    y is a non-tangential proper part of x
             {6:'tppi'}    y is a tangential proper part of x
             {7:'ntppi'}   y is a non-tangential proper part of x
             +-------------+         +-------------+
             |a            |         |a            |
             |             |         |             |
             |      A      |         |      B      |
             |             |         |             |
             |            b|         |            b|
             +-------------+         +-------------+  
        """
        
        # Object 1 Top Left X
        ax = bb1[0]
        # Object 1 Top Left Y
        ay = bb1[1]
        # Object 2 Top Left X
        cx = bb2[0]
        # Object 2 Top Left X
        cy = bb2[1]
        # Object 1 Bottom Right X
        bx = bb1[2]
        # Object 1 Bottom Right Y
        by = bb1[3]
        # Object 2 Bottom Right X
        dx = bb2[2]
        # Object 2 Bottom Right Y
        dy = bb2[3]
    
        # CALCULATE EQ
        # Is object1 equal to object2
        if(bb1 == bb2):
            return "eq"
    
        # Are objects disconnected?
        # Cond1. If A's left edge is to the right of the B's right edge, - then A is Totally to right Of B
        # Cond2. If A's right edge is to the left of the B's left edge, - then A is Totally to left Of B
        # Cond3. If A's top edge is below B's bottom edge, - then A is Totally below B
        # Cond4. If A's bottom edge is above B's top edge, - then A is Totally above B
        
        #    Cond1        Cond2        Cond3        Cond4
        if (ax > dx) or (bx < cx) or (ay > dy) or (by < cy):
            return "dc"
    
        # Is one object inside the other
        BinsideA = (ax <= cx) and (ay <= cy) and (bx >= dx) and (by >= dy)
        AinsideB = (ax >= cx) and (ay >= cy) and (bx <= dx) and (by <= dy)
    
        # Do objects share an X or Y (but are not necessarily touching)
        sameX = (ax == cx or ax == dx or bx == cx or bx == dx)
        sameY = (ay == cy or ay == dy or by == cy or by == dy)
    
        if AinsideB and (sameX or sameY):
            return "tpp"
    
        if BinsideA and (sameX or sameY):
            return "tppi"
    
        if AinsideB:
            return "ntpp"
    
        if BinsideA:
            return "ntppi"
    
        # Are objects touching?
        # Cond1. If A's left edge is equal to B's right edge, - then A is to the right of B and touching
        # Cond2. If A's right edge is qual to B's left edge, - then A is to the left of B and touching
        # Cond3. If A's top edge equal to B's bottom edge, - then A is below B and touching
        # Cond4. If A's bottom edge equal to B's top edge, - then A is above B and touching
    
        #    Cond1        Cond2        Cond3        Cond4
        if (ax == dx) or (bx == cx) or (ay == dy) or (by == cy):
            return "ec"
    
        # If none of the other conditions are met, the objects must be parially overlapping
        return "po"