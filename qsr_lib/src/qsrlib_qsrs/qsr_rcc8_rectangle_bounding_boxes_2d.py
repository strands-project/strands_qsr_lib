# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_Abstractclass
from qsrlib_io.world_qsr_trace import *


class QSR_RCC8_Rectangle_Bounding_Boxes_2D(QSR_Dyadic_Abstractclass):
    """RCC8

        # 'dc'     bb1 is disconnected from bb2
        # 'ec'     bb1 is externally connected with bb2
        # 'po'     bb1 partially overlaps bb2
        # 'eq'     bb1 equals bb2
        # 'tpp'    bb1 is a tangential proper part of bb2
        # 'ntpp'   bb1 is a non-tangential proper part of bb2
        # 'tppi'   bb2 is a tangential proper part of bb1
        # 'ntppi'  bb2 is a non-tangential proper part of bb1
    """
    def __init__(self):
        super(QSR_RCC8_Rectangle_Bounding_Boxes_2D, self).__init__()
        self._unique_id = "rcc8"
        self.all_possible_relations = ["dc", "ec", "po", "eq", "tpp", "ntpp", "tppi", "ntppi"]

    def custom_set_from_config_file(self, document):
        pass

    def custom_help(self):
        """Write your own help message function"""
        print("where,\nx1, y2: the xy-coords of the top-left corner of the rectangle\nx2, y2: the xy-coords of the bottom-right corner of the rectangle")

    # todo possibly no longer needed
    def custom_checks(self, input_data):
        """Write your own custom checks on top of the default ones


        :return: error code, error message (integer, string), use 10 and above for error code as 1-9 are reserved by system
        """
        return 0, ""

    def _process_qsr_parameters_from_request_parameters(self, req_params, **kwargs):
        return None

    def _postprocess_world_qsr_trace(self, world_qsr_trace, world_trace, world_trace_timestamps, req_params, qsr_params):
        return world_qsr_trace

    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, **kwargs):
        ret = World_QSR_Trace(qsr_type=self._unique_id)
        for t in timestamps:
            world_state = world_trace.trace[t]
            qsrs_for = self._process_qsrs_for(world_state.objects.keys(), kwargs)
            for p in qsrs_for:
                between = ",".join(p)
                bb1 = world_state.objects[p[0]].return_bounding_box_2d()
                bb2 = world_state.objects[p[1]].return_bounding_box_2d()
                ret.add_qsr(QSR(timestamp=t, between=between, qsr=self.format_qsr(self.__compute_qsr(bb1, bb2))),
                            t)
        return ret

    def __compute_qsr(self, bb1, bb2):
        """Return symmetrical RCC8 relation
            :param bb1: diagonal points coordinates of first bounding box (x1, y1, x2, y2)
            :param bb2: diagonal points coordinates of second bounding box (x1, y1, x2, y2)
            :return: an RCC8 relation from the following:
                'dc'     bb1 is disconnected from bb2
                'ec'     bb1 is externally connected with bb2
                'po'     bb1 partially overlaps bb2
                'eq'     bb1 equals bb2
                'tpp'    bb1 is a tangential proper part of bb2
                'ntpp'   bb1 is a non-tangential proper part of bb2
                'tppi'   bb2 is a tangential proper part of bb1
                'ntppi'  bb2 is a non-tangential proper part of bb1
                 +-------------+         +-------------+
                 |a            |         |c            |
                 |             |         |             |
                 |     bb1     |         |     bb2     |
                 |             |         |             |
                 |            b|         |            d|
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
