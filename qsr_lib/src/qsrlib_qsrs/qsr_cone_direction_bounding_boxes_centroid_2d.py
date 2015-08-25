# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_Abstractclass
from qsrlib_io.world_qsr_trace import *
import math

class QSR_Cone_Direction_Bounding_Boxes_Centroid_2D(QSR_Dyadic_Abstractclass):
    """Cone direction relations

        # 's'      south
        # 'sw'     south-west
        # 'w'      west
        # 'nw'     north-west
        # 'n'      north
        # 'ne'     north-east
        # 'e'      east
        # 'se'     south-east

        where,\nx1, y2: the xy-coords of the top-left corner of the rectangle\nx2, y2: the xy-coords of the bottom-right corner of the rectangle
    """
    def __init__(self):
        super(QSR_Cone_Direction_Bounding_Boxes_Centroid_2D, self).__init__()
        self._unique_id = "coneDir"
        self.all_possible_relations = ["n", "ne", "e", "se", "s", "sw", "w", "nw", "eq"]

    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, req_params, **kwargs):
        ret = World_QSR_Trace(qsr_type=self._unique_id)
        for t in timestamps:
            world_state = world_trace.trace[t]
            qsrs_for = self._process_qsrs_for(world_state.objects.keys(), req_params["dynamic_args"])
            for p in qsrs_for:
                between = ",".join(p)
                bb1 = world_state.objects[p[0]].return_bounding_box_2d()
                bb2 = world_state.objects[p[1]].return_bounding_box_2d()
                ret.add_qsr(QSR(timestamp=t, between=between, qsr=self._format_qsr(self.__compute_qsr(bb1, bb2))),
                            t)
        return ret

    def __compute_qsr(self, bb1, bb2):
        """Return cone direction relation
            :param bb1: diagonal points coordinates of first bounding box (x1, y1, x2, y2)
            :param bb2: diagonal points coordinates of second bounding box (x1, y1, x2, y2)
            :return: a string containing a cone-shaped direction relation
             directions: 'n', 'ne', 'e', 'se', 's', 'sw', 'w', 'nw', 'eq'
        """
        
        # Finds the differnece between the centres of each object
        dx = ((bb2[0]+bb2[2])/2.0) - ((bb1[0]+bb1[2])/2.0)
        dy = ((bb2[1]+bb2[3])/2.0) - ((bb1[1]+bb1[3])/2.0)

        if dx==0 and dy==0:
            return 'eq'

        # Calculate the angle of the line between the two objects (in degrees)
        angle = (math.atan2(dx,dy) * (180/math.pi))+22.5

        # If that angle is negative, invert it
        if angle < 0.0:
            angle = (360.0 + angle)

        # Lookup labels and return answer
        return self.__direction_switch(math.floor(((angle)/45.0)))

    # Switch Statement convert number into region label
    def __direction_switch(self,x):
        return {
            #    south
            0 : 's',
            #    south-west
            1 : 'sw',
            #    west
            2 : 'w',
            #    north-west
            3 : 'nw',
            #    north
            4 : 'n',
            #    north-east
            5 : 'ne',
            #    east
            6 : 'e',
            #    south-east
            7 : 'se',
        }.get(x)
