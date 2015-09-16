# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_1t_Abstractclass
import math

class QSR_Cardinal_Direction_Bounding_Boxes_Centroid_2D(QSR_Dyadic_1t_Abstractclass):
    """Cardinal direction relations
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

    _unique_id = "cardir"
    _all_possible_relations = ("n", "ne", "e", "se", "s", "sw", "w", "nw", "eq")
    _dtype = "bounding_boxes_2d"

    def __init__(self):
        super(QSR_Cardinal_Direction_Bounding_Boxes_Centroid_2D, self).__init__()

    def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
        """Cardinal direction relation

        :param data1: diagonal points coordinates of first bounding box (x1, y1, x2, y2)
        :param data2: diagonal points coordinates of second bounding box (x1, y1, x2, y2)
        :return: cardinal direction relation: 'n', 'ne', 'e', 'se', 's', 'sw', 'w', 'nw', 'eq'
        :rtype: str
        """
        # Finds the differnece between the centres of each object
        dx = ((data2[0]+data2[2])/2.0) - ((data1[0]+data1[2])/2.0)
        dy = ((data2[1]+data2[3])/2.0) - ((data1[1]+data1[3])/2.0)

        if dx==0 and dy==0:
            return 'eq'

        # Calculate the angle of the line between the two objects (in degrees)
        angle = (math.atan2(dx,dy) * (180/math.pi))+22.5

        # If that angle is negative, invert it
        if angle < 0.0:
            angle = (360.0 + angle)

        # Lookup labels and return answer
        return self.__direction_switch(math.floor(((angle)/45.0)))

    def __direction_switch(self, x):
        """Switch Statement convert number into region label.

        :param x:
        :return:
        """
        return {
            0: 's',
            1: 'sw',
            2: 'w',
            3: 'nw',
            4: 'n',
            5: 'ne',
            6: 'e',
            7: 'se',
        }.get(x)
