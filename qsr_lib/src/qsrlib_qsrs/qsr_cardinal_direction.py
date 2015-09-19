# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_1t_Abstractclass
import math

class QSR_Cardinal_Direction(QSR_Dyadic_1t_Abstractclass):
    """Cardinal direction relations.

    Values of the abstract properties
        * **_unique_id** = "cardir"
        * **_all_possible_relations** = ("n", "ne", "e", "se", "s", "sw", "w", "nw", "eq")
        * **_dtype** = "bounding_boxes_2d"

    Some explanation about the QSR or better link to a separate webpage explaining it. Maybe a reference if it exists.
    """

    _unique_id = "cardir"
    """str: Unique identifier name of the QSR."""

    _all_possible_relations = ("n", "ne", "e", "se", "s", "sw", "w", "nw", "eq")
    """tuple: All possible relations of the QSR."""

    _dtype = "bounding_boxes_2d"
    """str: On what kind of data the QSR works with."""

    def __init__(self):
        """Constructor."""
        super(QSR_Cardinal_Direction, self).__init__()

    def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
        """Compute QSR relation.

        :param data1: Bounding box.
        :type data1: list or tuple of int or floats
        :param data2: Bounding box.
        :type data2: list or tuple of int or floats
        :return: QSR relation.
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
        :type x:
        :return: QSR relation.
        :rtype: str
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
