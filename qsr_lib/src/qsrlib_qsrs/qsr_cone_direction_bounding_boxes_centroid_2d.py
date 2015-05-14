# -*- coding: utf-8 -*-
"""
:Author: Peter Lightbody <plightbody@lincoln.ac.uk>
:Organization: University of Lincoln
:Date: 12 May 2015
:Version: 0.1
:Status: Development
:Copyright: STRANDS default

"""


from __future__ import print_function, division
from qsrlib_qsrs.qsr_abstractclass import QSR_Abstractclass
from qsrlib_io.world_qsr_trace import *
import math

class QSR_Cone_Direction_Bounding_Boxes_Centroid_2D(QSR_Abstractclass):
    """Make default QSRs and provide an example for others"""
    def __init__(self):
        self.qsr_type = "cone_direction_bounding_boxes_centroid_2d"  # must be the same that goes in the QSR_Lib.__const_qsrs_available
        self.all_possible_relations = ["north", "north-east", "east", "south-east", "south", "south-west", "west", "north-west", "same", "unknown"]

    def custom_set_from_ini(self, parser):
        pass

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
        """Return cone direction relation
            :param bb1: diagonal points coordinates of first bounding box (x1, y1, x2, y2)
            :param bb2: diagonal points coordinates of second bounding box (x1, y1, x2, y2)
            :return: a string containing a cone-shaped direction relation
             directions: 'north', 'north-east', 'east', 'south-east', 'south', 'south-west', 'west', 'north-west', 'same', 'unknown'
        """
        
        # Finds the differnece between the centres of each object
        dx = ((bb2[0]+bb2[2])/2.0) - ((bb1[0]+bb1[2])/2.0)
        dy = ((bb2[1]+bb2[3])/2.0) - ((bb1[1]+bb1[3])/2.0)

        if dx==0 and dy==0:
            return 'same'

        # Calculate the angle of the line between the two objects (in degrees)
        angle = (math.atan2(dx,dy) * (180/math.pi))+22.5

        # If that angle is negative, invert it
        if angle < 0.0:
            angle = (360.0 + angle)

        # Lookup labels and return answer
        return self.directionSwitch(math.floor(((angle)/45.0)))

    # Switch Statement convert number into region label
    def directionSwitch(self,x):
        return {
            0 : 'south',
            1 : 'south-west',
            2 : 'west',
            3 : 'north-west',
            4 : 'north',
            5 : 'north-east',
            6 : 'east',
            7 : 'south-east',
        }.get((x), 'unknown')
