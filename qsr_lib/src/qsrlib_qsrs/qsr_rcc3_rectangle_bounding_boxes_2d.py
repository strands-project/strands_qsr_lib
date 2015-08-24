# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_Abstractclass
from qsrlib_io.world_qsr_trace import *


class QSR_RCC3_Rectangle_Bounding_Boxes_2D(QSR_Dyadic_Abstractclass):
    """Computes symmetrical RCC3 relations: 'dc':disconnected, 'po':partial overlap, 'o': occluded/part of

    """
    def __init__(self):
        super(QSR_RCC3_Rectangle_Bounding_Boxes_2D, self).__init__()
        self._unique_id = "rcc3"
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
        """Return symmetrical RCC3 relation

        :param bb1: diagonal points coordinates of first bounding box (x1, y1, x2, y2)
        :param bb2: diagonal points coordinates of second bounding box (x1, y1, x2, y2)
        :return: an RCC3 relation from the following: 'dc':disconnected, 'po':partial overlap, 'o': occluded/part of
        """

        bboxes_intercept_v, rabx, raxPrbx, raby, rayPrby = self.__bboxes_intercept(bb1, bb2)
        if bboxes_intercept_v:
            if rabx > 0.0 or raby > 0:
                return "po"
            else:
                occluded_points = self.__count_occluded_points(bb1, bb2)
                if occluded_points >= 4:
                    return "o"
                else:
                    return "po"
        else:
            return "dc"

    def __count_occluded_points(self, bb1, bb2):
        occluded_points = 0
        bb1_4corners = ((bb1[0], bb1[1]),
                        (bb1[2], bb1[1]),
                        (bb1[2], bb1[3]),
                        (bb1[0], bb1[3]))
        bb2_4corners = ((bb2[0], bb2[1]),
                        (bb2[2], bb2[1]),
                        (bb2[2], bb2[3]),
                        (bb2[0], bb2[3]))

        for p in bb1_4corners:
            if self.__is_point_in_rectangle(p, bb2):
                occluded_points += 1
        for p in bb2_4corners:
            if self.__is_point_in_rectangle(p, bb1):
                occluded_points += 1

        return occluded_points

    def __is_point_in_rectangle(self, p, r, d=0.):
        return p[0] >= r[0]-d and p[0] <= r[2]+d and p[1] >= r[0]-d and p[1] <= r[3]+d

    def __bboxes_intercept(self, bb1, bb2):
        """
        https://rbrundritt.wordpress.com/2009/10/03/determining-if-two-bounding-boxes-overlap/

        :param bb1: diagonal points coordinates of first bounding box (x1, y1, x2, y2)
        :param bb2: diagonal points coordinates of second bounding box (x1, y1, x2, y2)
        :return:
        """

        # First bounding box, top left corner, bottom right corner
        ATLx = bb1[0]
        ATLy = bb1[3]
        ABRx = bb1[2]
        ABRy = bb1[1]

        # Second bounding box, top left corner, bottom right corner
        BTLx = bb2[0]
        BTLy = bb2[3]
        BBRx = bb2[2]
        BBRy = bb2[1]

        rabx = abs(ATLx + ABRx - BTLx - BBRx)
        raby = abs(ATLy + ABRy - BTLy - BBRy)

        # rAx + rBx
        raxPrbx = ABRx - ATLx + BBRx - BTLx

        # rAy + rBy
        rayPrby = ATLy - ABRy + BTLy - BBRy

        if(rabx <= raxPrbx) and (raby <= rayPrby):
            return True, rabx, raxPrbx, raby, rayPrby
        else:
            return False, rabx, raxPrbx, raby, rayPrby
