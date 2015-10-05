# -*- coding: utf-8 -*-
from abc import abstractmethod, ABCMeta
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_1t_Abstractclass


class QSR_RCC_Abstractclass(QSR_Dyadic_1t_Abstractclass):
    """Abstract class of RCC relations.

    Values of the abstract properties
        * **_unique_id** = defined by the RCC variant.
        * **_all_possible_relations** = defined by the RCC variant.
        * **_dtype** = "bounding_boxes_2d"

    QSR specific `dynamic_args`
        * **quantisation_factor** (*float*) = 0.0: Threshold that determines whether two rectangle regions are disconnected.
    """

    __metaclass__ = ABCMeta

    _dtype = "bounding_boxes_2d"
    """str: On what kind of data the QSR works with."""

    def __init__(self):
        """Constructor."""
        super(QSR_RCC_Abstractclass, self).__init__()

        self.__qsr_params_defaults = {"quantisation_factor": 0.0}
        """float: ?"""

    def _process_qsr_parameters_from_request_parameters(self, req_params, **kwargs):
        qsr_params = self.__qsr_params_defaults.copy()
        try:
            qsr_params["quantisation_factor"] = float(req_params["dynamic_args"][self._unique_id]["quantisation_factor"])
        except (KeyError, TypeError):
            try:
                qsr_params["quantisation_factor"] = float(req_params["dynamic_args"]["for_all_qsrs"]["quantisation_factor"])
            except (TypeError, KeyError):
                pass
        return qsr_params

    def _compute_qsr(self, bb1, bb2, qsr_params, **kwargs):
        """Return symmetrical RCC8 relation
            :param bb1: diagonal points coordinates of first bounding box (x1, y1, x2, y2)
            :param bb2: diagonal points coordinates of second bounding box (x1, y1, x2, y2)
            :param q: quantisation factor for all objects
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
        q = qsr_params["quantisation_factor"]

        # CALCULATE EQ
        # Is object1 equal to object2
        if (bb1 == bb2):
            return self._convert_to_requested_rcc_type("eq")

        ax, ay, bx, by = bb1
        cx, cy, dx, dy = bb2

        # Are objects disconnected?
        # Cond1. If A's left edge is to the right of the B's right edge, - then A is Totally to right Of B
        # Cond2. If A's right edge is to the left of the B's left edge, - then A is Totally to left Of B
        # Cond3. If A's top edge is below B's bottom edge, - then A is Totally below B
        # Cond4. If A's bottom edge is above B's top edge, - then A is Totally above B

        #    Cond1           Cond2          Cond3         Cond4
        if (ax-q > dx+q) or (bx+q < cx-q) or (ay-q > dy+q) or (by+q < cy-q):
            return self._convert_to_requested_rcc_type("dc")

        # Is one object inside the other ()
        BinsideA = (ax <= cx) and (ay <= cy) and (bx >= dx) and (by >= dy)
        AinsideB = (ax >= cx) and (ay >= cy) and (bx <= dx) and (by <= dy)

        # Do objects share an X or Y (but are not necessarily touching)
        sameX = (abs(ax - cx)<=q) or (abs(ax - dx)<=q) or (abs(bx - cx)<=q) or (abs(bx - dx)<=q)
        sameY = (abs(ay - cy)<=q) or (abs(ay - dy)<=q) or (abs(by - cy)<=q) or (abs(by - dy)<=q)

        if AinsideB and (sameX or sameY):
            return self._convert_to_requested_rcc_type("tpp")

        if BinsideA and (sameX or sameY):
            return self._convert_to_requested_rcc_type("tppi")

        if AinsideB:
            return self._convert_to_requested_rcc_type("ntpp")

        if BinsideA:
            return self._convert_to_requested_rcc_type("ntppi")

        # Are objects touching?
        # Cond1. If A's left edge is equal to B's right edge, - then A is to the right of B and touching
        # Cond2. If A's right edge is qual to B's left edge, - then A is to the left of B and touching
        # Cond3. If A's top edge equal to B's bottom edge, - then A is below B and touching
        # Cond4. If A's bottom edge equal to B's top edge, - then A is above B and touching

        # If quantisation overlaps, but bounding boxes do not then edge connected,
        # include the objects edges, but do not include the quantisation edge
        if ((cx-q) <= (bx+q)) and ((cx-q) >= (bx)) or \
                        ((dx+q) >= (ax-q)) and ((dx+q) <= (ax)) or \
                        ((cy-q) <= (by+q)) and ((cy-q) >= (by)) or \
                        ((dy+q) >= (ay-q)) and ((dy+q) <= (ay)):
            return self._convert_to_requested_rcc_type("ec")

        # If none of the other conditions are met, the objects must be parially overlapping
        return self._convert_to_requested_rcc_type("po")

    @abstractmethod
    def _convert_to_requested_rcc_type(self, qsr):
        """Overwrite this function to filter and return only the relations corresponding to the particular RCC version.

        Example for RCC2: return qsr if qsr =="dc" else "c"

        :param qsr: The RCC8 relation between two objects
        :type qsr: str
        :return: The part of the tuple you would to have as a result
        :rtype: str
        """
        return
