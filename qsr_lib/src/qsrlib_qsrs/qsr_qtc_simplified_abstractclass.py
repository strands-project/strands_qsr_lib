#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 19 11:22:16 2015

@author: cdondrup
"""
from abc import abstractmethod, ABCMeta
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_Abstractclass
from qsrlib_io.world_qsr_trace import *
from exceptions import Exception, AttributeError
import numpy as np


class QTCException(Exception):
    pass


class QSR_QTC_Simplified_Abstractclass(QSR_Dyadic_Abstractclass):
    """

    print "where,\n" \
    "it is always necessary to have two agents in every timestep:\n"\
    "x, y: the xy-coords of the agents\n" \
    "quantisation_factor: the minimum distance the agents must diverge from the double cross between two timesteps to be counted as movement. Must be in the same unit as the x,y coordinates.\n"\
    "validate: True|False validates the QTC sequence to not have illegal transitions. This inserts necessary transitional steps and messes with the timesteps."
    """
    __metaclass__ = ABCMeta

    __global_unique_id = "qtcs"
    __no_state__ = 9.

    _unique_id = ""
    _all_possible_relations = ()
    _dtype = "points"

    def __init__(self):
        super(QSR_QTC_Simplified_Abstractclass, self).__init__()
        self.qtc_type = ""

        self.__qsr_params_defaults= {
            "quantisation_factor": 0.0,
            "validate": True,
            "no_collapse": False,
            "distance_threshold": 1.22
        }

    def return_all_possible_state_combinations(self):
        """Method that returns all possible state combinations for the qtc_type
        defined for this calss instance.

        :return:
            - String representation as a list of possible tuples
            - Integer representation as a list of lists of possible tuples
        """
        ret_str = []
        ret_int = []
        try:
            if self.qtc_type == "":
                raise AttributeError()
            elif self.qtc_type == 'b':
                for i in xrange(1, 4):
                    for j in xrange(1, 4):
                        ret_int.append([i-2, j-2])
                        ret_str.append(str(i-2) + "," + str(j-2))
            elif self.qtc_type is 'c':
                for i in xrange(1, 4):
                    for j in xrange(1, 4):
                        for k in xrange(1, 4):
                            for l in xrange(1, 4):
                                ret_int.append([i-2, j-2, k-2, l-2])
                                ret_str.append(str(i-2) + "," + str(j-2) + "," + str(k-2) + "," + str(l-2))
            elif self.qtc_type is 'bc':
                for i in xrange(1, 4):
                    for j in xrange(1, 4):
                        ret_int.append([i-2, j-2, np.NaN, np.NaN])
                        ret_str.append(str(i-2) + "," + str(j-2))
                for i in xrange(1, 4):
                    for j in xrange(1, 4):
                        for k in xrange(1, 4):
                            for l in xrange(1, 4):
                                ret_int.append([i-2, j-2, k-2, l-2])
                                ret_str.append(str(i-2) + "," + str(j-2) + "," + str(k-2) + "," + str(l-2))
        except AttributeError:
            raise QTCException("Please define a qtc type using self.qtc_type.")
            return None, None
        return [s.replace('-1','-').replace('1','+') for s in ret_str], ret_int

    def _validate_qtc_sequence(self, qtc):
        """Removes illegal state transition by inserting necessary intermediate states

        :param qtc: a numpy array of the qtc state chain. One qtc state per row

        :return: The valid state chain as a numpy array
        """
        if len(qtc.shape) == 1:
            return np.array(qtc)  # Only one state in chain.

        if self.qtc_type == "b":
            qtc = qtc[:,0:2]

        legal_qtc = np.array([qtc[0,:]])

        for i in xrange(1, qtc.shape[0]):
            insert = np.array(qtc[i,:].copy())
            ###################################################################
            # self transition = 0, transition form - to + and vice versa = 2
            # transitions from - to 0 or + to 0 and vice versa = 1
            insert[np.abs(qtc[i-1,:]-insert)>1] = 0

            ###################################################################
            # find invalid transitions according to CND:
            # 1,2: -000 <> 0-00 | +000 <> 0+00 | -000 <> 0+00 | +000 <> 0-00
            # 1,3: -000 <> 00-0 | +000 <> 00+0 | -000 <> 00+0 | +000 <> 00-0
            # 1,4: -000 <> 000- | +000 <> 000+ | -000 <> 000+ | +000 <> 000-
            # 2,3: 0-00 <> 00-0 | 0+00 <> 00+0 | 0-00 <> 00+0 | 0+00 <> 00-0
            # 2,4: 0-00 <> 000- | 0+00 <> 000+ | 0-00 <> 000+ | 0+00 <> 000-
            # 3,4: 00-0 <> 000- | 00+0 <> 000+ | 00-0 <> 000+ | 00+0 <> 000-
            for j1 in xrange(0, len(qtc[i,:])-1):
                for j2 in xrange(j1+1, len(insert)):
                    if np.sum(np.abs(qtc[i-1,[j1,j2]])) == 1 \
                            and np.sum(np.abs(insert[[j1,j2]])) == 1:
                        if np.nanmax(np.abs(qtc[i-1,[j1,j2]] - insert[[j1,j2]])) > 0 \
                                and np.sum(qtc[i-1,[j1,j2]] - insert[[j1,j2]]) != 1:
                            insert[[j1,j2]] = 0

            #print insert

            if not np.array_equal(insert[np.logical_not(np.isnan(insert))], qtc[i, np.logical_not(np.isnan(insert))]):
                legal_qtc = np.append(legal_qtc, [insert], axis=0)

            legal_qtc = np.append(legal_qtc, [qtc[i,:]], axis=0)

        return legal_qtc

    def _collapse_similar_states(self, qtc):
        """Collapses similar adjacent QTC states.

        :param qtc: a qtc state sequence

        :return: the input sequence without similar adjacent states
        """
        if len(qtc.shape) == 1:
            return qtc  # Only one state in chain.

        if self.qtc_type == "b":
            qtc = qtc[:,0:2].copy()

        # The nan handling is a bit hacky but fast and easy
        if qtc.dtype == np.float64: qtc[np.isnan(qtc)]=self.__no_state__
        qtc = qtc[np.concatenate(([True],np.any(qtc[1:]!=qtc[:-1],axis=1)))]
        if qtc.dtype == np.float64: qtc[qtc==self.__no_state__] = np.nan
        return qtc

    def _nan_equal(self, a, b):
        """Uses assert equal to compare if two arrays containing nan values
        are equal.

        :param a: first array
        :param b: second array

        :return: True|False
        """
        try:
            np.testing.assert_equal(a,b)
        except AssertionError:
            return False
        return True

    def _create_qtc_representation(self, pos_k, pos_l, quantisation_factor=0):
        """Creating the QTCC representation for the given data. Uses the
        double cross to determine to which side of the lines the points are
        moving.

        :param pos_k: An array of positions for agent k, exactly 2 entries of x,y positions
        :param pos_l: An array of positions for agent l, exactly 2 entries of x,y positions
        :param quantisation_factor: The minimum distance the points have to diverge from either line to be regrded a non-0-state

        :return: The QTCC 4-tuple (q1,q2,q4,q5) for the movement of the two agents: [k[0],l[0],k[1],l[1]]

        """
        #print "######################################################"
        pos_k = np.array(pos_k).reshape(-1, 2)
        pos_l = np.array(pos_l).reshape(-1, 2)

        # Creating double cross, RL_ext being the connecting line, trans_RL_k
        # and l being the orthogonal lines going through k and l respectively.
        RL_ext = np.append(
            self._translate(pos_k[-2], (pos_k[-2]-pos_l[-2])/2),
            self._translate(pos_l[-2], (pos_l[-2]-pos_k[-2])/2)
        ).reshape(-1,2)
        #print "RL_ext", RL_ext
        rot_RL = self._orthogonal_line(
            pos_k[-2],
            np.append(pos_k[-2], (pos_l[-2]-pos_k[-2]))
        ).reshape(-1,2)
        #print "rot_RL", rot_RL
        trans_RL_k = self._translate(
            [rot_RL[0], rot_RL[1]],
            (rot_RL[0]-rot_RL[1])/2
        )
        #print "transk", trans_RL_k
        trans_RL_l = self._translate(
            trans_RL_k[0:2],
            (pos_l[-2]-pos_k[-2])
        )
        #print "transl", trans_RL_l

        # Test constraints for k
        k = np.append(
            self._test_constraint(
                pos_k,
                trans_RL_k,
                quantisation_factor=quantisation_factor),
            self._test_constraint(
                pos_k,
                RL_ext,
                quantisation_factor=quantisation_factor,
                constraint="side")
        )
        #print "k", k

        # Test constraints for l
        l = np.append(
            self._test_constraint(
                pos_l,
                np.array([ # Needs to be turned around to determine correct side
                           [trans_RL_l[1,0],trans_RL_l[1,1]],
                           [trans_RL_l[0,0],trans_RL_l[0,1]]
                           ]),
                quantisation_factor=quantisation_factor),
            self._test_constraint(
                pos_l,
                np.array([ # Needs to be turned around to determine correct side
                           [RL_ext[1,0],RL_ext[1,1]],
                           [RL_ext[0,0],RL_ext[0,1]]
                           ]),
                quantisation_factor=quantisation_factor,
                constraint="side")
        )
        #print "l", l

        return np.array([k[0],l[0],k[1],l[1]])

    def _translate(self, point, trans_vec):
        """Translating points by trans_vec.

        :parma points: Can be [x,y] or list of lists [[x,y],[x,y]]
        :param trans_vec: The translation vector given as [x, y]

        :return: The translated points
        """
        point_array = np.array(point)
        point_array = point_array + np.array(trans_vec)
        return point_array

    def _orthogonal_line(self, point, line):
        """Returns the line orthogonal to the line LINE and going through the
        point given by POINT. Directed angle from LINE to PERP is pi/2.

        :param point: The point the line should pass thorugh represented by [xp yp]
        :param line: The original to which the new one will be orthogonal given as [x0 y0 dx dy]

        :return: the orthoginal line to *line* going through *point*
        """
        res = np.array(point)
        line = np.array(line)
        #print "Orth", res, line

        res = np.append(res, -1*line[3])
        res = np.append(res, line[2])

        res[2] = res[0]+res[2]
        res[3] = res[1]+res[3]

        return res

    def _test_constraint(self, pos, line, quantisation_factor=0, constraint=""):
        """Testing for distance and side constraint using the double cross. To
        determine if a point moved, it uses a line (from the cross) and checks
        the side the point moved to (depending on the orientation of the line)
        and how far away it moved. The latter is used for the quantisation_factor
        and to create 0-states from noisy data.
        Example:
            - QTCB: uses the orthogonal line going through the point to compute the distance constraint
            - QTCC: uses the connecting line to determine the side constraint

        :param pos: The position of the agent, point in space
        :param line: The line to use as reference for the movement
        :param quantisation_factor=0: Minimum distance a point has to move to be considered a non-0-state. same unit of mesuarment in which the points are described
        :param constraint="": Set to "side" if checking for the side constraint.
        The result for the side has to be inverted.

        :return: The qtc symbol for this movement

        """

        x0 = pos[-1,0]
        y0 = pos[-1,1]
        x1 = line[0,0]
        y1 = line[0,1]
        x2 = line[1,0]
        y2 = line[1,1]
        test = (x0 - x1) * (y2 - y1) - (x2 - x1) * (y0 - y1)

        d = np.abs(
            np.linalg.det(
                np.append(
                    [line[1,0:2]-line[0,0:2]],
                    [pos[-1,0:2]-line[0,0:2]]
                ).reshape(-1,2)
            )
        )/np.linalg.norm(line[1,0:2]-line[0,0:2])

        res = 0
        if test > 0 and np.abs(d) > quantisation_factor:
            res = -1
        elif test < 0 and np.abs(d) > quantisation_factor:
            res = 1

        # Side constraints need to be inverted to give the correct qtc state
        return res*-1 if constraint == "side" else res

    def _custom_checks_world_trace(self, input_data, qsr_params):
        """Write your own custom checks on top of the default ones and raise exceptions as necessary.

        :return: False for no problems.
        :rtype: bool
        """
        timestamps = input_data.get_sorted_timestamps()
        if len(timestamps) < 2:
            raise ValueError("Data for at least two separate timesteps has to be provided.")
        if self._unique_id != "qtcbs" or qsr_params["validate"] or not qsr_params["no_collapse"]:
            objects_names = sorted(input_data.trace[timestamps[0]].objects.keys())
            for t in timestamps:
                for o in objects_names:
                    try:
                        input_data.trace[t].objects[o]
                    except KeyError:
                        raise KeyError("Only one object defined for timestep %f. Two objects have to be present at any given step." % t)
                    if np.isnan(input_data.trace[t].objects[o].x) or np.isnan(input_data.trace[t].objects[o].y):
                        raise ValueError("Coordinates x: %f, y: %f are not defined correctly for timestep %f." % (input_data.trace[t].objects[o].x, input_data.trace[t].objects[o].y, t))
        return False

    def _process_qsr_parameters_from_request_parameters(self, req_params, **kwargs):
        qsr_params = self.__qsr_params_defaults.copy()

        try: # global namespace
            if req_params["dynamic_args"]["for_all_qsrs"]:
                for k, v in req_params["dynamic_args"]["for_all_qsrs"].items():
                    qsr_params[k] = v
        except KeyError:
            pass

        try: # General case
            if req_params["dynamic_args"][self.__global_unique_id]:
                for k, v in req_params["dynamic_args"][self.__global_unique_id].items():
                    qsr_params[k] = v
        except KeyError:
            pass

        try: # Parameters for a specific variant
            if req_params["dynamic_args"][self._unique_id]:
                for k, v in req_params["dynamic_args"][self._unique_id].items():
                    qsr_params[k] = v
        except KeyError:
            pass

        if not isinstance(qsr_params["no_collapse"], bool) or not isinstance(qsr_params["validate"], bool):
            raise TypeError("'no_collapse' and 'validate' have to be boolean values.")

        for param in qsr_params:
            if param not in self.__qsr_params_defaults and param not in self._common_dynamic_args:
                raise KeyError("%s is an unknown parameter" % str(param))

        return qsr_params

    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, req_params, **kwargs):
        ret = World_QSR_Trace(qsr_type=self._unique_id)
        qtc_sequence = {}
        for t, tp in zip(timestamps[1:], timestamps):
            world_state_now = world_trace.trace[t]
            world_state_previous = world_trace.trace[tp]
            qsrs_for = self._process_qsrs_for([world_state_previous.objects.keys(), world_state_now.objects.keys()],
                                              req_params["dynamic_args"])
            for o1_name, o2_name in qsrs_for:
                between = str(o1_name) + "," + str(o2_name)
                qtc = np.array([], dtype=int)
                k = [world_state_previous.objects[o1_name].x,
                     world_state_previous.objects[o1_name].y,
                     world_state_now.objects[o1_name].x,
                     world_state_now.objects[o1_name].y]
                l = [world_state_previous.objects[o2_name].x,
                     world_state_previous.objects[o2_name].y,
                     world_state_now.objects[o2_name].x,
                     world_state_now.objects[o2_name].y]
                qtc = self._create_qtc_representation(
                    k,
                    l,
                    qsr_params["quantisation_factor"]
                )

                try:
                    qtc_sequence[between] = np.append(
                        qtc_sequence[between],
                        qtc
                    ).reshape(-1,4)
                except KeyError:
                    qtc_sequence[between] = qtc

        for between, qtc in qtc_sequence.items():
            if not qsr_params["no_collapse"]:
                qtc = self._collapse_similar_states(qtc)
            if qsr_params["validate"]:
                qtc = self._validate_qtc_sequence(qtc)
            qtc = qtc if len(qtc.shape) > 1 else [qtc]
            for idx, q in enumerate(qtc):
                qsr = QSR(
                    timestamp=idx+1,
                    between=between,
                    qsr=self.qtc_to_output_format(q)
                )
                ret.add_qsr(qsr, idx+1)

        return ret

    def _postprocess_world_qsr_trace(self, world_qsr_trace, world_trace, world_trace_timestamps, qsr_params, req_params, **kwargs):
        if qsr_params["no_collapse"] and not qsr_params["validate"]:
            return World_QSR_Trace(
                qsr_type=world_qsr_trace.qsr_type,
                trace={t: world_qsr_trace.trace[tqtc]
                       for t, tqtc in zip(
                    world_trace.get_sorted_timestamps()[1:],
                    world_qsr_trace.get_sorted_timestamps()
                )
                       }
            )
        else:
            return world_qsr_trace

    def create_qtc_string(self, qtc):
        return ','.join(map(str, qtc.astype(int))).replace('-1','-').replace('1','+')

    @abstractmethod
    def qtc_to_output_format(self, qtc):
        """Overwrite this for the different QTC variants to select only the parts
        from the QTCC tuple that you would like to return.
        Example for QTCB: return qtc[0:2]

        :param qtc: The full QTCC tuple [q1,q2,q4,q5]

        :return: The part of the tuple you would to have as a result using create_qtc_string
        """
        return ""
