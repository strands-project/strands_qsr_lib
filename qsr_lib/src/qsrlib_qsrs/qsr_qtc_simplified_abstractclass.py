#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 19 11:22:16 2015

@author: cdondrup
"""
from abc import abstractmethod, ABCMeta
from qsrlib_qsrs.qsr_abstractclass import QSR_Abstractclass
from qsrlib_io.world_qsr_trace import *
from exceptions import Exception, AttributeError
import numpy as np


class QTCException(Exception):
    pass


class QSR_QTC_Simplified_Abstractclass(QSR_Abstractclass):
    """Abstract class for the QSR makers"""
    __metaclass__ = ABCMeta

    def __init__(self):
        self.qtc_type = ""
        self.qsr_keys = "qtcs"

    def custom_set_from_config_file(self, document):
        pass

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
        if self.qtc_type == "b":
            qtc = qtc[:,0:2]

        if len(qtc.shape) == 1:
            return qtc  # Only one state in chain.

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

        col_qtc = np.array([qtc[0,:].copy()])
        j = 0
        for i in xrange(1, qtc.shape[0]):
            if not self._nan_equal(col_qtc[j,:], qtc[i,:]):
                col_qtc = np.append(col_qtc, [qtc[i,:]], axis=0)
                j += 1

        return col_qtc

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

        return [k[0],l[0],k[1],l[1]]

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

    def custom_help(self):
        """Write your own help message function"""
        print "where,\n" \
            "it is always necessary to have two agents in every timestep:\n"\
            "x, y: the xy-coords of the agents\n" \
            "quantisation_factor: the minimum distance the agents must diverge from the double cross between two timesteps to be counted as movement. Must be in the same unit as the x,y coordinates.\n"\
            "validate: True|False validates the QTC sequence to not have illegal transitions. This inserts necessary transitional steps and messes with the timesteps."


    def custom_checks(self, input_data):
        """Write your own custom checks on top of the default ones


        :return: error code, error message (integer, string), use 10 and above for error code as 1-9 are reserved by system
        """
        timestamps = input_data.get_sorted_timestamps()
        if len(timestamps) < 2:
            return 50, "Data for at least two separate timesteps has to be provided."
        objects_names = sorted(input_data.trace[timestamps[0]].objects.keys())
        for t in timestamps:
            for o in objects_names:
                try :
                    input_data.trace[t].objects[o]
                except KeyError:
                        return 51, "Only one object defined for timestep %f. Two objects have to be present at any given step." % t
                if np.isnan(input_data.trace[t].objects[o].x) \
                    or np.isnan(input_data.trace[t].objects[o].y):
                        return 52, "Coordinates x: %f, y: %f are not defined correctly for timestep %f." % (x, y, t)
        return 0, ""

    def custom_checks_for_qsrs_for(self, qsrs_for, error_found):
        """Custom checks of the qsrs_for field

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
        ret = World_QSR_Trace(qsr_type=self.qsr_type)
        timestamps = input_data.get_sorted_timestamps()

        if kwargs["qsrs_for"]:
            qsrs_for, error_found = self.check_qsrs_for_data_exist(sorted(input_data.trace[timestamps[0]].objects.keys()), kwargs["qsrs_for"])
            if error_found:
                raise Exception("Invalid object combination. Has to be list of tuples. Heard: " + np.array2string(np.array(kwargs['qsrs_for'])))
        else:
            qsrs_for = self._return_all_possible_combinations(sorted(input_data.trace[timestamps[0]].objects.keys()))

        parameters = {
            "quantisation_factor": 0.0,
            "validate": True,
            "no_collapse": False
        }

        try:
            if kwargs["dynamic_args"]["parameters"]:
                for k, v in kwargs["dynamic_args"]["parameters"].items():
                    parameters[k] = v
        except:
            print "No parameters found, will use default parameters: ", parameters

        if qsrs_for:
            for p in qsrs_for:
                between = str(p[0]) + "," + str(p[1])
                o1_name = p[0]
                o2_name = p[1]
                quantisation_factor = parameters["quantisation_factor"]
                try:
                    if input_data.trace[0].objects[o1_name].kwargs["quantisation_factor"]:
                        print "Definition of quantisation_factor in object is depricated. Please use parameters field in dynamic_args in service call."
                        quantisation_factor = input_data.trace[0].objects[o1_name].kwargs["quantisation_factor"]
                except:
                    pass
                qtc_sequence = np.array([], dtype=int)
                for t0, t1 in zip(timestamps, timestamps[1:]):
                    timestamp = t1
                    try:
                        k = [input_data.trace[t0].objects[o1_name].x,
                             input_data.trace[t0].objects[o1_name].y,
                             input_data.trace[t1].objects[o1_name].x,
                             input_data.trace[t1].objects[o1_name].y]
                        l = [input_data.trace[t0].objects[o2_name].x,
                             input_data.trace[t0].objects[o2_name].y,
                             input_data.trace[t1].objects[o2_name].x,
                             input_data.trace[t1].objects[o2_name].y]
                        qtc_sequence = np.append(qtc_sequence, self._create_qtc_representation(
                            k,
                            l,
                            quantisation_factor
                        )).reshape(-1,4)

                    except KeyError:
                        ret.add_empty_world_qsr_state(timestamp)

                no_collapse = parameters["no_collapse"]
                try:
                    if input_data.trace[0].objects[o1_name].kwargs["no_collapse"]:
                        print "Definition of no_collapse in object is depricated. Please use parameters field in dynamic_args in service call."
                        no_collapse = input_data.trace[0].objects[o1_name].kwargs["no_collapse"]
                except:
                    pass
                try:
                    validate = parameters["validate"]
                    if input_data.trace[0].objects[o1_name].kwargs["validate"]:
                        print "Definition of validate in object is depricated. Please use parameters field in dynamic_args in service call."
                        validate = input_data.trace[0].objects[o1_name].kwargs["validate"]
                except:
                    pass

                if not type(no_collapse) is bool or not type(validate) is bool:
                    raise Exception("'no_collapse' and 'validate' have to be boolean values.")

                if not no_collapse:
                    qtc_sequence = self._collapse_similar_states(qtc_sequence)
                if validate:
                    qtc_sequence = self._validate_qtc_sequence(qtc_sequence)
                for idx, qtc in enumerate(qtc_sequence):
                    qsr = QSR(
                        timestamp=idx+1,
                        between=between,
                        qsr=self.qtc_to_output_format((qtc), kwargs["future"])
                    )
                    ret.add_qsr(qsr, idx+1)

        return ret

    def _return_all_possible_combinations(self, objects_names):
        if len(objects_names) < 2:
            return []
        ret = []
        for i in objects_names:
            for j in objects_names:
                if i != j:
                    ret.append((i, j))
        return ret


    @abstractmethod
    def qtc_to_output_format(self, qtc, future=False):
        """Overwrite this for the different QTC variants to select only the parts
        from the QTCC tuple that you would like to return.
        Example for QTCB: return qtc[0:2]

        :param qtc: The full QTCC tuple [q1,q2,q4,q5]

        :return: The part of the tuple you would to have as a result
        """
        # TODO bit weird that instantiation of this calls the parent abstract just for string replacement, better to be done in another method
        return ','.join(map(str, qtc.astype(int))).replace('-1','-').replace('1','+')
