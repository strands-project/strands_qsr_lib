#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 19 11:22:16 2015

@author: cdondrup
"""
from abc import abstractmethod, ABCMeta, abstractproperty
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
        except AttributeError as e:
            raise QTCException("Please define a qtc type using self.qtc_type.")
            return None, None
        return ret_str, ret_int

    def validate_qtc_Sequences(self, qtc):
        """Removes illegal state transition by inserting necessary intermediate states

        :param qtc: a numpy array of the qtc state chain. One qtc state per row

        :return: The valid state chain as a numpy array
        """

        newqtc = qtc[0].copy()
        j = 1
        for i in xrange(1, len(qtc)):
            checksum = np.nonzero(qtc[i-1]+qtc[i] == 0)
            intermediate = qtc[i].copy()
            if checksum[0].size > 0:
                for idx in checksum[0]:
                    if np.absolute(qtc[i-1][idx]) + np.absolute(qtc[i][idx]) > 0:
                        intermediate[idx] = 0
                if np.any(intermediate != qtc[i]):
                    newqtc = np.append(newqtc, intermediate)
                    j += 1
            newqtc = np.append(newqtc, qtc[i])
            j += 1

        return newqtc.reshape(-1, 4)

    def create_qtc_representation(self, pos_k, pos_l, quantisation_factor=0):
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
        RL = np.append(pos_k[-2], pos_l[-2]).reshape(-1,2)
        #print "RL", RL
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
        res = []
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
        objects_names = sorted(input_data.trace[timestamps[0]].objects.keys())
        o1_name = objects_names[0]
        o2_name = objects_names[1]
        between = o1_name + "," + o2_name
        timestamps = input_data.get_sorted_timestamps()
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
                qtc = self.create_qtc_representation(
                    k, 
                    l, 
                    input_data.trace[t0].objects[o1_name].kwargs["quantisation_factor"]
                )
                qtc = self.qtc_to_string(qtc)
                qsr = QSR(
                    timestamp=timestamp,
                    between=between,
                    qsr=qtc
                )
                ret.add_qsr(qsr, timestamp)
            except KeyError:
                ret.add_empty_world_qsr_state(timestamp)
        return ret
        
    @abstractmethod
    def qtc_to_string(self, qtc):
        """Overwrite this for the different QTC veriants to select only the parts
        from the QTCC tuple that you would like to return.
        Example for QTCB: return str(qtc[0]) + "," + str(qtc[1])
        
        :param qtc: The full QTCC tuple [q1,q2,q4,q5]
        
        :return: The part of the tuple you would to have as a result converted 
        to a comma separated string
        """
        return ""
