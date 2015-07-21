#!/usr/bin/env python

from abc import ABCMeta
import numpy as np
from qsrrep_hmms.hmm_abstractclass import HMMAbstractclass


class QTCHMMAbstractclass(HMMAbstractclass):
    __metaclass__ = ABCMeta

    def __init__(self):
        super(QTCHMMAbstractclass, self).__init__()

    def _create_transition_matrix(self, size, **kwargs):
        """Creates a Conditional Neighbourhood Diagram as a basis for the HMM.

        :param kwargs:
            * qtc: list of lists containing all possible qtc states. Different for all 3 qtc versions.

        :return: The transition matrix only allowing transitions according to the CND
        """

        qtc = np.array(kwargs["qtc"])

        # TODO insert matlab documentation on how this works here
        trans = np.zeros((size, size))
        for i1 in xrange(qtc.shape[0]):
            for i2 in xrange(i1+1, qtc.shape[0]):
                trans[i1+1, i2+1] = np.nanmax(np.absolute(qtc[i1]-qtc[i2])) != 2
                if trans[i1+1, i2+1] == 1:
                    for j1 in xrange(qtc.shape[1]-1):
                        for j2 in xrange(j1+1, qtc.shape[1]):
                            if sum(np.absolute(qtc[i1, [j1, j2]])) == 1 \
                                    and sum(np.absolute(qtc[i2, [j1, j2]])) == 1:
                                if np.nanmax(np.absolute(qtc[i1, [j1, j2]]-qtc[i2, [j1, j2]])) > 0 \
                                        and sum(qtc[i1, [j1, j2]]-qtc[i2, [j1,j2]]) != 1:
                                    trans[i1+1, i2+1] = 5
                                    break
                        if trans[i1+1, i2+1] != 1:
                            break
                trans[i2+1, i1+1] = trans[i1+1, i2+1]

        # Setting start and end transition probs and pseudo probs
        trans[trans != 1] = 0
        trans[trans == 0] = 0.00001
        trans[0] = 1
        trans[:, 0] = 0
        trans[:, -1] = 1
        trans[0, -1] = 0
        trans[-1] = 0
        trans += np.dot(np.eye(size), 0.00001)
        trans[0, 0] = 0

        return trans / trans.sum(axis=1).reshape(-1, 1)

    def _create_emission_matrix(self, size, **kwargs):
        """Creating an emission matrix with the highest prob along the diagonal
        and a "pseudo" prob for all other states.

        :param kwargs: empty

        :return: The emission probability matrix

        """
        emi = np.eye(size)
        emi[emi == 0] = 0.0001

        return emi/emi.sum(axis=1).reshape(-1, 1)


    def _qsr_to_symbol(self, qsr_data):
        """Transforms a qtc state chain to a list of numbers

        :param qsr_data: The list of lists of qtc strings or numpy array states
            E.g.: [['++++','+++0','+++-,]] or [[[1,1,1,1],[1,1,1,0],[1,1,1,-1]]]

        :return: A lists of lists of alphabet symbols corresponding to the given state chains
        """
        qsr_data = np.array(qsr_data)
        state_rep = []
        for idx, element in enumerate(qsr_data):
            if all(isinstance(x, str) or isinstance(x, unicode) for x in element):
                element = self._qtc_str_to_num(element) # check if content is string instead of numbers and convert
            element = np.array(element)
            try:
                d = element.shape[1]
            except IndexError: # Not a list of lists of lists
                return self._qsr_to_symbol([qsr_data])
            mult = 3**np.arange(d-1, -1, -1)
            state_num = np.append(
                0, # Start symbol
                ((element + 1)*np.tile(mult, (element.shape[0], 1))).sum(axis=1) + 1
            )
            state_num = np.append(state_num, self.num_possible_states-1) # End symbol
            state_char = ''
            for n in state_num:
                state_char += chr(int(n)+32)
            state_rep.append(state_num.tolist())

        return state_rep

    def _qtc_num_to_str(self, qtc_num_list):
        """Transforms qtc array representation to string

        :param qtc_num_list: A list of lists of arrays
            E.g.: [[1,1,1,1],[1,1,1,0]]

        :return: The corresponding string representation
            E.g. ['++++','+++0']
        """
        qtc_str = []
        for elem in qtc_num_list:
            s = ''
            for num in elem:
                if num == 0:
                    s +='0'
                elif num == 1:
                    s +='+'
                elif num == -1:
                    s +='-'
            qtc_str.append(s)
        return qtc_str

    def _qtc_str_to_num(self, qtc_str_list):
        """Transforms qtc string representation to an array

        :param qtc_num_list: A list of strings
            E.g.: ['++++','+++0']

        :return: The corresponding array representation
            E.g. [[1,1,1,1],[1,1,1,0]]
        """
        qtc_num = []
        for elem in qtc_str_list:
            n = []
            for s in elem:
                if s == '0':
                    n.append(0.0)
                elif s == '+':
                    n.append(1.0)
                elif s == '-':
                    n.append(-1.0)
            qtc_num.append(n)
        return qtc_num
