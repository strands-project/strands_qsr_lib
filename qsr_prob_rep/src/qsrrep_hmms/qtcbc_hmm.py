#!/usr/bin/env python

from qsrrep_hmms.qtc_hmm_abstractclass import QTCHMMAbstractclass
from qsrrep_hmms.qtcb_hmm import QTCBHMM
from qsrrep_hmms.qtcc_hmm import QTCCHMM
import numpy as np


class QTCBCHMM(QTCHMMAbstractclass):

    def __init__(self):
        super(QTCBCHMM, self).__init__()
        self.num_possible_states = 92 # Setting number of possible states: QTCBC + start and end
        self.qtcb = QTCBHMM()
        self.qtcc = QTCCHMM()

    def _create_transition_matrix(self, size, **kwargs):
        """Creates a Conditional Neighbourhood Diagram for QTCBC as a basis for the HMM.

        :param kwargs:
            * qtc: list of lists containing all possible qtc states. Different for all 3 qtc versions.

        :return: The transition matrix only allowing transitions according to the CND
        """

        qtc = []
        # creating list of possible states
        for i in xrange(1, 4):
            for j in xrange(1, 4):
                qtc.append([i-2, j-2, np.NaN, np.NaN])
        for i in xrange(1, 4):
            for j in xrange(1, 4):
                for k in xrange(1, 4):
                    for l in xrange(1, 4):
                        qtc.append([i-2, j-2, k-2, l-2])

        # Calling parent to generate actual matrix
        return super(QTCBCHMM, self)._create_transition_matrix(size=size, qtc=qtc)

    def _symbol_to_qsr(self, symbols):
        """Transforming alphabet symbols to QTCBC states.

        :param symbols: A list of symbols

        :return: The list of corresponding qtc symbols
        """

        ret = []
        for s in symbols:
            qtc = []
            for c in s[1:-1]:
                if c <= 9: # QTCB
                    qtc.append(self.qtcb.symbol_to_qsr(c))
                else:
                    qtc.append(self.qtcc.symbol_to_qsr(c-9))

            ret.append(self._qtc_num_to_str(qtc))

        return ret

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
            state_num = np.array([0]) # Start symbol
            #Not ellegant at all but necessary due to the nan values and the different multipliers for qtcb and qtcc
            for x in element:
                x = x[~np.isnan(x)]
                d = x.shape[0]
                mult = 3**np.arange(d-1, -1, -1)
                num = ((x + 1)*mult).sum() + 1
                state_num = np.append(
                    state_num,
                    num if d == 2 else num + 9 # Adding a 9 when the state is qtcc
                )
            state_num = np.append(state_num, self.num_possible_states-1) # End symbol
            state_char = ''
            for n in state_num:
                state_char += chr(int(n)+32)
            state_rep.append(state_num.tolist())

        return state_rep
