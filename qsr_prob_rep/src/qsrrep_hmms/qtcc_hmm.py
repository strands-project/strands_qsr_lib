#!/usr/bin/env python

from qsrrep_hmms.qtc_hmm_abstractclass import QTCHMMAbstractclass
import numpy as np


class QTCCHMM(QTCHMMAbstractclass):

    def __init__(self):
        super(QTCCHMM, self).__init__()
        self.num_possible_states = 83 # Setting number of possible states: QTCC + start and end
        self.multiplier = 3**np.array(range(3,-1,-1))

    def _create_transition_matrix(self, size, **kwargs):
        """Creates a Conditional Neighbourhood Diagram for QTCC as a basis for the HMM.

        :param kwargs:
            * qtc: list of lists containing all possible qtc states. Different for all 3 qtc versions.

        :return: The transition matrix only allowing transitions according to the CND
        """

        qtc = []
        # creating list of possible states
        for i in xrange(1, 4):
            for j in xrange(1, 4):
                for k in xrange(1, 4):
                    for l in xrange(1, 4):
                        qtc.append([i-2, j-2, k-2, l-2])

        # Calling parent to generate actual matrix
        return super(QTCCHMM, self)._create_transition_matrix(size=size, qtc=qtc)

    def _symbol_to_qsr(self, symbols):
        """Transforming alphabet symbols to QTCC states.

        :param symbols: A list of symbols

        :return: The list of corresponding qtc symbols
        """

        ret = []
        for s in symbols:
            qtc = []
            for c in s[1:-1]:
                qtc.append(self.symbol_to_qsr(c).tolist())

            ret.append(self._qtc_num_to_str(qtc))

        return ret

    def symbol_to_qsr(self, symbol):
        rc = np.array([symbol-1])
        f = np.array([np.floor(rc[0]/self.multiplier[0])])

        for i in range(1, len(self.multiplier)):
            rc = np.append(rc, rc[i-1] - f[i-1] * self.multiplier[i-1])
            f = np.append(f, np.floor(rc[i]/self.multiplier[i]))

        return f-1