#!/usr/bin/env python

from hmmrep_hmms.create_qtc_hmm_abstractclass import CreateQTCHMMAbstractclass
import numpy as np


class QtcException(Exception):
    def __init__(self, message):

        # Call the base class constructor with the parameters it
        # needs
        Exception.__init__(self, "QTC Exception: " + message)


class CreateQTCBCHMM(CreateQTCHMMAbstractclass):

    def __init__(self):
        super(CreateQTCBCHMM, self).__init__()
        self.num_possible_states = 92

    def create_transition_matrix(self, size, **kwargs):
        """Creates a Conditional Neighbourhood Diagram as a basis for the HMM"""

        qtc = []

        for i in xrange(1, 4):
            for j in xrange(1, 4):
                qtc.append([i-2, j-2, np.NaN, np.NaN])
        for i in xrange(1, 4):
            for j in xrange(1, 4):
                for k in xrange(1, 4):
                    for l in xrange(1, 4):
                        qtc.append([i-2, j-2, k-2, l-2])

        return super(CreateQTCBCHMM, self).create_transition_matrix(size=size, qtc=qtc)