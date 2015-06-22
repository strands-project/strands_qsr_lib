#!/usr/bin/env python

from hmmrep_hmms.create_qtc_hmm_abstractclass import CreateQTCHMMAbstractclass


class QtcException(Exception):
    def __init__(self, message):

        # Call the base class constructor with the parameters it
        # needs
        Exception.__init__(self, "QTC Exception: " + message)


class CreateQTCCHMM(CreateQTCHMMAbstractclass):

    def __init__(self):
        super(CreateQTCCHMM, self).__init__()
        self.num_possible_states = 83

    def create_transition_matrix(self, size, **kwargs):
        """Creates a Conditional Neighbourhood Diagram as a basis for the HMM"""

        qtc = []

        for i in xrange(1, 4):
            for j in xrange(1, 4):
                for k in xrange(1, 4):
                    for l in xrange(1, 4):
                        qtc.append([i-2, j-2, k-2, l-2])

        return super(CreateQTCCHMM, self).create_transition_matrix(size=size, qtc=qtc)