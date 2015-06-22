#!/usr/bin/env python

from hmmrep_hmms.create_qtc_hmm_abstractclass import CreateQTCHMMAbstractclass


class QtcException(Exception):
    def __init__(self, message):

        # Call the base class constructor with the parameters it
        # needs
        Exception.__init__(self, "QTC Exception: " + message)


class CreateQTCBHMM(CreateQTCHMMAbstractclass):

    def __init__(self):
        super(CreateQTCBHMM, self).__init__()
        self.num_possible_states = 11

    def create_transition_matrix(self, size, **kwargs):
        """Creates a Conditional Neighbourhood Diagram as a basis for the HMM"""

        qtc = []

        for i in xrange(1, 4):
            for j in xrange(1, 4):
                qtc.append([i-2, j-2])

        return super(CreateQTCBHMM, self).create_transition_matrix(size=size, qtc=qtc)