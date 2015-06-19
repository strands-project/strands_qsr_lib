#!/usr/bin/env python

import numpy as np
from hmmrep_hmms.create_hmm_abstractclass import CreateHMMAbstractclass


class QtcException(Exception):
    def __init__(self, message):

        # Call the base class constructor with the parameters it
        # needs
        Exception.__init__(self, "QTC Exception: " + message)


class CreateQTCHMM(CreateHMMAbstractclass):

    def __init__(self):
        super(CreateQTCHMM, self).__init__()

    def create_transisition_and_emission_matrices(self, size, **kwargs):
        """Creates a Conditional Neighbourhood Diagram as a basis for the HMM"""

        qtc = []

        if kwargs["qtc_type"] == 'qtcb':
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    qtc.append([i-2, j-2])
        elif kwargs["qtc_type"] == 'qtcc':
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    for k in xrange(1, 4):
                        for l in xrange(1, 4):
                            qtc.append([i-2, j-2, k-2, l-2])
        elif kwargs["qtc_type"] == 'qtcbc':
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    qtc.append([i-2, j-2, np.NaN, np.NaN])
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    for k in xrange(1, 4):
                        for l in xrange(1, 4):
                            qtc.append([i-2, j-2, k-2, l-2])
        else:
            raise(QtcException("createCNDTransEmiProb: Unknow qtc type: {!r}".format(kwargs["qtc_type"])))

        qtc = np.array(qtc)
        #np.savetxt('/home/cdondrup/qtc.csv', qtc, delimiter=',', fmt='%1f')

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

        trans[trans != 1] = 0
        #np.savetxt('/home/cdondrup/trans.csv', np.rint(trans).astype(int), delimiter=',', fmt='%i')
        trans[trans == 0] = 0.00001
        trans[0] = 1
        trans[:, 0] = 0
        trans[:, -1] = 1
        trans[0, -1] = 0
        trans[-1] = 0
        trans += np.dot(np.eye(size), 0.00001)
        trans[0, 0] = 0

        trans = trans / trans.sum(axis=1).reshape(-1, 1)
        #np.savetxt('/home/cdondrup/trans.csv', trans, delimiter=',')

        emi = np.eye(size)
        emi[emi == 0] = 0.0001

        return trans, emi


    def qsr_to_state(self, qsr_data):
        """Transforms a qtc state to a number"""

        state_rep = []
        for idx, element in enumerate(qsr_data):
            d = element.shape[1]
            mult = 3**np.arange(d-1, -1, -1)
            state_num = np.append(
                0,
                ((element + 1)*np.tile(mult, (element.shape[0], 1))).sum(axis=1) + 1
            )
            state_num = np.append(state_num, 82)
            state_char = ''
            for n in state_num:
                state_char += chr(int(n)+32)
            state_rep.append(state_num.tolist())

        return state_rep

#    def create(self, qsr_seq, num_possible_states, *args, **kwargs):
#        """Create and trains a HMM to represent the given qtc sequences"""
#
#        super(CreateQTCHMM, self).create(
#            qsr_seq=qsr_seq,
#            num_possible_states=num_possible_states,
#            args=args,
#            kwargs=kwargs
#        )


#    def createTestSequence(self, seq_path, qtc_type='qtcc'):
#        if qtc_type is 'qtcb':
#            return createSequenceSet(qtc2state(readQtcFiles(seq_path)), generateAlphabet(11))
#        elif qtc_type is 'qtcc':
#            return createSequenceSet(qtc2state(readQtcFiles(seq_path)), generateAlphabet(83))
#        elif qtc_type is 'qtcbc':
#            return createSequenceSet(qtc2state(readQtcFiles(seq_path)), generateAlphabet(92))
#        else:
#            raise(QtcException("Unknow qtc type: {!r}".format(qtc_type)))
