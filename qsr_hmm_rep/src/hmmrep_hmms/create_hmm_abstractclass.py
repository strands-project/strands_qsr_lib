#!/usr/bin/env python

from abc import abstractmethod, ABCMeta
import numpy as np
import ghmm as gh


class CreateHMMAbstractclass():
    """Abstract class for HMM generation"""
    __metaclass__ = ABCMeta

    def __init__(self):
        self.hmm = None

    def _create_sequence_set(self, qsr_seq, symbols):
        """Creating a sequence set for training

        :param qsr_seq: the observation seqence of symbols according to the alphabet as a list of lists
        :param symbols: the alphabet of possible symbols

        :return: the sequence set for the given observations
        """
        return gh.SequenceSet(symbols, qsr_seq)

    @abstractmethod
    def create_transisition_and_emission_matrices(self, size, **kwargs):
        """Abstract method for the creation of the transition and emission probability matrices

        :return: trans: the transition matrix, emi: emission matrix both as numpy arrays
        """
        trans = np.array([])
        emi = np.array([])

        return trans, emi

    @abstractmethod
    def qsr_to_state(self, qsr_data):
        """Transforms a list of qsr state chains to a list of lists of numbers according to the alphabet

        :return: List of lists containing the qsr input data as symbols from the alphabet
            E.g.: [[1,4,2,7],[0,5,3,8,5,1,3]]
        """

        state_rep = []

        return state_rep

    def _generate_alphabet(self, num_symbols):
        """Generate a simple integer alphabet: [0:num_symbols-1]"""
        return gh.IntegerRange(0, num_symbols)

    def _train(self, seq, trans, emi, num_possible_states):
        """Uses the given parameters to train a multinominal HMM to represent the given seqences of observations

        :param seq: the sequence of observations represented by alphabet symbols
        :param trans: the transition matrix as a numpy array
        :param emi: the emission matrix as a numpy array
        :param num_possible_states: the total number of possible states

        :return: the via baum-welch training generated hmm
        """

        print 'Generating HMM:'
        print '\tCreating symbols...'
        symbols = self._generate_alphabet(num_possible_states)
        startprob = np.zeros((num_possible_states))
        startprob[0] = 1
        print '\t\t', symbols
        print '\tCreating HMM...'
        hmm = gh.HMMFromMatrices(
            symbols,
            gh.DiscreteDistribution(symbols),
            trans.tolist(),
            emi.tolist(),
            startprob.tolist()
        )
        print '\tTraining...'
        hmm.baumWelch(self._create_sequence_set(seq, symbols))

        return hmm


    def create(self, qsr_seq, num_possible_states, **kwargs):
        """Create and trains a HMM to represent the given qtc sequences"""

        state_seq = self.qsr_to_state(qsr_seq)
        trans, emi = self.create_transisition_and_emission_matrices(size=num_possible_states, **kwargs)
        self.hmm = self._train(state_seq, trans, emi, num_possible_states)
        print '...done'
        return self.hmm
