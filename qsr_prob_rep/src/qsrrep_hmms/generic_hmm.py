# -*- coding: utf-8 -*-

from qsrrep_hmms.hmm_abstractclass import HMMAbstractclass
import numpy as np

class GenericHMM(HMMAbstractclass):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.num_possible_states = -1

    def _create(self, **kwargs):
        """Creates and trains (using '_train') a HMM to represent the given qtc sequences.
        Main function to create and train the hmm. Please override with special
        behaviour if necessary.

        This function is called by the library to create the hmm.

        :param **kwargs:
            - qsr_seq: the sequence of QSRs. This should be a list of state chains, i.e. a list of lists

        :return: The trained HMM

        """

        self._state_list = kwargs["lookup_table"]
        if self._state_list == None:
            raise Exception("Generic HMMs need at least a state look-up table.")
        self.trans = kwargs["transition_matrix"]
        if self.trans != None:
            if np.array(self.trans).shape[0] != len(self._state_list) \
                or np.array(self.trans).shape[1] != len(self._state_list):
                raise Exception("Transition matrix needs to be square (MxM) and M has to be equal to the number of states in the state look-up table.")
        self.emi = kwargs["emission_matrix"]
        if self.emi != None:
            if np.array(self.emi).shape[0] != len(self._state_list) \
                or np.array(self.emi).shape[1] != len(self._state_list):
                raise Exception("Emission matrix needs to be square (MxM) and M has to be equal to the number of states in the state look-up table.")

        self.num_possible_states = len(self._state_list)

        return super(self.__class__, self)._create(**kwargs)

    def _sample(self, **kwargs):
        """Gnerating samples from the trained HMM given a maximum sample length
        and thee total number of samples.

        :param kwargs:
            * max_length: The maximum length of the resulting samples. This will always be kept if possible.
            * num_samples: The number of samples to generate
            * hmm: The HMM from which to sample

        :return: A list of lists of samples

        """
        self._state_list = kwargs["lookup_table"]
        if self._state_list == None:
            raise Exception("Generic HMMs need at least a state look-up table.")
        self.num_possible_states = len(self._state_list)

        return super(self.__class__, self)._sample(**kwargs)


    def _log_likelihood(self, **kwargs):
        """Computeed the loglikelihood for the given sample(s) to be produced by
        the HMM.

        :param kwargs:
            * qsr_seq: A list of lists of qsr sequences to check against the HMM
            * hmm: The to generate the loglikelihood for

        :return: The accumulated loglikelihood for all the given samples
        """

        self._state_list = kwargs["lookup_table"]
        if self._state_list == None:
            raise Exception("Generic HMMs need at least a state look-up table.")
        self.num_possible_states = len(self._state_list)

        return super(self.__class__, self)._log_likelihood(**kwargs)

    def _qsr_to_symbol(self, qsr_data):
        """Transforms a list of qsr state chains to a list of lists of numbers according to the alphabet.
        Needs to be overridden by the specific QSR to handle the correct symbols.

        :return: List of lists containing the qsr input data as symbols from the alphabet
            E.g.: [[1,4,2,7],[0,5,3,8,5,1,3]]
        """
        if not type(qsr_data[0]) == list:
            return self._qsr_to_symbol([qsr_data])
        state_rep = []
        for element in qsr_data:
            state_rep.append([self._state_list.index(x) for x in element])
        return state_rep

    def _symbol_to_qsr(self, symbols):
        """Transforms a list of symbols to the corresponding qsr state chains.
        Needs to be overridden by the specific QSR to handle the correct symbols.

        :return: List of lists of qsr state chains
            E.g.: [['dc','po','o'],['dc','po']]
        """
        ret = []
        for s in symbols:
            ret.append([self._state_list[x] for x in s])
        return ret

    def _create_transition_matrix(self, size, **kwargs):
        """Method for the creation of the transition probability matrix. Creates
        a uniformly distributed matrix. Please override if special behaviour
        is necessary.

        :return: uniform SIZExSIZE transition matrix as a numpy array
        """

        if self.trans != None:
            return np.array(self.trans)
        return super(self.__class__, self)._create_transition_matrix(size=size, **kwargs)

    def _create_emission_matrix(self, size, **kwargs):
        """Method for the creation of the emission probability matrix. Creates
        a uniformly distributed matrix. Please override if special behaviour
        is necessary.

        :return: uniform SIZExSIZE emission matrix as a numpy array
        """

        if self.emi != None:
            return np.array(self.emi)
        return super(self.__class__, self)._create_emission_matrix(size=size, **kwargs)