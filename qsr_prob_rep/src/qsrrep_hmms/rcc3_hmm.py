# -*- coding: utf-8 -*-

from qsrrep_hmms.hmm_abstractclass import HMMAbstractclass

class RCC3HMM(HMMAbstractclass):

    _state_list = ["dc", "po", "o"]

    def __init__(self):
        super(self.__class__, self).__init__()
        self.num_possible_states = 3

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