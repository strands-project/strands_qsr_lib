# -*- coding: utf-8 -*-

from abc import ABCMeta, abstractmethod

class HMMRepRequestAbstractclass(object):
    __metaclass__ = ABCMeta

    _const_function_pointer = lambda *args, **kwargs: args[1].my_function(**kwargs) # Example function pointer

    @abstractmethod
    def __init__(self):
        self.kwargs = {}

    def call_function(self, inst):
        return self._const_function_pointer(inst, **self.kwargs)


class HMMRepRequestCreate(HMMRepRequestAbstractclass):

    _const_function_pointer = lambda *args, **kwargs: args[1]._create_hmm(**kwargs)

    def __init__(self, qsr_type, qsr_seq, store=False):
        self.kwargs = {
            "qsr_type": qsr_type,
            "qsr_seq": qsr_seq,
            "store": store
        }


class HMMRepRequestSample(HMMRepRequestAbstractclass):

    _const_function_pointer = lambda *args, **kwargs: args[1]._sample_hmm(**kwargs)

    def __init__(self, qsr_type, xml, max_length, num_samples=1):
        self.kwargs = {
            "qsr_type": qsr_type,
            "xml": xml,
            "max_length": max_length,
            "num_samples": num_samples
        }

class HMMRepRequestLogLikelihood(HMMRepRequestAbstractclass):

    _const_function_pointer = lambda *args, **kwargs: args[1]._get_log_likelihood(**kwargs)

    def __init__(self, qsr_type, xml, qsr_seq, num_samples=1):
        self.kwargs = {
            "qsr_type": qsr_type,
            "xml": xml,
            "qsr_seq": qsr_seq
        }