# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 14:19:25 2015

@author: cdondrup
"""

from hmmrep_hmms.create_qtcc_hmm import CreateQTCCHMM
from hmmrep_hmms.create_qtcb_hmm import CreateQTCBHMM
from hmmrep_hmms.create_qtcbc_hmm import CreateQTCBCHMM
import tempfile
import os
from abc import ABCMeta, abstractmethod


class HMMRepLib(object):

    __hmm_types_available = {
        "create_qtcc_hmm": CreateQTCCHMM,
        "create_qtcb_hmm": CreateQTCBHMM,
        "create_qtcbc_hmm": CreateQTCBCHMM
    }

    __hmm_reps_active = {}

    def __init__(self):
        for k, v in self.__hmm_types_available.items():
            self.__hmm_reps_active[k] = v()

    def print_hmms_available(self):
        l = sorted(self.__hmm_types_available)
        print("Types of HMMs that have been included so far are:")
        for i in l:
            print("-", i)

    def create_hmm(self, **kwargs):
        hmm = self.__hmm_reps_active[kwargs["qsr_type"]].get(
            qsr_seq=kwargs["qsr_seq"]
        )
        xml = self.__create_xml_rep(hmm)

        return xml, hmm

    def __create_xml_rep(self, hmm):
        fd, name = tempfile.mkstemp()
        hmm.write(name)
        f = os.fdopen(fd, 'r')
        xml = f.read()
        f.close()
        os.remove(name)
        return xml

    def request(self, request_message):
        assert(issubclass(request_message.__class__, HMMRepRequestAbstractclass))
        return request_message.call_function(self)

class HMMRepRequestAbstractclass(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def call_function(self, inst):
        return


class HMMRepRequestCreate(HMMRepRequestAbstractclass):

    __const_function_pointer = lambda *args, **kwargs: args[1].create_hmm(**kwargs)

    def __init__(self, qsr_type, qsr_seq, store=False):
        self.qsr_type = qsr_type
        self.qsr_seq = qsr_seq
        self.store = store

    def call_function(self, inst):
        return self.__const_function_pointer(inst, qsr_type=self.qsr_type, qsr_seq=self.qsr_seq, store=self.store)
