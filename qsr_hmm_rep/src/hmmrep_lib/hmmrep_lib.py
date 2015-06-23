# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 14:19:25 2015

@author: cdondrup
"""

from hmmrep_hmms.qtcc_hmm import QTCCHMM
from hmmrep_hmms.qtcb_hmm import QTCBHMM
from hmmrep_hmms.qtcbc_hmm import QTCBCHMM
from hmmrep_requests import HMMRepRequestAbstractclass
from hmmrep_responses import HMMReqResponseCreate, HMMReqResponseSample, HMMReqResponseLogLikelihood
import tempfile
import os
import ghmm as gh
import json


class HMMRepLib(object):

    __hmm_types_available = {
        "qtcc_hmm": QTCCHMM,
        "qtcb_hmm": QTCBHMM,
        "qtcbc_hmm": QTCBCHMM
    }

    __hmm_types_active = {}

    def __init__(self):
        for k, v in self.__hmm_types_available.items():
            self.__hmm_types_active[k] = v()

    def print_hmms_available(self):
        l = sorted(self.__hmm_types_available)
        print("Types of HMMs that have been included so far are:")
        for i in l:
            print("-", i)

    def _create_hmm(self, **kwargs):
        hmm = self.__hmm_types_active[kwargs["qsr_type"]].get_hmm(
            **kwargs
        )
        xml = self.__create_xml_from_hmm(hmm)

        return HMMReqResponseCreate(data=xml, qsr_type=kwargs["qsr_type"])

    def _sample_hmm(self, **kwargs):
        sample = self.__hmm_types_active[kwargs["qsr_type"]].get_samples(
            hmm=self.__create_hmm_from_xml(xml=kwargs["xml"]),
            **kwargs
        )
        return HMMReqResponseSample(data=json.dumps(sample), qsr_type=kwargs["qsr_type"])

    def _get_log_likelihood(self, **kwargs):
        loglike = self.__hmm_types_active[kwargs["qsr_type"]].get_log_likelihood(
            hmm=self.__create_hmm_from_xml(xml=kwargs["xml"]),
            **kwargs
        )
        return HMMReqResponseLogLikelihood(data=json.dumps(loglike), qsr_type=kwargs["qsr_type"])

    def __create_xml_from_hmm(self, hmm):
        """Creates a xml representation of the hmm. Not nice to use tempfile
        but not otherwise possible due to hidden code and swig in ghmm.
        """
        fd, name = tempfile.mkstemp()
        hmm.write(name)
        f = os.fdopen(fd, 'r')
        xml = f.read()
        f.close()
        os.remove(name)

        return xml

    def __create_hmm_from_xml(self, xml):
        """Creates a hmm from the xml representation. Not nice to use tempfile
        but not otherwise possible due to hidden code and swig in ghmm.
        """
        fd, name = tempfile.mkstemp()
        f = os.fdopen(fd, 'w')
        f.write(xml)
        f.close() # File has to be closed before reading from it. Otherwise this will fail.
        hmm = gh.HMMOpen(fileName=name, filetype=gh.GHMM_FILETYPE_XML)
        os.remove(name)
        return hmm

    def request(self, request_message):
        assert(issubclass(request_message.__class__, HMMRepRequestAbstractclass))
        return request_message.call_function(self)
