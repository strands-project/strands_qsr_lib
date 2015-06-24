# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 14:19:25 2015

@author: cdondrup
"""

from hmmrep_hmms.qtcc_hmm import QTCCHMM
from hmmrep_hmms.qtcb_hmm import QTCBHMM
from hmmrep_hmms.qtcbc_hmm import QTCBCHMM
from hmmrep_io import HMMRepRequestAbstractclass, HMMReqResponseCreate, HMMReqResponseSample, HMMReqResponseLogLikelihood
import tempfile
import os
import ghmm as gh
import json
from collections import OrderedDict
import uuid
import time


class HMMRepLib(object):

    hmm_types_available = {
        "qtcc": QTCCHMM,
        "qtcb": QTCBHMM,
        "qtcbc": QTCBCHMM
    }

    __hmm_types_active = {}

    def __init__(self):
        self.__cnt = 0
        self.__hmm_buffer = HMMBuffer(size_limit=50)
        for k, v in self.hmm_types_available.items():
            self.__hmm_types_active[k] = v()

    def print_hmms_available(self):
        l = sorted(self.hmm_types_available)
        print("Types of HMMs that have been included so far are:")
        for i in l:
            print("-", i)

    def _create_hmm(self, **kwargs):
        hmm = self.__hmm_types_active[kwargs["qsr_type"]].get_hmm(
            **kwargs
        )
        xml = self.__create_xml_from_hmm(hmm)

        self.__hmm_buffer[xml] = hmm

        return HMMReqResponseCreate(data=xml, qsr_type=kwargs["qsr_type"])

    def _sample_hmm(self, **kwargs):
        sample = self.__hmm_types_active[kwargs["qsr_type"]].get_samples(
            hmm=self.__load_hmm(xml=kwargs["xml"]),
            **kwargs
        )
        return HMMReqResponseSample(data=json.dumps(sample), qsr_type=kwargs["qsr_type"])

    def _get_log_likelihood(self, **kwargs):
        loglike = self.__hmm_types_active[kwargs["qsr_type"]].get_log_likelihood(
            hmm=self.__load_hmm(xml=kwargs["xml"]),
            **kwargs
        )
        return HMMReqResponseLogLikelihood(data=json.dumps(loglike), qsr_type=kwargs["qsr_type"])

    def __load_hmm(self, xml):
        try:
            t = time.time()
            hmm = self.__hmm_buffer[xml]
            self.__hmm_buffer[xml] = self.__hmm_buffer.pop(xml) # Moving hmm to end of dict to prevent loosing active ones
            elapsed = time.time() - t
            print "TIME BUFFER", elapsed
            return hmm
        except KeyError:
            t = time.time()
            hmm = self.__create_hmm_from_xml(xml=xml)
            self.__hmm_buffer[xml] = hmm
            elapsed = time.time() - t
            print "TIME LOAD", elapsed
            return hmm

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
        print "############### LOADING #####################"
        fd, name = tempfile.mkstemp()
        f = os.fdopen(fd, 'w')
        f.write(xml)
        f.close() # File has to be closed before reading from it. Otherwise this will fail.
        hmm = gh.HMMOpen(fileName=name, filetype=gh.GHMM_FILETYPE_XML)
        os.remove(name)
        return hmm

    def __create_new_uuid(self):
        date = time.strftime("%Y%m%d%H%M%S")
        i = uuid.uuid3(uuid.NAMESPACE_DNS, date+str(self.__cnt))
        print "UUID", i
        self.__cnt += 1
        return i

    def request(self, request_message):
        assert(issubclass(request_message.__class__, HMMRepRequestAbstractclass))
        return request_message.call_function(self)


class HMMBuffer(OrderedDict):
    def __init__(self, *args, **kwds):
        self.size_limit = kwds.pop("size_limit", None)
        OrderedDict.__init__(self, *args, **kwds)
        self._check_size_limit()

    def __setitem__(self, key, value):
        OrderedDict.__setitem__(self, key, value)
        self._check_size_limit()

    def _check_size_limit(self):
        if self.size_limit is not None:
            while len(self) > self.size_limit:
                self.popitem(last=False)
