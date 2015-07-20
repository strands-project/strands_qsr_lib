# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 14:19:25 2015

@author: cdondrup

"""

from qsrrep_hmms.qtcc_hmm import QTCCHMM
from qsrrep_hmms.qtcb_hmm import QTCBHMM
from qsrrep_hmms.qtcbc_hmm import QTCBCHMM
from rep_io import HMMRepRequestAbstractclass, HMMReqResponseCreate, HMMReqResponseSample, HMMReqResponseLogLikelihood
import tempfile
import os
import ghmm as gh
import json
from collections import OrderedDict
import uuid
import time


class ProbRepLib(object):
    """This class provides the main functionalities of the library by calling
    the appropriate functions in the HMM implementations. None of the providing
    functions should be called directly but via the function pointer '_const_function_pointer'
    in rep_io.py in the request classes. The function 'request' in here will
    take care of executing it.

    Adding a new HMM type:
        * Add an entry to 'hmm_types_available', following the examples given and import the correct class

    """

    hmm_types_available = {
        "qtcc": QTCCHMM,
        "qtcb": QTCBHMM,
        "qtcbc": QTCBCHMM
    }

    __hmm_types_active = {}

    def __init__(self):
        self.__cnt = 0
        self.__hmm_buffer = RepBuffer(size_limit=50)
        for k, v in self.hmm_types_available.items():
            self.__hmm_types_active[k] = v()

    def print_hmms_available(self):
        """Prints all the available HMM types."""
        l = sorted(self.hmm_types_available)
        print("Types of HMMs that have been included so far are:")
        for i in l:
            print "-", i

    def _create_hmm(self, **kwargs):
        """Creates a new HMM by calling the get_hmm function in hmm_abstractclass.py.
        Called by the 'HMMRepRequestCreate' request class in rep_io.py.

        :param kwargs:
            * qsr_type: The type of HMM, needs to be a key in 'hmm_types_available'
            * qsr_seq: The list of lists of the QSR state chains
            * store: Unused. Might leave that to client side.

        :return: A 'HMMReqResponseCreate' object containing the resulting data

        """
        hmm = self.__hmm_types_active[kwargs["qsr_type"]].get_hmm(
            **kwargs
        )
        xml = self.__create_xml_from_hmm(hmm)

        self.__hmm_buffer[xml] = hmm

        return HMMReqResponseCreate(data=xml, qsr_type=kwargs["qsr_type"])

    def _sample_hmm(self, **kwargs):
        """Generates samples from the given HMM by calling the get_samples
        function in hmm_abstractclass.py.
        Called by the 'HMMRepRequestSamples' request class in rep_io.py.

        :param kwargs:
            * qsr_type: The type of HMM, needs to be a key in 'hmm_types_available'
            * xml: The xml representation of the HMM from which to sample
            * max_length: The maximum length of the sample. This will be kept if at all possible
            * num_samples: The number of samples to take


        :return: A 'HMMReqResponseSamples' object containing the resulting data

        """
        sample = self.__hmm_types_active[kwargs["qsr_type"]].get_samples(
            hmm=self.__load_hmm(xml=kwargs["xml"]),
            **kwargs
        )
        return HMMReqResponseSample(data=json.dumps(sample), qsr_type=kwargs["qsr_type"])

    def _get_log_likelihood(self, **kwargs):
        """Calculates the cummulative loglikelihood for the given state chains and the given HMM
        by calling the get_log_likelihood function in hmm_abstractclass.py.
        Called by the 'HMMRepRequestLogLikelihood' request class in rep_io.py.

        :param kwargs:
            * qsr_type: The type of HMM, needs to be a key in 'hmm_types_available'
            * xml: The xml representation of the HMM from which to sample
            * qsr_seq: A list of lists of QSR state chains to check against the given HMM


        :return: A 'HMMReqResponseLogLikelihood' object containing the resulting data

        """
        loglike = self.__hmm_types_active[kwargs["qsr_type"]].get_log_likelihood(
            hmm=self.__load_hmm(xml=kwargs["xml"]),
            **kwargs
        )
        return HMMReqResponseLogLikelihood(data=json.dumps(loglike), qsr_type=kwargs["qsr_type"])

    def __load_hmm(self, xml):
        """Loads the ghmm hmm object from an xml representation. The xml is used
        as the key in the active HMMs buffer. If found, the object will be loaded
        from that, otherwise a nasty tempfile loading is necessary due to hidden
        functions made available via swig.

        :param xml: The xml representation of the HMM

        :return: The ghmm hmm object
        """
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

        :param hmm: the ghmm hmm object to be represent as xml

        :return: The xml string
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

        :param xml: The xml string

        :return: the ghmm hmm object
        """
        fd, name = tempfile.mkstemp()
        f = os.fdopen(fd, 'w')
        f.write(xml)
        f.close() # File has to be closed before reading from it. Otherwise this will fail.
        hmm = gh.HMMOpen(fileName=name, filetype=gh.GHMM_FILETYPE_XML)
        os.remove(name)
        return hmm

    def __create_new_uuid(self):
        """Creates a UUID from the current time and date plus a counter.
        Current not used.
        """
        date = time.strftime("%Y%m%d%H%M%S")
        i = uuid.uuid3(uuid.NAMESPACE_DNS, date+str(self.__cnt))
        self.__cnt += 1
        return i

    def request(self, request_message):
        """The magic function that calls the appropiate function beased on
        '_const_function_pointer' in the requestclass definition.

        :param request_message: The request message object which has to be an instance of one of the request classes in rep_io.py and has to inherit from 'HMMRepRequestAbstractclass'

        :return: The resulting respons message that corresponds to the request message.
        """
        assert(issubclass(request_message.__class__, HMMRepRequestAbstractclass))
        return request_message.call_function(self)


class RepBuffer(OrderedDict):
    """This class extends the OrderedDict to have a fixed size. This is used to
    keep the currently active HMMs in a buffer as ghmm objects instead of loading
    them using the tempfile way.
    """

    def __init__(self, *args, **kwds):
        """
        :param kwds:
            * size_limit: The maximum number of entries. If this is exceeded, the elements inserted first will be popped until the limit is reached.
        """
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
