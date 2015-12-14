# -*- coding: utf-8 -*-

#from rep_abstractclass import RepAbstractclass
from rep_io import ServiceManager
from rep_io_hmm import HMMRepRequestCreate, HMMRepRequestSample, HMMRepRequestLogLikelihood
from rep_io_hmm import HMMReqResponseCreate, HMMReqResponseSample, HMMReqResponseLogLikelihood
from qsrrep_hmms.qtcc_hmm import QTCCHMM
from qsrrep_hmms.qtcb_hmm import QTCBHMM
from qsrrep_hmms.qtcbc_hmm import QTCBCHMM
from qsrrep_hmms.rcc3_hmm import RCC3HMM
from collections import OrderedDict
import os
import tempfile
import ghmm as gh
import time
import uuid
import json


class RepHMM(object):

    hmm_types_available = {
        "qtcc": QTCCHMM,
        "qtcb": QTCBHMM,
        "qtcbc": QTCBCHMM,
        "rcc3": RCC3HMM
    }

    __hmm_types_active = {}

    namespace = "hmm"

    def __init__(self):
        # Activate hmms
        self.__hmm_buffer = HmmBuffer(size_limit=50)
        for k, v in self.hmm_types_available.items():
            self.__hmm_types_active[k] = v()

    @ServiceManager.service_function(namespace, HMMRepRequestCreate, HMMReqResponseCreate)
    def create(self, **kwargs):
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

    @ServiceManager.service_function(namespace, HMMRepRequestSample, HMMReqResponseSample)
    def sample(self, **kwargs):
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

    @ServiceManager.service_function(namespace, HMMRepRequestLogLikelihood, HMMReqResponseLogLikelihood)
    def log_likelihood(self, **kwargs):
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
            hmm = self.__hmm_buffer[xml]
            self.__hmm_buffer[xml] = self.__hmm_buffer.pop(xml) # Moving hmm to end of dict to prevent loosing active ones
            return hmm
        except KeyError:
            hmm = self.__create_hmm_from_xml(xml=xml)
            self.__hmm_buffer[xml] = hmm
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


class HmmBuffer(OrderedDict):
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