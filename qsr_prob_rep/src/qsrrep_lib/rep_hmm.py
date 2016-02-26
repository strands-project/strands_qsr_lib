# -*- coding: utf-8 -*-

#from rep_abstractclass import RepAbstractclass
from rep_io import ServiceManager
from rep_io_hmm import HMMRepRequestCreate, HMMRepRequestSample, HMMRepRequestLogLikelihood
from rep_io_hmm import HMMReqResponseCreate, HMMReqResponseSample, HMMReqResponseLogLikelihood
from qsrrep_hmms.qtcc_hmm import QTCCHMM
from qsrrep_hmms.qtcb_hmm import QTCBHMM
from qsrrep_hmms.qtcbc_hmm import QTCBCHMM
from qsrrep_hmms.rcc3_hmm import RCC3HMM
from qsrrep_hmms.generic_hmm import GenericHMM
import ghmm as gh
import json


class RepHMM(object):
    TRANS = "trans"
    EMI ="emi"
    START = "start"

    hmm_types_available = {
        "qtcc": QTCCHMM,
        "qtcb": QTCBHMM,
        "qtcbc": QTCBCHMM,
        "rcc3": RCC3HMM,
        "generic": GenericHMM
    }

    namespace = "hmm"

    def __init__(self):
        pass

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
        hmm = self.hmm_types_available[kwargs["qsr_type"]]().get_hmm(
            **kwargs
        )
        return HMMReqResponseCreate(data=self.__create_dict_from_hmm(hmm), qsr_type=kwargs["qsr_type"])

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
        num_symbols = len(kwargs["lookup_table"]) if kwargs["qsr_type"] == "generic" else self.hmm_types_available[kwargs["qsr_type"]]().get_num_possible_states()
        sample = self.hmm_types_available[kwargs["qsr_type"]]().get_samples(
            hmm=self.__create_hmm_from_dict(dictionary=kwargs["dictionary"], qsr_type=kwargs["qsr_type"], num_symbols=num_symbols),
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
        num_symbols = len(kwargs["lookup_table"]) if kwargs["qsr_type"] == "generic" else self.hmm_types_available[kwargs["qsr_type"]]().get_num_possible_states()
        loglike = self.hmm_types_available[kwargs["qsr_type"]]().get_log_likelihood(
            hmm=self.__create_hmm_from_dict(dictionary=kwargs["dictionary"], qsr_type=kwargs["qsr_type"], num_symbols=num_symbols),
            **kwargs
        )
        return HMMReqResponseLogLikelihood(data=json.dumps(loglike), qsr_type=kwargs["qsr_type"])

    def __create_dict_from_hmm(self, hmm):
        """Creates a dictionary representation of the hmm.

        :param hmm: the ghmm hmm object to be represent as xml

        :return: The dictionary containing the transition, emission, and start probailities
        """
        trans, emi, start = hmm.asMatrices()

        ret = {
            self.TRANS: trans,
            self.EMI: emi,
            self.START: start,
        }

        return ret

    def __create_hmm_from_dict(self, dictionary, qsr_type, num_symbols):
        """Creates a hmm from the xml representation. Not nice to use tempfile
        but not otherwise possible due to hidden code and swig in ghmm.

        :param xml: The xml string

        :return: the ghmm hmm object
        """
        symbols = self.hmm_types_available[qsr_type]().generate_alphabet(num_symbols)
        hmm = gh.HMMFromMatrices(
            symbols,
            gh.DiscreteDistribution(symbols),
            dictionary[self.TRANS],
            dictionary[self.EMI],
            dictionary[self.START]
        )

        return hmm
