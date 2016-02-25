# -*- coding: utf-8 -*-

"""This file defines classes for the interaction with the HMMs. To create your
own please follow these steps:

TODO: Update description

    * Create a request class following the naming scheme: HMMRepRequestFunctionality
    where 'Functionality' is replaced by your new functionality name.
        * Inherit from 'HMMRepRequestAbstractclass'
        * Define the '_const_function_pointer' to use your function in qsrrep_lib.py.
            * Make sure you implemented such a function in there
            * Make th pointer look like the one in 'HMMRepRequestAbstractclass'
            and replace 'my_function' with the function name in qsrrep_lib.py
        * Override '__init__' defiinig a custom finction header and adding the
        variables to the variable 'self.kwargs'
    * Create a response class following the naming scheme 'HMMReqResponseFunctionality'
    where 'Funtionality' should be the same as for the request class.
        * Override the 'get' function to make sure it returns a string (str or
        json dump)
    * Add you new functionality to 'available_services' in the bottom of the file.
        * The string key will be used to create the service name
        * The value should be a list where the first entry is your request class
        and the second the response class.
"""

from abc import ABCMeta, abstractmethod
from rep_io import RepRequestAbstractclass, ReqResponseAbstractclass
import json


class HMMRepRequestAbstractclass(RepRequestAbstractclass):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self):
        self.kwargs = {}


class HMMReqResponseAbstractclass(ReqResponseAbstractclass):
    __metaclass__ = ABCMeta

    def __init__(self, qsr_type, data):
        """
        :param qsr_type: The qqsr this HMM models
        :param data: Gneric data object, as string or json.dump
        """
        self.qsr_type = qsr_type
        self.data = data

    def get_type(self):
        """Getting the type of the qsr this HMM is modelling

        :return: The qsr type as a string
        """
        return self.qsr_type

    def get(self):
        """Getting the resulting data. What this depends on the actual request made.

        :return: the data as a string or json.dump
        """
        return self.data


class HMMRepRequestCreate(HMMRepRequestAbstractclass):

    def __init__(self, qsr_type, qsr_seq, pseudo_transitions=False, lookup_table=None, transition_matrix=None, emission_matrix=None, start_at_zero=False):
        """
        :param qsr_type: The QSR this HMM is modelling
        :param qsr_seq: The list of lists of the QSR state chains
        :param store: Unused. Might leave that to client side.
        """
        super(self.__class__, self).__init__()
        self.kwargs = {
            "qsr_type": qsr_type,
            "qsr_seq": qsr_seq,
            "pseudo_transitions": pseudo_transitions,
            "lookup_table": lookup_table,
            "transition_matrix": transition_matrix,
            "emission_matrix": emission_matrix,
            "start_at_zero": start_at_zero
        }


class HMMReqResponseCreate(HMMReqResponseAbstractclass):

    def get(self):
        """
        :return: The HMM in an distionary representation as a json str
        """
        return json.dumps(super(self.__class__, self).get())


class HMMRepRequestSample(HMMRepRequestAbstractclass):

    def __init__(self, qsr_type, dictionary, max_length, num_samples=1, lookup_table=None):
        """
        :param qsr_type: The QSR this HMM is modelling
        :param dictionary: The HMM in its dictionary json representation
        :param max_length: The maximum length of the sample. This will be kept if at all possible
        :param num_samples: The number of samples to take
        """
        super(self.__class__, self).__init__()
        self.kwargs = {
            "qsr_type": qsr_type,
            "dictionary": dictionary if isinstance(dictionary, dict) else json.loads(dictionary),
            "max_length": max_length,
            "num_samples": num_samples,
            "lookup_table": lookup_table
        }


class HMMReqResponseSample(HMMReqResponseAbstractclass):

    def get(self):
        """
        :return: The list of lists of samples take from the given HMM as a json.dump
        """
        return super(self.__class__, self).get()


class HMMRepRequestLogLikelihood(HMMRepRequestAbstractclass):

    def __init__(self, qsr_type, dictionary, qsr_seq, lookup_table=None):
        """
        :param qsr_type: The QSR this HMM is modelling
        :param dictionary: The HMM in its dictionary json representation
        :param qsr_seq: A list of lists of QSR state chains to check against the given HMM
        """
        super(self.__class__, self).__init__()
        self.kwargs = {
            "qsr_type": qsr_type,
            "dictionary": dictionary if isinstance(dictionary, dict) else json.loads(dictionary),
            "qsr_seq": qsr_seq,
            "lookup_table": lookup_table
        }


class HMMReqResponseLogLikelihood(HMMReqResponseAbstractclass):

    def get(self):
        """
        :return: The accumulated loglikelihood of the given state chains being produced by the given HMM as a json.dump
        """
        return super(self.__class__, self).get()
