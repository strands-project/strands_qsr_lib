# -*- coding: utf-8 -*-

"""
TODO: Update description

"""

from abc import ABCMeta, abstractmethod
from rep_io import RepRequestAbstractclass, ReqResponseAbstractclass


class PfRepRequestAbstractclass(RepRequestAbstractclass):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self):
        self.kwargs = {}


class PfReqResponseAbstractclass(ReqResponseAbstractclass):
    __metaclass__ = ABCMeta

    def __init__(self, data):
        """
        :param qsr_type: The qqsr this HMM models
        :param data: Gneric data object, as string or json.dump
        """
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


class PfRepRequestCreate(PfRepRequestAbstractclass):

    def __init__(self, num_particles, models, state_lookup_table, uuid=None, starvation_factor=0.1, ensure_particle_per_state=False, debug=False):
        """
        TODO: Create description
        """
        super(self.__class__, self).__init__()
        self.kwargs = {
            "num_particles": num_particles,
            "models": models,
            "state_lookup_table": state_lookup_table,
            "uuid": uuid,
            "starvation_factor": starvation_factor,
            "ensure_particle_per_state": ensure_particle_per_state,
            "debug": debug
        }


class PfReqResponseCreate(PfReqResponseAbstractclass):

    def get(self):
        """
        :return: a UUID for the generated filter
        """
        return str(super(self.__class__, self).get())


class PfRepRequestUpdate(PfRepRequestAbstractclass):

    def __init__(self, uuid, observation, debug=False):
        """
        TODO: Create description
        """
        super(self.__class__, self).__init__()
        self.kwargs = {
            "uuid": uuid,
            "observation": observation,
            "debug": debug
        }


class PfReqResponseUpdate(PfReqResponseAbstractclass):

    def get(self):
        """
        :return: The list of lists of samples take from the given HMM as a json.dump
        """
        return super(self.__class__, self).get()


class PfRepRequestPredict(PfRepRequestAbstractclass):

    def __init__(self, uuid, num_steps, debug=False):
        """
        TODO: Create description
        """
        super(self.__class__, self).__init__()
        self.kwargs = {
            "uuid": uuid,
            "num_steps": num_steps,
            "debug": debug
        }


class PfReqResponsePredict(PfReqResponseAbstractclass):

    def get(self):
        """
        :return: The accumulated loglikelihood of the given state chains being produced by the given HMM as a json.dump
        """
        return super(self.__class__, self).get()


class PfRepRequestRemove(PfRepRequestAbstractclass):

    def __init__(self, uuid):
        """
        TODO: Create description
        """
        super(self.__class__, self).__init__()
        self.kwargs = {
            "uuid": uuid
        }


class PfReqResponseRemove(PfReqResponseAbstractclass):
    SUCCESS = 1
    FAILURE = 0

    def get(self):
        """
        :return: The outcome as a json.dump. SUCCESS or FAILURE
        """
        return super(self.__class__, self).get()


class PfRepRequestList(PfRepRequestAbstractclass):

    def __init__(self):
        """
        TODO: Create description
        """
        super(self.__class__, self).__init__()
        self.kwargs = {}


class PfReqResponseList(PfReqResponseAbstractclass):

    def get(self):
        """
        :return: a list of uuids and model keys for all particle filters
        """
        return super(self.__class__, self).get()
