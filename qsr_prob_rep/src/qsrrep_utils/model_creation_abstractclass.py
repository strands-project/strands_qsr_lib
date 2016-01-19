# -*- coding: utf-8 -*-

from abc import abstractmethod, ABCMeta
import numpy as np

def return_numpy_array(func):
    def wrapper(*args, **kwargs):
        return np.array(func(*args, **kwargs))
    return wrapper


class ModelCreationAbstractclass(object):
    """Abstract class for Particle Filter model generation"""
    __metaclass__ = ABCMeta

    def __init__(self):
        self.execute_function = {
            "prediction": self.create_prediction_model,
            "observation": self.create_observation_model,
            "lookup": self.create_states
        }

    @abstractmethod
    def create_states(self, **kwargs):
        return

    @abstractmethod
    def create_prediction_model(self, **kwargs):
        return

    @abstractmethod
    def create_observation_model(self, **kwargs):
        return