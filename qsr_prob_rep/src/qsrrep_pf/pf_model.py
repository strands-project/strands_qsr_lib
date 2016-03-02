# -*- coding: utf-8 -*-

import numpy as np
from probability_density_functions import PredictionPdf, ObservationPdf
import json


class PfModel(object):
    def __init__(self):
        self._model = {}

    def add_model(self, name, pred, obs):
        """Add a model to the particle filter model representation.

        :param name: the name of the model, used to idenify it afterwards
        :param pred: the prediction MxM matrix as a list or numpy array
        :param obs: the observation scoring MxM matrix as a list or numpy array
        """

        if name in self._model:
            raise KeyError("Key '%s' already in model." % name)

        pred = np.array(pred)
        obs = np.array(obs)

        if pred.shape != obs.shape:
            raise IndexError("Prediction and observation matrix have to have the same shape.")

        self._model[name] = {
            PredictionPdf.key: pred.tolist(),
            ObservationPdf.key: obs.tolist()
        }

    def add_prediction_matrix(self, name, pred):
        """Add a prediction matrix to an existing model. If model does not exists
        yet, a new one will be created. Overrides existing prediction matrix if
        model with the same name exists.

        :param name: the name of the model, used to idenify it afterwards
        :param pred: the prediction MxM matrix as a list or numpy array
        """
        if not name in self._model:
            self._model[name] = {}

        pred = np.array(pred)
        self._model[name][PredictionPdf.key] = pred.tolist()

    def add_observation_matrix(self, name, obs):
        """Add an observation matrix to an existing model. If model does not exists
        yet, a new one will be created. Overrides existing observation matrix if
        model with the same name exists.

        :param name: the name of the model, used to idenify it afterwards
        :param obs: the observation scoring MxM matrix as a list or numpy array
        """
        if not name in self._model:
            self._model[name] = {}

        obs = np.array(obs)
        self._model[name][ObservationPdf.key] = obs.tolist()

    def __check_model(self):
        for k,v in self._model.items():
            if PredictionPdf.key not in v:
                raise KeyError("'%s' matrix not in dictionary for model '%s'" % (PredictionPdf.key, k))
            if ObservationPdf.key not in v:
                raise KeyError("'%s' matrix not in dictionary for model '%s'" % (ObservationPdf.key, k))

            pred = np.array(v[PredictionPdf.key])
            obs = np.array(v[ObservationPdf.key])
            if pred.shape != obs.shape:
                raise IndexError("Prediction and observation matrix have to have the same shape for model '%s'." %k)

    def to_string(self):
        """
        :return: json string of model
        """
        self.__check_model()
        return json.dumps(self._model)

    def get(self):
        """
        :return: the model dict
        """
        self.__check_model()
        return self._model
