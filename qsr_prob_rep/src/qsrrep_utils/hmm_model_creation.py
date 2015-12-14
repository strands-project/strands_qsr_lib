# -*- coding: utf-8 -*-

from qsrrep_utils.model_creation_abstractclass import ModelCreationAbstractclass, return_numpy_array
import ghmm as gh


class HMMModelCreation(ModelCreationAbstractclass):

    ex = Exception("For the model creation from HMM, input is required. Please select a HMM file using [-i].")
    open_hmm = lambda _,i: gh.HMMOpen(fileName=i, filetype=gh.GHMM_FILETYPE_XML)

    def create_states(self, **kwargs):
        raise NotImplementedError("State look-up table creation for HMM currently not available.")

    @return_numpy_array
    def create_prediction_model(self, **kwargs):
        if kwargs["input"] == None:
            raise self.ex
        hmm = self.open_hmm(kwargs["input"])
        trans, _, _ = hmm.asMatrices()
        return trans

    @return_numpy_array
    def create_observation_model(self, **kwargs):
        if kwargs["input"] == None:
            raise self.ex
        hmm = self.open_hmm(kwargs["input"])
        _, emi, _ = hmm.asMatrices()
        return emi