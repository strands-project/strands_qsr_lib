# -*- coding: utf-8 -*-

import pybayes as pb
import numpy as np


class UniIntPdf(pb.UniPdf):
    def __init__(self, a, b, rv = None, cheat=False):
        super(UniIntPdf, self).__init__(a, b, rv)
        self.cheat = cheat

    def sample(self, cond = None):
        return np.round(super(UniIntPdf, self).sample(cond))

    def samples(self, n, cond = None):
        # Cheating: ensuring that every state in every model has at least one particle
        # if there are enough particles. Otherwise later models will be empty. If there
        # are not enough particles, you have much bigger problems than this anyway.
        models = [np.repeat(x,self.b[0]+1-self.a[0]) for x in np.arange(self.a[1], self.b[1]+1)]
        states = np.arange(self.a[0], self.b[0]+1)
        spam = np.array([])
        for x in models:
            spam = np.append(spam, np.append(states, x).reshape(2,-1).T)
        spam = spam.reshape(-1,2)
        ret = np.empty((n, self.shape()))
        for i in range(n):
            ret[i, :] = self.sample(cond) if not self.cheat else spam[i, :] if i < spam.shape[0] else self.sample(cond)
        return ret


class PredictionPdf(pb.CPdf):
    key = "prediction"

    def __init__(self, models, states, rv=None, cond_rv=None):
        self._set_rvs(2, rv, 2, cond_rv)
        self.models = models
        self.states = states
        self.mu = 0

    def mean(self, cond=None):
        return self.mu

    def _set_mean(self, cond):
        self._check_cond(cond)
        self.mu = np.round(cond[0])
        return True

    def variance(self, cond=None):
        return 0. # No clue...

    def _get_current_state_transitions(self, model):
        return np.array(self.models[self.models.keys()[int(model)]][self.key])[self.mean(),:]

    def sample(self, cond=None):
        return self.samples(1, cond=cond)

    def samples(self, n, cond=None):
        self._set_mean(cond)
        st = self._get_current_state_transitions(cond[1])
        # Much quicker than repeatedly calling sample like in the default implementation
        # sample_multiple is even quicker
        try:
            r = np.append(np.random.choice(np.arange(0,len(st)), p=st, size=n), np.repeat(cond[1], n)).reshape(2,-1).T
        except IndexError:
            return cond
        return r

    def sample_multiple(self, particles, cond=None):
        """ Sampling for all given particles at the same time using the model
        assigned at creation.

        :param particles The numpy array of particle for which to sample based on the model
        :param cond unused

        :return Same size numpy array with new particles

        """
        res = np.array([])
        for m in range(len(self.models.keys())):
            # Create a list of all current particles of that model
            p = particles[particles[:,1]==m][:,0].astype(int)
            # Create transition matrix for all those particles, rows can be
            # duplicated if more than one particle for the specific state exists
            t = np.array(self.models[self.models.keys()[m]][self.key])[p,:]
            #Create a uniform random matrix of the same shape as the trans matrix
            r = np.random.rand(*t.shape)
            # Subtract the two to create our random sampling and find the argmax
            l = np.subtract(t,r)
            p = l.argmax(axis=1)
            # Recreate particle list by appending the model number to each particle
            res = np.append(res, np.append(p, np.repeat(m, len(p))).reshape(2,-1).T).reshape(-1,2)
        return res

    def eval_log(self, x, cond=None):
        self._check_x(x)
        self._set_mean(cond)
        st = self._get_current_state_transitions(cond[1])
        return np.log(st[x[0]])

    def eval_multiple(self, x, particles, cond=None):
        """ Sampling for all given particles at the same time using the model
        assigned at creation.

        :param x the observed particle, model number is ignored
        :param particles The numpy array of particle for which to sample based on the model
        :param cond unused

        :return Same size numpy array with new particles

        """
        res = np.zeros((len(particles),))
        current_models = self.models.keys()
        for m in range(len(current_models)):
            # Create a list of all current particles of that model
            indexs = (particles[:,1]==m)

            # Lookup observation probability for all particles given x
            p_i = particles[indexs][:,0].astype(int)#.tolist()
            x0 = int(x[0])
            vector_obs = np.array(self.models[current_models[m]][self.key])
           
            o = vector_obs[p_i,x0]
            
            # Create log likelihood list
            res[indexs] = np.log(o)
        return res

    def to_string(self, x):
        self._check_x(x)
        return self.states[int(x[0])], self.models.keys()[int(x[1])]


class ObservationPdf(PredictionPdf):
    key = "observation"

    def __init__(self, states, models, rv=None, cond_rv=None):
        super(self.__class__, self).__init__(models, states, rv, cond_rv)


class EmpPdf(pb.EmpPdf):
    r"""An abstraction of empirical probability density functions that provides common methods such
    as weight normalisation. Extends :class:`Pdf`.

    :var numpy.ndarray weights: 1D array of particle weights
       :math:`\omega_i >= 0 \forall i; \quad \sum \omega_i = 1`
    """

    def __init__(self, init_particles, rv = None, starvation_factor=0.9):
        super(self.__class__, self).__init__(init_particles=init_particles, rv=rv)
        self.__starvation_factor = starvation_factor

    def get_resample_indices(self):
        r"""Calculate first step of resampling process (dropping low-weight particles and
        replacing them with more weighted ones.

        :return: integer array of length n: :math:`(a_1, a_2 \dots a_n)` where
            :math:`a_i` means that particle at ith place should be replaced with particle
            number :math:`a_i`
        :rtype: :class:`numpy.ndarray` of ints

        *This method doesnt modify underlying pdf in any way - it merely calculates how
        particles should be replaced.*
        """
        n = self.weights.shape[0]
        cum_weights = np.cumsum(self.weights)

        m = int(np.round(float(n)*self.__starvation_factor))
        u = np.empty(m)
        fuzz = np.random.uniform()
        for i in range(m):
            u[i] = (i + fuzz) / m

        # calculate number of babies for each particle
        baby_indices = np.empty(n, dtype=int)  # index array: a[i] contains index of
        # original particle that should be at i-th place in new particle array
        j = 0
        for i in range(m):
            while u[i] > cum_weights[j]:
                j += 1
            baby_indices[i] = j

        for i in np.arange(m, n):
            baby_indices[i] = -1
        return baby_indices

    def resample(self, init_pdf):
        super(EmpPdf, self).resample()
        n = self.weights.shape[0]
        m = int(np.round(float(n)*self.__starvation_factor))
        for i in np.arange(m, n):
            self.particles[i] = init_pdf.sample()
