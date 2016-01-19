# -*- coding: utf-8 -*-

import pybayes as pb
import numpy as np
#import math
import time

from qsrrep_pf.probability_density_functions import EmpPdf


class ParticleFilter(pb.Filter):
    r"""Standard particle filter implementation with resampling.

    Specifying proposal density is currently unsupported, but planned; speak up if you want it!
    Posterior pdf is represented using :class:`~pybayes.pdfs.EmpPdf` and takes following form:

    .. math:: p(x_t|y_{1:t}) = \sum_{i=1}^n \omega_i \delta ( x_t - x_t^{(i)} )
    """

    def __init__(self, n, init_pdf, p_xt_xtp, p_yt_xt, starvation_factor=0.9, debug=False):
        r"""Initialise particle filter.

        :param int n: number of particles
        :param init_pdf: either :class:`~pybayes.pdfs.EmpPdf` instance that will be used
            directly as a posterior (and should already have initial particles sampled) or
            any other probability density which initial particles are sampled from
        :type init_pdf: :class:`~pybayes.pdfs.Pdf`
        :param p_xt_xtp: :math:`p(x_t|x_{t-1})` cpdf of state in *t* given state in *t-1*
        :type p_xt_xtp: :class:`~pybayes.pdfs.CPdf`
        :param p_yt_xt: :math:`p(y_t|x_t)` cpdf of observation in *t* given state in *t*
        :type p_yt_xt: :class:`~pybayes.pdfs.CPdf`
        """
        self.__initial_sample = True
        self.debug = debug

        if not isinstance(n, int) or n < 1:
            raise TypeError("n must be a positive integer")
        if not isinstance(init_pdf, pb.Pdf):
            raise TypeError("init_pdf must be an instance ot the Pdf class")
        if not isinstance(p_xt_xtp, pb.CPdf) or not isinstance(p_yt_xt, pb.CPdf):
            raise TypeError("both p_xt_xtp and p_yt_xt must be instances of the CPdf class")

        self.init_pdf = init_pdf
        dim = init_pdf.shape()  # dimension of state
        if p_xt_xtp.shape() != dim or p_xt_xtp.cond_shape() < dim:
            raise ValueError("Expected shape() and cond_shape() of p_xt_xtp will "
                + "be (respectively greater than) {0}; ({1}, {2}) given.".format(dim,
                p_xt_xtp.shape(), p_xt_xtp.cond_shape()))
        self.p_xt_xtp = p_xt_xtp
        if p_yt_xt.cond_shape() != dim:
            raise ValueError("Expected cond_shape() of p_yt_xt will be {0}; {1} given."
                .format(dim, p_yt_xt.cond_shape()))
        self.p_yt_xt = p_yt_xt

        if isinstance(init_pdf, EmpPdf):
            self.emp = init_pdf  # use directly
        else:
            self.emp = EmpPdf(init_pdf.samples(n), starvation_factor=starvation_factor)

    def bayes(self, yt, cond = None):
        r"""Perform Bayes rule for new measurement :math:`y_t`; *cond* is ignored.

        :param numpy.ndarray cond: optional condition that is passed to :math:`p(x_t|x_{t-1})`
          after :math:`x_{t-1}` so that is can be rewritten as: :math:`p(x_t|x_{t-1}, c)`.

        The algorithm is as follows:

        1. generate new particles: :math:`x_t^{(i)} = \text{sample from }
           p(x_t^{(i)}|x_{t-1}^{(i)}) \quad \forall i`
        2. recompute weights: :math:`\omega_i = p(y_t|x_t^{(i)})
           \omega_i \quad \forall i`
        3. normalise weights
        4. resample particles
        """
#        bla = []
#        bla0 = []
#        bla1 = []
#        loglike = []

        if not self.__initial_sample: # Only sample if not the first round
            self.emp.particles = self.p_xt_xtp.sample_multiple(self.emp.particles)
        self.emp.weights = np.multiply(self.emp.weights, np.exp(self.p_yt_xt.eval_multiple(yt, self.emp.particles)))

#        for i in range(self.emp.particles.shape[0]):
#            # generate new ith particle:
#            if not self.__initial_sample: # Only sample if not the first round
#                self.emp.particles[i] = self.p_xt_xtp.sample(self.emp.particles[i])
#
#            # recompute ith weight:
#            self.emp.weights[i] *= math.exp(self.p_yt_xt.eval_log(yt, self.emp.particles[i]))


#            loglike.append(self.p_yt_xt.eval_log(yt, self.emp.particles[i]))

#            if self.debug:
#                bla.append(np.append(self.emp.particles[i], self.emp.weights[i]))
#                if self.emp.particles[i][1] == 0: bla0.append(self.emp.particles[i][0])
#                else: bla1.append(self.emp.particles[i][0])

#        print "LIKELIHOODS:", max(loglike), min(loglike), np.mean(loglike)
#        if self.debug:
#            bla = np.array(bla)
#            b = np.ascontiguousarray(bla).view(np.dtype((np.void, bla.dtype.itemsize * bla.shape[1])))
#            _, idx = np.unique(b, return_index=True)
#
#            print np.array([[int(x[0]), int(x[1]), int(x[2]*1000000.)] for x in bla[idx]], dtype=int)
#            print "PARTICLE 0", np.bincount(bla0)
#            print "PARTICLE 1", np.bincount(bla1)


#        p = self.emp.particles
#        print "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
#        print "MODEL SIZES:", np.bincount(map(int,p[:,1].flatten()), minlength=2)
#        print "STATES:", np.bincount(map(int,p[:,0].flatten())), len(np.bincount(map(int,p[:,0].flatten())))
#        print "STATES model 0:", np.bincount(map(int,p[np.where(p[:,1] == 0),0].flatten())), len(np.bincount(map(int,p[np.where(p[:,1] == 0),0].flatten())))
#        print "STATES model 1:", np.bincount(map(int,p[np.where(p[:,1] == 1),0].flatten())), len(np.bincount(map(int,p[np.where(p[:,1] == 1),0].flatten())))
#        print "----------------------------------------------------------------"
#        print "WEIGHTS", self.emp.weights

        # assure that weights are normalised
        self.emp.normalise_weights()
        # resample
        self.emp.resample(self.init_pdf)

        self.__initial_sample = False
        return True

    def posterior(self):
        return self.emp
