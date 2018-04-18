# -*- coding: utf-8 -*-

import pybayes as pb
import numpy as np
from qsrrep_pf.probability_density_functions import PredictionPdf, ObservationPdf, UniIntPdf
from qsrrep_pf.particle_filter_base import ParticleFilter
from collections import OrderedDict
from copy import deepcopy
import time


class ParticleFilterPredictor(object):

    def create(self, **kwargs):
        models = OrderedDict(kwargs["models"])
        states = kwargs["state_lookup_table"]
        x_t = pb.RVComp(2, 'x_t')
        p = PredictionPdf(models=models, states=states, rv=x_t, cond_rv=pb.RVComp(2, 'x_tp'))
        o = ObservationPdf(states=states, models=models, rv=pb.RVComp(2, 'y_t'), cond_rv=[x_t])

        # prepare initial particle density:
        try:
            init_pdf = UniIntPdf(
                np.array([0., 0.]),
                np.array([float(len(states)-1), float(len(models.keys())-1)]),
                cheat=kwargs["ensure_particle_per_state"]
            )
        except ValueError as e:
            print "### [ParticleFilterPredictor] Encountered a problem while creating intial particle distribution:", e
            print "### [ParticleFilterPredictor]  This might happen if there is only one model or one state defined."
            print "### [ParticleFilterPredictor]  Currently defined number of states: %s and number of models: %s" % (str(len(states)),str(len(models.keys()))), models.keys()
            return None

        return {
            "filter": ParticleFilter(kwargs["num_particles"], init_pdf, p, o, starvation_factor=1.-kwargs["starvation_factor"]),
            "models": models.keys(),
            "states": kwargs["state_lookup_table"]
        }

    def predict(self, **kwargs):
        ret = []
        if kwargs["debug"]: start = time.time()
        p = deepcopy(kwargs["filter"].emp.particles)
        for n in range(kwargs["num_steps"]):
            if kwargs["debug"]: start = time.time()
            pn = kwargs["filter"].p_xt_xtp.sample_multiple(p)
            if kwargs["debug"]:
                print "[ParticleFilterPredictor] elapsed time sampling particles.:", time.time() -start
                print "[ParticleFilterPredictor] ################ PARTICLES ###############################################"
                print pn
                print "[ParticleFilterPredictor] ###############################################################"
                

            _,bs,sp,_,bm,mp = self._get_best_state_and_model(pn, **kwargs)
            ret.append((bs,sp,bm,mp))

            if kwargs["debug"]: print "[ParticleFilterPredictor]  total elapsed", time.time() -start
        return ret

    def update(self, **kwargs):
        obs = kwargs["observation"]
        if kwargs["debug"]:
            start = time.time()
            #print("[filter] Obs is ["+str(obs)+"]")   

        obs_ind = kwargs["states"].index(obs)
        #print("[filter] Obs ind ["+str(obs_ind)+"]")

        obs_v = np.array([obs_ind, np.nan])
        
        #print("[filter] Performing bayes with vector ["+str(obs_v)+"]")
        kwargs["filter"].bayes(obs_v)

        #print("[filter] Getting particles ")
        p = kwargs["filter"].emp.particles
        #print("[filter] Particle cloud is: ["+str(p)+"]")
        
        if kwargs["debug"]:
            print "[ParticleFilterPredictor] ###############################################################"
            print "[ParticleFilterPredictor] OBS:", obs, kwargs["states"].index(obs)
            try:
                print "[ParticleFilterPredictor] MODELS:", kwargs["models"]
                print "[ParticleFilterPredictor] MODEL SIZES:", np.bincount(map(int,p[:,1].flatten()), minlength=len(kwargs["models"]))
            except:
                pass

        #print("[filter] Obtaining best state and model")
        _,bs,sp,_,bm,mp = self._get_best_state_and_model(p, **kwargs)

        if kwargs["debug"]: print "[ParticleFilterPredictor] total elapsed", time.time()-start
        return bs,sp,bm,mp

    def _get_best_state_and_model(self, p, **kwargs):
        state_bins = np.bincount(map(int,p[:,0].flatten()))
        model_bins = np.bincount(map(int,p[:,1].flatten()))
        best_state = state_bins.argmax()
        best_model = model_bins.argmax()

        if kwargs["debug"]:
            print "[ParticleFilterPredictor] states,len(state):",state_bins, len(state_bins)
            print "[ParticleFilterPredictor] models,len(model):",model_bins, len(model_bins)
            print "[ParticleFilterPredictor] best_state, best_model,len:",best_state, best_model, len(kwargs["states"])

        state_prob = float(state_bins[best_state])/float(np.sum(state_bins))
        model_prob = float(model_bins[best_model])/float(np.sum(model_bins))

        return best_state, kwargs["states"][best_state], state_prob, best_model, kwargs["models"][best_model], model_prob
