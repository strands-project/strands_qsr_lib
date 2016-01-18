#!/usr/bin/env python
PKG = 'qsr_prob_rep'
NAME = 'pf_tester'

import rospy
import sys
import unittest
from roslib.packages import find_resource
from qsrrep_ros.ros_client import ROSClient
from qsrrep_lib.rep_io_pf import PfRepRequestCreate,PfRepRequestPredict, PfRepRequestUpdate, PfRepRequestList, PfRepRequestRemove
from qsrrep_pf.pf_model import PfModel
import json
import numpy as np


class TestPf(unittest.TestCase):
    CROSSING_OBS   = find_resource(PKG, 'crossing.obs')[0]
    CROSSING_PRED  = find_resource(PKG, 'crossing.pred')[0]
    PASSBY_OBS     = find_resource(PKG, 'passby.obs')[0]
    PASSBY_PRED    = find_resource(PKG, 'passby.pred')[0]
    STATES         = find_resource(PKG, 'qtch.states')[0]

    filters = map(str, range(5))

    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)

        rospy.init_node(NAME)

        self.r = ROSClient()

    def _create_model(self, obs, pred):
        m = PfModel()
        for f in pred:
            name = f.split('/')[-1].split('.')[0]
            with open(f, 'r') as a:
                m.add_prediction_matrix(name, np.loadtxt(a))

        for f in obs:
            name = f.split('/')[-1].split('.')[0]
            with open(f, 'r') as a:
                m.add_observation_matrix(name, np.loadtxt(a))

        return m.get()

    def _load_lookup(self, filename):
        with open(filename, 'r') as f:
            return json.load(f)

    def _create_pf(self, pred, obs, states, num_particles=1000, uuid="", starvation_factor=0.1, ensure_particle_per_state=True, debug=True):
        models = self._create_model(pred, obs)
        states = self._load_lookup(states)
        d = self.r.call_service(
            PfRepRequestCreate(
                num_particles=num_particles,
                models=models,
                state_lookup_table=states,
                starvation_factor=starvation_factor,
                ensure_particle_per_state=ensure_particle_per_state,
                debug=debug,
                uuid=uuid if uuid != "" else None
            )
        )
        return d

    def _predict(self, uuid, num_steps=5, debug=True):
        p = self.r.call_service(
            PfRepRequestPredict(
                uuid=uuid,
                num_steps=num_steps,
                debug=debug
            )
        )
        return p

    def _update(self, uuid, obs, debug=True):
        p = self.r.call_service(
            PfRepRequestUpdate(
                uuid=uuid,
                observation=obs,
                debug=debug
            )
        )
        return p

    def _list(self):
        p = self.r.call_service(
            PfRepRequestList()
        )
        return p

    def _remove(self, uuid):
        p = self.r.call_service(
            PfRepRequestRemove(
                uuid=uuid
            )
        )
        return p

    def test1_pf_create(self):
        res = []
        for uuid in self.filters:
            res.append(self._create_pf(
                pred=[self.CROSSING_PRED, self.PASSBY_PRED],
                obs=[self.CROSSING_OBS, self.PASSBY_OBS],
                states=self.STATES,
                uuid=uuid
            ))
        self.assertEqual(map(str, res), self.filters)

    def test2_pf_predict(self):
        res = []
        for uuid in self.filters:
            res.append(self._predict(
                uuid=uuid,
                num_steps=1
            )[0])
        self.assertEqual(len(res), len(self.filters))

    def test3_pf_update(self):
        res = []
        for uuid in self.filters:
            res.append(self._update(
                uuid=uuid,
                obs='-,-,-'
            ))
        self.assertEqual(len(res), len(self.filters))

    def test4_pf_list(self):
        res = self._list()
        self.assertEqual(len(res), len(self.filters))

    def test5_pf_remove(self):
        for uuid in self.filters:
            self._remove(uuid)
        self.assertFalse(len(self._list()))


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestPf, sys.argv)
