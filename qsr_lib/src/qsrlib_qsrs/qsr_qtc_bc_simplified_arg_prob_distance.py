# -*- coding: utf-8 -*-
"""Example that shows how to implement QSR makers.

:Author: Christan Dondrup <cdondrup@lincoln.ac.uk>
:Organization: University of Lincoln
:Date: 10 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
:Notes: future extension to handle polygons, to do that use matplotlib.path.Path.contains_points
        although might want to have a read on the following also...
        http://matplotlib.1069221.n5.nabble.com/How-to-properly-use-path-Path-contains-point-td40718.html
"""

from __future__ import division
from qsrlib_qsrs.qsr_qtc_bc_simplified import QSR_QTC_BC_Simplified
from qsrlib_qsrs.qsr_arg_prob_relations_distance import QSR_Arg_Prob_Relations_Distance
import numpy as np
from qsrlib_io.world_qsr_trace import *
from random import uniform


# QSR_QTC_BC_Simplified has to be first so it finds the correct functions implementations from QTC
class QSR_QTC_BC_Simplified_Arg_Prob_Distance(QSR_QTC_BC_Simplified, QSR_Arg_Prob_Relations_Distance):
    """Make default QSRs and provide an example for others"""
    def __init__(self):
        super(QSR_QTC_BC_Simplified_Arg_Prob_Distance, self).__init__()
        self.qtc_type = "bc"
        self.all_possible_relations = self.return_all_possible_state_combinations()[0]
        self._unique_id = "qtcbcs_argprobd"
        self.prev_dist = ''
        self._qsr_params_defaults = {
            "distance_threshold": 'und',
            "quantisation_factor": 0.0,
            "validate": True,
            "no_collapse": False,
            "qsr_relations_and_values": {}
        }
        self.allowed_value_types = (tuple, list)
        self.value_sort_key = lambda x: x[1][0] # Sort by first element in value tuple, i.e. mean

    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, req_params, **kwargs):
        ret = World_QSR_Trace(qsr_type=self._unique_id)
        self.set_qsr_relations_and_values(qsr_params["qsr_relations_and_values"])
        qtc_sequence = {}
        for t, tp in zip(timestamps[1:], timestamps):
            world_state_now = world_trace.trace[t]
            world_state_previous = world_trace.trace[tp]
            if set(world_state_now.objects.keys()) != set(world_state_previous.objects.keys()):
                ret.add_empty_world_qsr_state(t)
                continue # Objects have to be present in both timestamps
            qsrs_for = self._process_qsrs_for(world_state_now.objects.keys(), req_params["dynamic_args"])
            for o1_name, o2_name in qsrs_for:
                between = str(o1_name) + "," + str(o2_name)
                qtc = np.array([], dtype=int)
                k = [world_state_previous.objects[o1_name].x,
                     world_state_previous.objects[o1_name].y,
                     world_state_now.objects[o1_name].x,
                     world_state_now.objects[o1_name].y]
                l = [world_state_previous.objects[o2_name].x,
                     world_state_previous.objects[o2_name].y,
                     world_state_now.objects[o2_name].x,
                     world_state_now.objects[o2_name].y]
                qtc = self._create_qtc_representation(
                    k,
                    l,
                    qsr_params["quantisation_factor"]
                )
                distance = self._compute_distance((world_state_now.objects[o1_name], world_state_now.objects[o2_name]), qtc)

                try:
                    qtc_sequence[between]["qtc"] = np.append(
                        qtc_sequence[between]["qtc"],
                        qtc
                    ).reshape(-1,4)
                    qtc_sequence[between]["distances"] = np.append(
                            qtc_sequence[between]["distances"],
                            distance
                    )
                except KeyError:
                    qtc_sequence[between] = {
                        "qtc": qtc,
                        "distances": np.array([distance])
                    }

        for between, qtcbc in qtc_sequence.items():
            qtcbc["qtc"] = self._create_bc_chain(qtcbc["qtc"], qtcbc["distances"], qsr_params["distance_threshold"])
            if not qsr_params["no_collapse"]:
                qtcbc["qtc"] = self._collapse_similar_states(qtcbc["qtc"])
            if qsr_params["validate"]:
                qtcbc["qtc"] = self._validate_qtc_sequence(qtcbc["qtc"])
            for idx, q in enumerate(qtcbc["qtc"]):
                qsr = QSR(
                    timestamp=idx+1,
                    between=between,
                    qsr=self.qtc_to_output_format(q)
                )
                ret.add_qsr(qsr, idx+1)

        return ret

    def _create_bc_chain(self, qtc, distances, distance_threshold):
        ret = np.array([])
        if len(qtc.shape) == 1:
            qtc = [qtc]
        for dist, state in zip(distances, qtc):
            if self.all_possible_relations.index(dist) > self.all_possible_relations.index(distance_threshold):
                ret = np.append(ret, np.append(state[0:2],[np.nan,np.nan]), axis=0)
            else:
                ret = np.append(ret, state, axis=0)

        return ret.reshape(-1,4)

    def _compute_distance(self, objs, qtc):
        # Same computation as in argprobd but taking the QTC state into account.
        # E.g. if the velocities point towards each other, the distance cannot increase
        if self.prev_dist != '':
            if np.all(qtc[:2] == [0,0]): # No change in distance possible
                return self.prev_dist
            elif np.sum(qtc[:2]) < 0: # [-0,--,0-] coming closer, ruling out distances further away than the last
                start_idx = 0
                end_idx = self.all_possible_relations.index(self.prev_dist)+1
            elif np.sum(qtc[:2]) > 0: # [+0,++,0+] moving away, ruling out distances closer than the last
                start_idx = self.all_possible_relations.index(self.prev_dist)
                end_idx = len(self.all_possible_relations)
            else: # [+-,-+] Undefined states in simplified QTC, all is fair game
                start_idx = 0
                end_idx = len(self.all_possible_relations)
        else:
            start_idx = 0
            end_idx = len(self.all_possible_relations)

        d = np.sqrt(np.square(objs[0].x - objs[1].x) + np.square(objs[0].y - objs[1].y))
        r = (None, 0.0)

        for values, relation in zip(self.all_possible_values[start_idx:end_idx], self.all_possible_relations[start_idx:end_idx]):
            prob = uniform(0.0, self._normpdf(d, mu=values[0], sigma=values[1]))
            r = (relation, prob) if prob > r[1] else r
        self.prev_dist = r[0] if r[0] else self.all_possible_relations[-1]
        return self.prev_dist
