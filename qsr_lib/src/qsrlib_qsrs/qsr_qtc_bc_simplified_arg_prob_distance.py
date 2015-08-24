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


class QSR_QTC_BC_Simplified_Arg_Prob_Distance(QSR_Arg_Prob_Relations_Distance, QSR_QTC_BC_Simplified):
    """Make default QSRs and provide an example for others"""
    def __init__(self):
        # Calling QSR_Arg_Prob_Relations_Distance.__init__ as all other variable of QTC are overridden any way.
        # Depends on order of super classes in class header.
        super(QSR_QTC_BC_Simplified_Arg_Prob_Distance, self).__init__()
        self.qtc_type = "bc"
        self.all_possible_relations = self.return_all_possible_state_combinations()[0]
        self._unique_id = "qtcbcs_argprobd"
        self.prev_dist = ''

    def make(self, *args, **kwargs):
        """Make the QSRs

        :param args: not used at the moment
        :param kwargs:
                        - input_data: World_Trace
        :return: World_QSR_Trace
        """
        input_data = kwargs["input_data"]
        ret = World_QSR_Trace(qsr_type=self._unique_id)
        timestamps = input_data.get_sorted_timestamps()

        parameters = {
            "distance_threshold": 'und',
            "quantisation_factor": 0.0,
            "validate": True,
            "no_collapse": False,
            "qsr_relations_and_values": {}
        }

        parameters = self._get_parameters(parameters, **kwargs)
        self.set_qsr_relations_and_values(parameters["qsr_relations_and_values"])

        if kwargs["qsrs_for"]:
            qsrs_for, error_found = self.check_qsrs_for_data_exist(sorted(input_data.trace[timestamps[0]].objects.keys()), kwargs["qsrs_for"])
            if error_found:
                raise Exception("Invalid object combination. Has to be list of tuples. Heard: " + np.array2string(np.array(kwargs['qsrs_for'])))
        else:
            qsrs_for = self._return_all_possible_combinations(sorted(input_data.trace[timestamps[0]].objects.keys()))
        distances = {}
        if qsrs_for:
            for p in qsrs_for:
                between = str(p[0]) + "," + str(p[1])
                o1_name = p[0]
                o2_name = p[1]
                quantisation_factor = parameters["quantisation_factor"]
                try:
                    if input_data.trace[0].objects[o1_name].kwargs["quantisation_factor"]:
                        print "Definition of quantisation_factor in object is depricated. Please use parameters field in dynamic_args in service call."
                        quantisation_factor = input_data.trace[0].objects[o1_name].kwargs["quantisation_factor"]
                except:
                    pass
                distance_threshold = parameters["distance_threshold"]
                try:
                    if input_data.trace[0].objects[o1_name].kwargs["distance_threshold"]:
                        print "Definition of distance_threshold in object is depricated. Please use parameters field in dynamic_args in service call."
                        quantisation_factor = input_data.trace[0].objects[o1_name].kwargs["distance_threshold"]
                except:
                    pass

                if not isinstance(distance_threshold, str):
                    raise Exception(self._unique_id+ " only accepts a string naming the abstract distance relation used as a threshold.")
                if not distance_threshold in self.all_possible_relations:
                    raise Exception(self._unique_id+ ": distance threshold: "+distance_threshold+" is not in: "+(','.join(x for x in self.all_possible_relations)))

                qtc_sequence = np.array([], dtype=int)
                for t0, t1 in zip(timestamps, timestamps[1:]):
                    timestamp = t1
                    try:
                        k = [input_data.trace[t0].objects[o1_name].x,
                             input_data.trace[t0].objects[o1_name].y,
                             input_data.trace[t1].objects[o1_name].x,
                             input_data.trace[t1].objects[o1_name].y]
                        l = [input_data.trace[t0].objects[o2_name].x,
                             input_data.trace[t0].objects[o2_name].y,
                             input_data.trace[t1].objects[o2_name].x,
                             input_data.trace[t1].objects[o2_name].y]
                        qtc_sequence = np.append(qtc_sequence, self._create_qtc_representation(
                            k,
                            l,
                            quantisation_factor
                        )).reshape(-1,4)
                    except KeyError:
                        ret.add_empty_world_qsr_state(timestamp)

                    # Calculating distances. Distances for objc1 <-> objc2 are the same
                    # as objc2 <-> objc1. Saves time and makes for more consistent results.
                    if not o1_name+'_'+o2_name in distances.keys() \
                        and not o2_name+'_'+o1_name in distances.keys():
                            distances[o1_name+'_'+o2_name] = []

                    try:
                        dist = distances[o1_name+'_'+o2_name].append(self._compute_distance(input_data.trace[t1].objects.values(), qtc_sequence[-1]))
                    except KeyError:
                        pass # If this fails we already have distance values for the opposite combination of objects

                # Find which combinations of objects has distance values and use it for the opposite combination as well.
                try:
                    dist = distances[o2_name+'_'+o1_name]
                except KeyError:
                    dist = distances[o1_name+'_'+o2_name]

                no_collapse = parameters["no_collapse"]
                try:
                    if input_data.trace[0].objects[o1_name].kwargs["no_collapse"]:
                        print "Definition of no_collapse in object is depricated. Please use parameters field in dynamic_args in service call."
                        no_collapse = input_data.trace[0].objects[o1_name].kwargs["no_collapse"]
                except:
                    pass
                try:
                    validate = parameters["validate"]
                    if input_data.trace[0].objects[o1_name].kwargs["validate"]:
                        print "Definition of validate in object is depricated. Please use parameters field in dynamic_args in service call."
                        validate = input_data.trace[0].objects[o1_name].kwargs["validate"]
                except:
                    pass

                if not type(no_collapse) is bool or not type(validate) is bool:
                    raise Exception("'no_collapse' and 'validate' have to be boolean values.")

                qtc_sequence = self._create_bc_chain(qtc_sequence, dist, distance_threshold)
                if not no_collapse:
                    qtc_sequence = self._collapse_similar_states(qtc_sequence)
                if validate:
                    qtc_sequence = self._validate_qtc_sequence(qtc_sequence)
                for idx, qtc in enumerate(qtc_sequence):
                    qsr = QSR(
                        timestamp=idx+1,
                        between=between,
                        qsr=self.qtc_to_output_format((qtc), kwargs["future"])
                    )
                    ret.add_qsr(qsr, idx+1)

        if no_collapse and not validate:
            self._rectify_timestamps(input_data, ret)

        return ret

    def _create_bc_chain(self, qtc, distances, distance_threshold):
        ret = np.array([])
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
