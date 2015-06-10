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

from __future__ import print_function, division
from qsrlib_qsrs.qsr_qtc_simplified_abstractclass import QSR_QTC_Simplified_Abstractclass
import numpy as np
from qsrlib_io.world_qsr_trace import *


class QSR_QTC_BC_Simplified(QSR_QTC_Simplified_Abstractclass):
    """Make default QSRs and provide an example for others"""
    def __init__(self):
        super(QSR_QTC_BC_Simplified, self).__init__()
        self.qtc_type = "bc"
        self.qsr_type = "qtc_bc_simplified"  # must be the same that goes in the QSR_Lib.__const_qsrs_available
        self.all_possible_relations = self.return_all_possible_state_combinations()[0]
        self.qsr_keys = "qtcbcs"

    def make(self, *args, **kwargs):
        """Make the QSRs

        :param args: not used at the moment
        :param kwargs:
                        - input_data: World_Trace
        :return: World_QSR_Trace
        """
        input_data = kwargs["input_data"]
        ret = World_QSR_Trace(qsr_type=self.qsr_type)
        timestamps = input_data.get_sorted_timestamps()

        if kwargs["qsrs_for"]:
            qsrs_for, error_found = self.check_qsrs_for_data_exist(sorted(input_data.trace[timestamps[0]].objects.keys()), kwargs["qsrs_for"])
            if error_found:
                raise Exception("Invalid object combination. Has to be list of tuples. Heard: " + np.array2string(np.array(kwargs['qsrs_for'])))
        else:
            qsrs_for = self._return_all_possible_combinations(sorted(input_data.trace[timestamps[0]].objects.keys()))

        if qsrs_for:
            for p in qsrs_for:
                between = str(p[0]) + "," + str(p[1])
                o1_name = p[0]
                o2_name = p[1]
                quantisation_factor = \
                    input_data.trace[0].objects[o1_name].kwargs["quantisation_factor"] \
                    if input_data.trace[0].objects[o1_name].kwargs["quantisation_factor"] \
                    else 0.
                distance_threshold = input_data.trace[0].objects[o1_name].kwargs["distance_threshold"]
                distances = np.array([])
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
                        distances = np.append(
                            distances,
                            self._get_euclidean_distance(
                                (input_data.trace[timestamp].objects[o1_name].x,
                                     input_data.trace[timestamp].objects[o1_name].y),
                                (input_data.trace[timestamp].objects[o2_name].x,
                                     input_data.trace[timestamp].objects[o2_name].y)
                            )
                        )

                    except KeyError:
                        ret.add_empty_world_qsr_state(timestamp)

                qtc_sequence = self._create_bc_chain(qtc_sequence, distances, distance_threshold)
                if not input_data.trace[0].objects[o1_name].kwargs["no_collapse"]:
                    qtc_sequence = self._collapse_similar_states(qtc_sequence)
                if input_data.trace[0].objects[o1_name].kwargs["validate"]:
                    qtc_sequence = self._validate_qtc_sequence(qtc_sequence)
                for idx, qtc in enumerate(qtc_sequence):
                    qsr = QSR(
                        timestamp=idx+1,
                        between=between,
                        qsr=self.qtc_to_output_format((qtc), kwargs["future"])
                    )
                    ret.add_qsr(qsr, idx+1)

        return ret

    def _create_bc_chain(self, qtc, distances, distance_threshold):
        ret = np.array([])
        for dist, state in zip(distances, qtc):
            if dist > distance_threshold:
                ret = np.append(ret, np.append(state[0:2],[np.nan,np.nan]), axis=0)
            else:
                ret = np.append(ret, state, axis=0)
        return ret.reshape(-1,4)


    def qtc_to_output_format(self, qtc, future=False):
        """Overwrite this for the different QTC veriants to select only the parts
        from the QTCC tuple that you would like to return.
        Example for QTCB: return qtc[0:2]

        :param qtc: The full QTCC tuple [q1,q2,q4,q5]

        :return: "q1,q2,q4,q5" or {"qtcbcs": "q1,q2,q4,q5"} if future is True
        """
        s = super(QSR_QTC_BC_Simplified, self).qtc_to_output_format(qtc) if not np.isnan(qtc[2]) else super(QSR_QTC_BC_Simplified, self).qtc_to_output_format(qtc[0:2])
        return self.handle_future(future, s, self.qsr_keys)

    def _get_euclidean_distance(self, p, q):
        """Calculate the Euclidean distance between points p and q

        :param p: tuple of x,y coordinates
        :param q: tuple of x,y coordinates

        :return: the euclidean distance between p and q
        """
        return np.sqrt(np.power((float(p[0])-float(q[0])),2)+np.power((float(p[1])-float(q[1])),2))
