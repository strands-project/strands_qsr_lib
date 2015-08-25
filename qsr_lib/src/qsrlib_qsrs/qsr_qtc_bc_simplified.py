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
from qsrlib_qsrs.qsr_qtc_simplified_abstractclass import QSR_QTC_Simplified_Abstractclass
import numpy as np
from qsrlib_io.world_qsr_trace import *


class QSR_QTC_BC_Simplified(QSR_QTC_Simplified_Abstractclass):
    """Make default QSRs and provide an example for others"""
    def __init__(self):
        super(QSR_QTC_BC_Simplified, self).__init__()
        self._unique_id = "qtcbcs"
        self.qtc_type = "bc"
        self.all_possible_relations = self.return_all_possible_state_combinations()[0]

    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, **kwargs):
        ret = World_QSR_Trace(qsr_type=self._unique_id)
        qtc_sequence = {}
        for t, tp in zip(timestamps[1:], timestamps):
            world_state_now = world_trace.trace[t]
            world_state_previous = world_trace.trace[tp]
            if set(world_state_now.objects.keys()) != set(world_state_previous.objects.keys()):
                ret.add_empty_world_qsr_state(t)
                continue # Objects have to be present in both timestamps
            qsrs_for = self._process_qsrs_for(world_state_now.objects.keys(), kwargs)
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
                distance = self._get_euclidean_distance(
                    (world_state_now.objects[o1_name].x,
                         world_state_now.objects[o1_name].y),
                    (world_state_now.objects[o2_name].x,
                         world_state_now.objects[o2_name].y)
                )

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
        for dist, state in zip(distances, qtc):
            if dist > distance_threshold:
                ret = np.append(ret, np.append(state[0:2],[np.nan,np.nan]), axis=0)
            else:
                ret = np.append(ret, state, axis=0)
        return ret.reshape(-1,4)

    def qtc_to_output_format(self, qtc):
        """Overwrite this for the different QTC veriants to select only the parts
        from the QTCC tuple that you would like to return.
        Example for QTCB: return qtc[0:2]

        :param qtc: The full QTCC tuple [q1,q2,q4,q5]

        :return: "q1,q2,q4,q5" or {"qtcbcs": "q1,q2,q4,q5"} if future is True
        """
        s = self.create_qtc_string(qtc) if not np.isnan(qtc[2]) else self.create_qtc_string(qtc[0:2])
        return self._format_qsr(s)

    def _get_euclidean_distance(self, p, q):
        """Calculate the Euclidean distance between points p and q

        :param p: tuple of x,y coordinates
        :param q: tuple of x,y coordinates

        :return: the euclidean distance between p and q
        """
        return np.sqrt(np.power((float(p[0])-float(q[0])),2)+np.power((float(p[1])-float(q[1])),2))
