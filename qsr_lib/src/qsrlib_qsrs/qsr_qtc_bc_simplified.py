# -*- coding: utf-8 -*-
from __future__ import division
from qsrlib_qsrs.qsr_qtc_simplified_abstractclass import QSR_QTC_Simplified_Abstractclass
import numpy as np
from qsrlib_io.world_qsr_trace import *


class QSR_QTC_BC_Simplified(QSR_QTC_Simplified_Abstractclass):
    """QTCBC simplified relations.

    Values of the abstract properties
        * **_unique_id** = "qtcbcs"
        * **_all_possible_relations** = ?
        * **_dtype** = "points"

    Some explanation about the QSR or better link to a separate webpage explaining it. Maybe a reference if it exists.
    """

    def __init__(self):
        """Constructor."""
        super(QSR_QTC_BC_Simplified, self).__init__()

        self._unique_id = "qtcbcs"
        """str: Unique identifier name of the QSR."""

        self.qtc_type = "bc"
        """str: QTC specific type."""

        self._all_possible_relations = tuple(self.return_all_possible_state_combinations()[0])
        """tuple: All possible relations of the QSR."""

    def make_world_qsr_trace(self, world_trace, timestamps, qsr_params, req_params, **kwargs):
        """Compute the world QSR trace from the arguments.

        :param world_trace: Input data.
        :type world_trace: :class:`World_Trace <qsrlib_io.world_trace.World_Trace>`
        :param timestamps: List of sorted timestamps of `world_trace`.
        :type timestamps: list
        :param qsr_params: QSR specific parameters passed in `dynamic_args`.
        :type qsr_params: dict
        :param req_params: Dynamic arguments passed with the request.
        :type dynamic_args: dict
        :param kwargs: kwargs arguments.
        :return: Computed world QSR trace.
        :rtype: :class:`World_QSR_Trace <qsrlib_io.world_qsr_trace.World_QSR_Trace>`
        """
        ret = World_QSR_Trace(qsr_type=self._unique_id)
        qtc_sequence = {}
        for t, tp in zip(timestamps[1:], timestamps):
            world_state_now = world_trace.trace[t]
            world_state_previous = world_trace.trace[tp]
            if set(world_state_now.objects.keys()) != set(world_state_previous.objects.keys()):
                ret.put_empty_world_qsr_state(t)
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
        """

        :param qtc:
        :type qtc:
        :param distances:
        :type distances:
        :param distance_threshold:
        :type distance_threshold:
        :return:
        :rtype:
        """
        ret = np.array([])
        if len(qtc.shape) == 1:
            qtc = [qtc]
        for dist, state in zip(distances, qtc):
            if dist > distance_threshold:
                ret = np.append(ret, np.append(state[0:2],[np.nan,np.nan]), axis=0)
            else:
                ret = np.append(ret, state, axis=0)
        return ret.reshape(-1,4)

    def qtc_to_output_format(self, qtc):
        """Overwrite this for the different QTC variants to select only the parts from the QTCCS tuple that you would
        like to return. Example for QTCBS: return `qtc[0:2]`.

        :param qtc: Full QTCC tuple [q1,q2,q4,q5].
        :type qtc: list or tuple
        :return: {"qtcbcs": "q1,q2,q4,q5"}
        :rtype: dict
        """
        s = self.create_qtc_string(qtc) if not np.isnan(qtc[2]) else self.create_qtc_string(qtc[0:2])
        return self._format_qsr(s)

    def _get_euclidean_distance(self, p, q):
        """Calculate the Euclidean distance between points `p` and `q`.

        :param p: x,y coordinates.
        :type p: tuple
        :param q: x,y coordinates.
        :type q: tuple
        :return: Euclidean distance between `p` and `q`.
        :rtype: float
        """
        return np.sqrt(np.power((float(p[0])-float(q[0])),2)+np.power((float(p[1])-float(q[1])),2))
