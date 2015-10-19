# -*- coding: utf-8 -*-
from __future__ import print_function, division
import copy


class QSR(object):
    """Data class structure that is holding the QSR and other related information."""

    def __init__(self, timestamp, between, qsr, qsr_type=""):
        """Constructor.

        :param timestamp: The timestamp of the QSR, which matches the corresponding key `t` in `World_QSR_Trace.trace[t]`.
        :type: float
        :param between: For which object(s) is the QSR for. For multiple objects names are separated by commas, e.g. "o1,o2".
        :type between: str
        :param qsr: The QSR value(s). It is a dictionary where the keys are the unique names of each QSR and the values
        are the QSR values as strings.
        :type qsr: dict
        :param qsr_type: The name of the QSR. For multiple QSRs it is usually a sorted comma separated string.
        :type qsr_type: str
        """
        self.timestamp = timestamp
        """float: Timestamp of the QSR, which usually matches the corresponding key `t` in `World_QSR_Trace.trace[t]`."""

        self.between = between
        """str: For which object(s) is the QSR for. Multiple objects are comma separated, e.g. "o1,o2"."""

        self.qsr = qsr
        """dict: QSR value(s). It is a dictionary where the keys are the unique names of each QSR and the values
        are the QSR values as strings."""

        self.type = qsr_type
        """str: Name of the QSR. For multiple QSRs it is usually a sorted comma separated string."""


class World_QSR_State(object):
    """Data class structure that is holding various information about the QSR world at a particular time."""

    def __init__(self, timestamp, qsrs=None):
        """Constructor.

        :param timestamp: The timestamp of the state, which matches the corresponding key `t` in `World_QSR_Trace.trace[t]`.
        :type timestamp: float
        :param qsrs: A dictionary holding the QSRs that exist in this world QSR state, i.e. a dict of
        objects of type QSR with the keys being the object(s) names that these QSR are for.
        :type qsrs: dict
        """
        self.timestamp = timestamp
        """float: Timestamp of the state, which matches the corresponding key `t` in `World_QSR_Trace.trace[t]`."""

        self.qsrs = qsrs if qsrs else {}
        """dict: Holds the QSRs that exist in this world QSR state, i.e. a dict of objects of type QSR with the keys
        being the object(s) names that these QSR are for."""

    def add_qsr(self, qsr):
        """Add/Overwrite a QSR object to the state.

        :param qsr: QSR to be added in the world QSR state.
        :type qsr: QSR
        """
        self.qsrs[qsr.between] = qsr


class World_QSR_Trace(object):
    """Data class structure that is holding a time series of the world QSR states.

    """
    def __init__(self, qsr_type, trace=None):
        """Constructor.

        :param qsr_type: Name of the QSR. For multiple QSRs it is usually a sorted comma separated string.
        :type qsr_type: str
        :param trace: Time series of world QSR states, i.e. a dict of objects of type :class:`World_QSR_State` with the keys being the timestamps.
        :type trace: dict
        """
        self.qsr_type = qsr_type
        """str: Name of the QSR. For multiple QSRs it is usually a sorted comma separated string."""

        self.trace = trace if trace else {}
        """dict: Time series of world QSR states, i.e. a dict of objects of type :class:`World_QSR_State` with the keys being the timestamps."""

    def get_sorted_timestamps(self):
        """Return a sorted list of the timestamps.

        :return: Sorted list of the timestamps.
        :rtype: list of floats
        """
        # NOTE: self.trace.keys() should be floats just like World_Trace.trace.keys(), no casting for better performance
        return sorted(self.trace.keys())

    def add_world_qsr_state(self, world_qsr_state):
        """Add/Overwrite a world QSR state.

        :param world_qsr_state: World QSR state to be added.
        :type world_qsr_state: World_QSR_State
        """
        self.trace[world_qsr_state.timestamp] = world_qsr_state

    def add_qsr(self, qsr, timestamp):
        """Add/Overwrite a QSR at timestamp.

        :param qsr: QSR object to be added.
        :type qsr: QSR
        :param timestamp: Timestamp of the QSR.
        :type timestamp: float
        """
        try:
            self.trace[timestamp].add_qsr(qsr)
        except KeyError:
            world_qsr_state = World_QSR_State(timestamp=timestamp, qsrs={qsr.between: qsr})
            self.add_world_qsr_state(world_qsr_state)

    def put_empty_world_qsr_state(self, timestamp):
        """Put an empty :class:`World_QSR_State` object at timestamp.

        :param timestamp: Timestamp of where to add an empty World_QSR_State
        :type timestamp: float
        :return:
        """
        self.add_world_qsr_state(World_QSR_State(timestamp=timestamp))

    def get_last_state(self, copy_by_reference=False):
        """ Get the last world QSR state.

        :param copy_by_reference: Return a copy or by reference.
        :type copy_by_reference: bool
        :return: Last world QSR state in `self.trace`.
        :rtype: World_QSR_State
        """
        t = self.get_sorted_timestamps()[-1]
        return self.trace[t] if copy_by_reference else copy.deepcopy(self.trace[t])

    # *** slicing utilities
    def get_at_timestamp_range(self, start=None, stop=None, istep=1, copy_by_reference=False, include_finish=True):
        """Return a subsample between start and stop timestamps.

        :param start: Sstart timestamp.
        :type start: int or float
        :param stop: Finish timestamp. If empty then stop is set to the last timestamp.
        :type stop: int or float
        :param istep: subsample based on istep measured in timestamps list index
        :type istep: int
        :param copy_by_reference: Return a copy or by reference.
        :type copy_by_reference: bool
        :param include_finish: Whether to include or not the world state at the stop timestamp.
        :type include_finish: bool
        :return: Subsample between start and stop.
        :rtype: World_QSR_Trace
        """
        timestamps = self.get_sorted_timestamps()
        if start is None:
            start = timestamps[0]
        try:
            istart = timestamps.index(start)
        except ValueError:
            raise ValueError("start not found")
        if stop is None:
            stop = timestamps[-1]
        try:
            istop = timestamps.index(stop)
        except ValueError:
            raise ValueError("stop not found")
        if istart > istop:
            raise ValueError("start cannot be after stop")
        timestamps = timestamps[istart:istop] + [timestamps[istop]] if include_finish else timestamps[istart:istop]
        if istep > 1:
            timestamps = timestamps[::istep]
        ret = World_QSR_Trace(self.qsr_type)
        for t in timestamps:
            ret.trace[t] = self.trace[t] if copy_by_reference else copy.deepcopy(self.trace[t])
        return ret

    def get_for_objects(self, objects_names, copy_by_reference=False):
        """Return a subsample for requested objects.

        :param objects_names: Requested objects names.
        :type objects_names: list or tuple of str
        :param copy_by_reference: Return a copy or by reference.
        :type copy_by_reference: bool
        :return: Subsample for the requested objects.
        :rtype: World_QSR_Trace
        """
        ret = World_QSR_Trace(self.qsr_type)
        all_objects = set([oname for t in self.get_sorted_timestamps() for oname in self.trace[t].qsrs])
        for t, state in self.trace.items():
            for oname in objects_names:
                try:
                    if copy_by_reference:
                        ret.add_qsr(state.qsrs[oname], t)
                    else:
                        ret.add_qsr(copy.deepcopy(state.qsrs[oname]), t)
                except KeyError as e:
                    if oname not in all_objects:
                        raise e
        return ret

    def get_for_qsrs(self, qsrs_list):
        """Return a subsample for requested QSRs only.

        :param qsrs_list: List of requested QSRs.
        :type qsrs_list: list of str
        :return: Subsample for the requested QSRs.
        :rtype: World_QSR_Trace
        """
        ret = World_QSR_Trace(self.qsr_type)
        for t, state in self.trace.items():
            for oname, qsrs in state.qsrs.items():
                qsr = {}
                for q in qsrs_list:
                    try:
                        qsr[q] = qsrs.qsr[q]
                    except KeyError:
                        pass
                if qsr:
                    ret.add_qsr(QSR(t, oname, qsr, ",".join(sorted(qsrs_list))), t)
        return ret
    # *** end of slicing utilities
