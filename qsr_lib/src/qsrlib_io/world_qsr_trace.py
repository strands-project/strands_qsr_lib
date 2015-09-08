# -*- coding: utf-8 -*-
from __future__ import print_function, division
import copy


class QSR(object):
    def __init__(self, timestamp, between, qsr, qsr_type=""):
        self.timestamp = timestamp
        self.between = between
        self.qsr = qsr
        self.type = qsr_type


class World_QSR_State(object):
    def __init__(self, timestamp, qsrs=None):
        self.timestamp = timestamp
        self.qsrs = qsrs if qsrs else {}

    def add_qsr(self, qsr):
        self.qsrs[qsr.between] = qsr

class World_QSR_Trace(object):
    def __init__(self, qsr_type, last_updated=False, trace=None):
        self.qsr_type = qsr_type
        self.last_updated = last_updated
        # self.trace.keys() should be floats just like World_Trace.trace.keys(); hence no casting and no checking here to avoid slowing down execution time for all QSRs
        self.trace = trace if trace else {}

    def get_sorted_timestamps(self):
        # self.trace.keys() should be floats just like World_Trace.trace.keys()
        return sorted(self.trace.keys())

    def __add_world_qsr_state(self, world_qsr_state):
        self.trace[world_qsr_state.timestamp] = world_qsr_state
        # if world_qsr_state.timestamp not in self.timestamps:
        #     self.insert_timestamp(timestamp=world_qsr_state.timestamp, append=False)
        self.last_updated = world_qsr_state.timestamp

    def add_world_qsr_state(self, world_qsr_state, overwrite=False):
        if world_qsr_state.timestamp in self.trace:
            print("Warning: state already exists")
            if overwrite:
                print("Overwrite is enabled, overwriting...")
                self.__add_world_qsr_state(world_qsr_state)
        else:
            self.__add_world_qsr_state(world_qsr_state)

    def add_qsr(self, qsr, timestamp):
        try:
            self.trace[timestamp].add_qsr(qsr)
        except KeyError:
            world_qsr_state = World_QSR_State(timestamp=timestamp, qsrs={qsr.between: qsr})
            self.add_world_qsr_state(world_qsr_state)
        self.last_updated = timestamp

    def add_empty_world_qsr_state(self, timestamp):
        self.add_world_qsr_state(World_QSR_State(timestamp=timestamp))

    def get_last_state(self, copy_by_reference=False):
        t = self.get_sorted_timestamps()[-1]
        return self.trace[t] if copy_by_reference else copy.deepcopy(self.trace[t])

    # *** slicing utilities
    def get_at_timestamp_range(self, start, finish=None, copy_by_reference=False, include_finish=True):
        timestamps = self.get_sorted_timestamps()
        try:
            istart = timestamps.index(start)
        except ValueError:
            raise ValueError("start not found")
        if finish is None:
            finish = timestamps[-1]
        try:
            ifinish = timestamps.index(finish)
        except ValueError:
            raise ValueError("finish not found")
        if istart > ifinish:
            raise ValueError("start cannot be after finish")
        timestamps = timestamps[istart:ifinish] + [timestamps[ifinish]] if include_finish else timestamps[istart:ifinish]
        ret = World_QSR_Trace(self.qsr_type, self.last_updated)
        for t in timestamps:
            ret.trace[t] = self.trace[t] if copy_by_reference else copy.deepcopy(self.trace[t])
        return ret

    def get_for_objects(self, objects_names, copy_by_reference=False):
        ret = World_QSR_Trace(self.qsr_type, self.last_updated)
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

    def get_for_objects_at_timestamp_range(self, start, finish, objects_names,
                                           copy_by_reference=False, include_finish=True, time_slicing_first=True):
        if time_slicing_first:
            ret = self.get_at_timestamp_range(start, finish, copy_by_reference, include_finish)
            ret = ret.get_for_objects(objects_names)
        else:
            ret = self.get_for_objects(objects_names, copy_by_reference)
            ret = ret.get_at_timestamp_range(start, finish, include_finish=include_finish)
        return ret

    def get_for_qsrs(self, qsrs_list):
        ret = World_QSR_Trace(self.qsr_type, self.last_updated)
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
