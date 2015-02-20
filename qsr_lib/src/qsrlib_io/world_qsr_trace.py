from __future__ import print_function, division
# import copy

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
        # self.timestamps = timestamps if timestamps else []
        self.trace = trace if trace else {}

    def get_sorted_timestamps(self):
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

    # def insert_timestamp(self, timestamp, append):
    #     if append:
    #           self.timestamps.append(timestamp)
    #     else: # for now always append
    #         self.timestamps.append(timestamp)
    #
    # def get_last(self):
    #     timestamp = self.timestamps[-1]
    #     return World_QSR_Trace(last_updated=self.last_updated,
    #                            timestamps=[timestamp],
    #                            trace=copy.deepcopy(self.trace[timestamp]))
    #
    # def get_at_timestamp(self, timestamp):
    #     try:
    #         trace = copy.deepcopy(self.trace[timestamp])
    #         return World_QSR_Trace(last_updated=self.last_updated, timestamps=[timestamp], trace=trace)
    #     except KeyError:
    #         print("ERROR: Timestamp not in trace")
    #         return False
    #
    # def get_at_timestamp_range(self, start, finish):
    #     ret = World_QSR_Trace(last_updated=self.last_updated, timestamps=[], trace={})
    #     try:
    #         iStart = self.timestamps.index(start)
    #     except ValueError:
    #         print("ERROR: start not found")
    #         return False
    #     try:
    #         iFinish = self.timestamps.index(finish)
    #     except ValueError:
    #         print("ERROR: finish not found")
    #         return False
    #     if iStart > iFinish:
    #         print("ERROR: start after finish")
    #         return False
    #     ret.timestamps = self.timestamps[iStart:iFinish] + [self.timestamps[iFinish]]
    #     for timestamp in ret.timestamps:
    #         ret.trace[timestamp] = copy.deepcopy(self.trace[timestamp])
    #     return ret

    # def get_for_objects(self, objects_names):
    #     ret = World_QSR_Trace(last_updated=self.last_updated,
    #                           timestamps=copy.deepcopy(self.timestamps),
    #                           trace=copy.deepcopy(self.trace))
    #     for world_state in ret.trace.values():
    #         for object_state_name in world_state.objects.keys():
    #             if object_state_name not in objects_names:
    #                 world_state.objects.pop(object_state_name)
    #     return ret
    #
    # def get_for_objects_at_timestamp_range(self, start, finish, objects_names):
    #     try:
    #         ret = self.get_at_timestamp_range(start, finish)
    #         ret = ret.get_for_objects(objects_names)
    #         return ret
    #     except:
    #         print("ERROR: something went wrong")
    #         return False
