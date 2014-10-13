# -*- coding: utf-8 -*-
"""World state, provides input to QSRlib

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 22 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""

from __future__ import print_function, division
import copy

class Object_State(object):
    def __init__(self, name, timestamp,
                 x=0., y=0., z=0.,
                 roll=0., pitch=0., yaw=0.,
                 length=0., width=0., height=0.,
                 *args, **kwargs):
        self.name = name
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.length = length # y size
        self.width = width # x size
        self.height = height # z size
        self.args = args
        self.kwargs = kwargs

    def return_bounding_box_2d(self):
        if self.width <= 0 or self.length <= 0:
            print("ERROR: can't compute bounding box, width or height has no positive value")
            return []
        return [self.x-self.width/2, self.y-self.length/2, self.x+self.width/2, self.y+self.length/2]


class World_State(object):
    def __init__(self, timestamp, objects=None):
        self.timestamp = timestamp
        self.objects = objects if objects else {}

    def add_object_state(self, object_state):
        self.objects[object_state.name] = object_state

class World_Trace(object):
    def __init__(self, description="", last_updated=False, trace=None):
        self.description = description
        self.last_updated = last_updated
        self.trace = trace if trace else {}

    def get_sorted_timestamps(self):
        return sorted(self.trace.keys())

    def add_object_state_to_trace(self, object_state, timestamp=None):
        if not timestamp:
            timestamp = object_state.timestamp
        try:
            self.trace[timestamp].add_object_state(object_state)
        except KeyError:
            world_state = World_State(timestamp=timestamp, objects={object_state.name: object_state})
            self.trace[timestamp] = world_state
        self.last_updated = timestamp

    def add_object_state_series(self, object_states):
        for s in object_states:
            self.add_object_state_to_trace(object_state=s)

    def get_last(self):
        timestamps = self.get_sorted_timestamps()
        timestamp = timestamps[-1]
        return World_Trace(last_updated=self.last_updated,
                           trace=copy.deepcopy(self.trace[timestamp]))

    def get_at_timestamp(self, timestamp):
        try:
            trace = copy.deepcopy(self.trace[timestamp])
            return World_Trace(last_updated=self.last_updated, timestamps=[timestamp], trace=trace)
        except KeyError:
            print("ERROR: Timestamp not in trace")
            return False

    def get_at_timestamp_range(self, start, finish):
        timestamps = self.get_sorted_timestamps()
        ret = World_Trace(last_updated=self.last_updated, timestamps=[], trace={})
        try:
            iStart = timestamps.index(start)
        except ValueError:
            print("ERROR: start not found")
            return False
        try:
            iFinish = timestamps.index(finish)
        except ValueError:
            print("ERROR: finish not found")
            return False
        if iStart > iFinish:
            print("ERROR: start after finish")
            return False
        ret.timestamps = timestamps[iStart:iFinish] + [timestamps[iFinish]]
        for timestamp in ret.timestamps:
            ret.trace[timestamp] = copy.deepcopy(self.trace[timestamp])
        return ret


    def get_for_objects(self, objects_names):
        ret = World_Trace(last_updated=self.last_updated,
                          trace=copy.deepcopy(self.trace))
        for world_state in ret.trace.values():
            for object_state_name in world_state.objects.keys():
                if object_state_name not in objects_names:
                    world_state.objects.pop(object_state_name)
        return ret

    def get_for_objects_at_timestamp_range(self, start, finish, objects_names):
        try:
            ret = self.get_at_timestamp_range(start, finish)
            ret = ret.get_for_objects(objects_names)
            return ret
        except:
            print("ERROR: something went wrong")
            return False
