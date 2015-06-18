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
from numpy import isnan
import copy

class Object_State(object):
    def __init__(self, name, timestamp,
                 x=float('nan'), y=float('nan'), z=float('nan'),
                 roll=float('nan'), pitch=float('nan'), yaw=float('nan'),
                 length=float('nan'), width=float('nan'), height=float('nan'),
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

    def return_bounding_box_2d(self, minimal_width=2, minimal_length=2):
        width = minimal_width if (isnan(self.width) or self.width == 0) else self.width
        length = minimal_length if (isnan(self.length) or self.length == 0) else self.length
        return [self.x-width/2, self.y-length/2, self.x+width/2, self.y+length/2]


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
        # TODO this is so inefficient, why do I store the timestamps as strings in the first place (answer ros serializ)
        ret = self.trace.keys()
        foo = ret[0]
        ret = [int(i) for i in ret]
        ret = sorted(ret)
        if type(foo) is str:  # if they were str make them strs again
            ret = [str(i) for i in ret]
        return ret

    def add_object_track_from_list(self, obj_name, track, t0=0, **kwargs):
        """Add the objects data to the world_trace from a list of values

        :param obj_name: name of object
        :param track: list/tuple of values as [[x1, y1, w1, l1], [x2, y2, w2, l2], ...] or [[x1, y1], [x2, y2], ...]
        :param t0: time offset
        :param kwargs:
        """
        object_state_series = []
        for t in range(len(track)):
            x = track[t][0]
            y = track[t][1]
            try:
                width = track[t][2]
            except IndexError:
                width = float('nan')
            try:
                length = track[t][3]
            except IndexError:
                length = float('nan')
            object_state_series.append(Object_State(name=obj_name, timestamp=t+t0,
                                                    x=x, y=y, width=width, length=length,
                                                    **kwargs))
        self.add_object_state_series(object_state_series)

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
