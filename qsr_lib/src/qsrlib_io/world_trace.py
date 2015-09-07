# -*- coding: utf-8 -*-
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
        self.timestamp = float(timestamp)
        self.x = x
        self.y = y
        self.z = z
        # self.length = float('nan')  # y size
        # self.width = float('nan')  # x size
        # self.height = float('nan')  # z size
        self.set_length_width_height(length=length, width=width, height=height)
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

        self.args = args
        self.kwargs = kwargs

    def set_length_width_height(self, length=float('nan'), width=float('nan'), height=float('nan')):
        if length < 0 or width < 0 or height < 0:
            raise ValueError("Object length, width and height cannot be negative; leave them to default values if unset")
        else:
            self.width = width  # x size
            self.length = length  # y size
            self.height = height  # z size

    def return_bounding_box_2d(self, minimal_width=0, minimal_length=0):
        if self.width < 0 or self.length < 0:
            raise ValueError("Object width and length cannot be negative")
        width = minimal_width if isnan(self.width) else self.width
        length = minimal_length if isnan(self.length) else self.length
        return [self.x-width/2, self.y-length/2, self.x+width/2, self.y+length/2]


class World_State(object):
    def __init__(self, timestamp, objects=None):
        self.timestamp = float(timestamp)
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

    # *** data adders
    def add_object_track_from_list(self, obj_name, track, t0=0, **kwargs):
        """Add the objects data to the world_trace from a list of values

        :param obj_name: name of object
        :param track: list/tuple of values as [[x1, y1, w1, l1], [x2, y2, w2, l2], ...] or [[x1, y1], [x2, y2], ...]
        :param t0: time offset
        :param kwargs:
        """
        object_state_series = []
        for t, v in enumerate(track):
            x = v[0]
            y = v[1]
            try:
                width = v[2]
            except IndexError:
                width = float('nan')
            try:
                length = v[3]
            except IndexError:
                length = float('nan')
            object_state_series.append(Object_State(name=obj_name, timestamp=t+t0,
                                                    x=x, y=y, width=width, length=length,
                                                    **kwargs))
        self.add_object_state_series(object_state_series)

    def add_object_state_to_trace(self, object_state, timestamp=None):
        timestamp = float(timestamp) if timestamp else object_state.timestamp
        try:
            self.trace[timestamp].add_object_state(object_state)
        except KeyError:
            world_state = World_State(timestamp=timestamp, objects={object_state.name: object_state})
            self.trace[timestamp] = world_state
        self.last_updated = timestamp

    def add_object_state_series(self, object_states):
        for s in object_states:
            self.add_object_state_to_trace(object_state=s)
    # *** end of data adders

    def get_last_world_state(self, return_by_reference=True):
        t = self.get_sorted_timestamps()[-1]
        return self.trace[t] if return_by_reference else copy.deepcopy(self.trace[t])

    # *** slicing utilities
    def get_at_timestamp_range(self, start, finish=None, return_by_reference=True, include_finish=True):
        """Returns a World_Trace object between start and finish timestamps.

        :param start: Start timestamp.
            :type start: timestamp format, hopefully
        :param finish: Finish timestamp.
        :param return_by_reference: Returned World_Trace contains links to original or is a deepcopy (default=True).
            :type return_by_reference: bool
        :param include_finish: Include or not the finish element (default=True).
            :type include_finish: bool
        :return: A subsampled between start and finish (including finish element by default) World_Trace.
        :rtype: World_Trace
        """
        timestamps = self.get_sorted_timestamps()
        try:
            istart = timestamps.index(start)
        except ValueError:
            raise ValueError("start not found")
        if not finish:
            finish = timestamps[-1]
        try:
            ifinish = timestamps.index(finish)
        except ValueError:
            raise ValueError("finish not found")
        if istart > ifinish:
            raise ValueError("start cannot be after finish")
        timestamps = timestamps[istart:ifinish] + [timestamps[ifinish]] if include_finish else timestamps[istart:ifinish]
        ret = World_Trace(last_updated=self.last_updated)
        for t in timestamps:
            ret.trace[t] = self.trace[t] if return_by_reference else copy.deepcopy(self.trace[t])
        return ret

    def get_for_objects(self, objects_names, return_by_reference=True):
        ret = World_Trace(last_updated=self.last_updated)
        for t, world_state in self.trace.items():
            for oname in objects_names:
                if return_by_reference:
                    ret.add_object_state_to_trace(world_state.objects[oname], t)
                else:
                    ret.add_object_state_to_trace(copy.deepcopy(world_state.objects[oname]), t)
        return ret

    def get_for_objects_at_timestamp_range(self, start, finish, objects_names,
                                           return_by_reference=True, include_finish=True, time_slicing_first=True):
        if time_slicing_first:
            ret = self.get_at_timestamp_range(start, finish, return_by_reference, include_finish)
            ret = ret.get_for_objects(objects_names)
        else:
            ret = self.get_for_objects(objects_names, return_by_reference)
            ret = ret.get_at_timestamp_range(start, finish, include_finish=include_finish)
        return ret
    # *** end of slicing utilities
