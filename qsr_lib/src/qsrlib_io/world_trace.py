# -*- coding: utf-8 -*-
from __future__ import print_function, division
from numpy import isnan
import copy


# todo issue #105, probably the easiest is to refactor width, length, height to xsize, ysize, zsize
# todo also deprecate roll, pitch, yaw and use quartenions?
class Object_State(object):
    """Data class structure that is holding various information about an object.

    """
    def __init__(self, name, timestamp,
                 x=float('nan'), y=float('nan'), z=float('nan'),
                 roll=float('nan'), pitch=float('nan'), yaw=float('nan'),
                 length=float('nan'), width=float('nan'), height=float('nan'),
                 *args, **kwargs):
        """Constructor.

        :param name: The typically unique name of the object.
        :type name: str
        :param timestamp: The timestamp of the object state, which matches the corresponding key `t` in `World_Trace.trace[t]`.
        :type timestamp: float or int
        :param x: The x-coordinate of the center point.
        :type x: float or int
        :param y: The y-coordinate of the center point.
        :type y: float or int
        :param z: The z-coordinate of the center point.
        :type z: float or int
        :param roll: Roll of the object.
        :type roll: float or int
        :param pitch: Pitch of the object.
        :type pitch: float or int
        :param yaw: Yaw of the object.
        :type yaw: float or int
        :param length: Total y-size of the object.
        :type length: float or int
        :param width: Total x-size of the object.
        :type width: float or int
        :param height: Total z-size of the object.
        :type height: float or int
        :param args: Other optional args.
        :param kwargs: Other optional kwargs.
        :return:
        """
        self.name = name
        """str: The name of the object"""

        self.timestamp = float(timestamp)
        """float: The timestamp of the object state, which matches the corresponding key `t` in `World_Trace.trace[t]`."""

        self.x = x
        """int or float: The x-coordinate of the center point."""

        self.y = y
        """int or float: The y-coordinate of the center point."""

        self.z = z
        """int or float: The z-coordinate of the center point."""

        self.set_length_width_height(length=length, width=width, height=height)
        """int or float: The length (total y-size), width (total x-size) and height (total z-size) of the object."""

        self.roll = roll
        """int or float: The roll of the object."""

        self.pitch = pitch
        """int or float: The pitch of the object."""

        self.yaw = yaw
        """int or float: The yaw of the object."""

        self.args = args
        self.kwargs = kwargs

    # todo turn this into three separate setter methods
    def set_length_width_height(self, length=float('nan'), width=float('nan'), height=float('nan')):
        """Setter method for length, width, height.

        :param length: Total y-size of the object.
        :type length: int or float
        :param width: Total x-size of the object.
        :type width: int or float
        :param height: Total z-size of the object.
        :type height: int or float
        :return:
        """
        if length < 0 or width < 0 or height < 0:
            raise ValueError("Object length, width and height cannot be negative; leave them to default values if unset")
        else:
            self.width = width  # x size
            """Total x-size of the object."""

            self.length = length  # y size
            """Total y-size of the object."""

            self.height = height  # z size
            """Total z-size of the object."""

    def return_bounding_box_2d(self, minimal_width=0, minimal_length=0):
        """Compute the 2D bounding box of the object.

        :param minimal_width: If object has no x-size (i.e. simply a point) then compute bounding box based on this minimal x-size.
        :type minimal_width: int or float
        :param minimal_length: If object has no y-size (i.e. simply a point) then compute bounding box based on this minimal y-size.
        :return: The coordinates of the upper-left and bottom-right corners of the bounding box.
        :rtype: list
        """
        if self.width < 0 or self.length < 0:
            raise ValueError("Object width and length cannot be negative")
        # todo need to add a check that minimal width and length are both not < 0
        width = minimal_width if isnan(self.width) else self.width
        length = minimal_length if isnan(self.length) else self.length
        return [self.x-width/2, self.y-length/2, self.x+width/2, self.y+length/2]


class World_State(object):
    """Data class structure that is holding various information about the world at a particular time.

    """
    def __init__(self, timestamp, objects=None):
        """Constructor.

        :param timestamp: The timestamp of the world state, which matches the corresponding key `t` in `World_Trace.trace[t]`.
        :type timestamp: int or float
        :param objects: A dictionary holding the state of the objects that exist in this world state, i.e. a dict of objects of type Object_State with the keys being the objects names.
        :type objects: dict
        :return:
        """
        self.timestamp = float(timestamp)
        """float: The timestamp of the object, which matches the corresponding key `t` in `World_Trace.trace[t]`."""

        self.objects = objects if objects else {}
        """dict: Holds the state of the objects that exist in this world state, i.e. a dict of objects of type Object_State
        with the keys being the objects names."""

    # todo maybe add overwrite protection, add argument overwrite=False; I think it was removed for efficiency
    def add_object_state(self, object_state):
        """Add an object state.

        :param object_state: Object state to be added in the world state.
        :type object_state: Object_State
        :return:
        """
        self.objects[object_state.name] = object_state


class World_Trace(object):
    """Data class structure that is holding a time series of the world states.

    """
    # todo deprecate last_updated
    def __init__(self, description="", last_updated=False, trace=None):
        """Constructor.

        :param description: Optional description of the world.
        :type description: str
        :param last_updated: to be deprecated
        :param trace: A time series of world states, i.e. a dict of objects of type World_State with the keys being the timestamps.
        :type trace: dict
        :return:
        """
        self.description = description
        """str: Optional description of the world."""

        # todo decide what to do with this, probably deprecate as it has never been used anywhere before
        self.last_updated = last_updated
        """to be deprecated"""

        self.trace = trace if trace else {}
        """dict: A time series of world states, i.e. a dict of objects of type World_State with the keys being the timestamps."""

    def get_sorted_timestamps(self):
        """Return a sorted list of the timestamps.

        :return: A sorted list of the timestamps.
        :rtype: list
        """
        return sorted(self.trace.keys())

    # *** data adders
    # todo refactor to add_object_track_from_list or better have internal methods that handle lengths 2, 3, 4, 6
    def add_object_track_from_list(self, obj_name, track, t0=0, **kwargs):
        """Add the objects data to the world_trace from a list of values

        :param obj_name: name of object
        :type obj_name: str
        :param track:  List of values as [[x1, y1, w1, l1], [x2, y2, w2, l2], ...] or [[x1, y1], [x2, y2], ...].
        :type track: list
        :param t0: First timestamp to offset timestamps.
        :type t0: int or float
        :param kwargs: Optional arguments.
        :return:
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

    # todo refactor to add_object_state
    def add_object_state_to_trace(self, object_state, timestamp=None):
        """Add an Object_State object.

        :param object_state: The object state.
        :type object_state: Object_State
        :param timestamp: The timestamp where the object state is to be inserted, if not given it is added in the timestamp of the object state.
        :type timestamp: int or float
        :return:
        """
        timestamp = float(timestamp) if timestamp else object_state.timestamp
        try:
            self.trace[timestamp].add_object_state(object_state)
        except KeyError:
            world_state = World_State(timestamp=timestamp, objects={object_state.name: object_state})
            self.trace[timestamp] = world_state
        self.last_updated = timestamp

    def add_object_state_series(self, object_states):
        """Add a series of object states.

        :param object_states: The object states, i.e. a list of Object_State objects.
        :type object_states: list or tuple
        :return:
        """
        for s in object_states:
            self.add_object_state_to_trace(object_state=s)
    # *** end of data adders

    def get_last_state(self, copy_by_reference=False):
        """ Get the last world state.

        :param copy_by_reference: Return by value or by reference.
        :type copy_by_reference: bool
        :return:
        """
        t = self.get_sorted_timestamps()[-1]
        return self.trace[t] if copy_by_reference else copy.deepcopy(self.trace[t])

    # *** slicing utilities
    def get_at_timestamp_range(self, start, finish=None, copy_by_reference=False, include_finish=True):
        """Return a subsample between start and finish timestamps.

        :param start: The start timestamp.
        :type start: int or float
        :param finish: The finish timestamp. If empty then finish is set to the last timestamp.
        :param copy_by_reference: Return by value or by reference.
        :type copy_by_reference: bool
        :param include_finish: Whether to include or not the world state at the finish timestamp.
        :type include_finish: bool
        :return: A subsample between start and finish.
        :rtype: World_Trace
        """
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
        ret = World_Trace(last_updated=self.last_updated)
        for t in timestamps:
            ret.trace[t] = self.trace[t] if copy_by_reference else copy.deepcopy(self.trace[t])
        return ret

    def get_for_objects(self, objects_names, copy_by_reference=False):
        """Return a subsample for requested objects.

        :param objects_names: The requested objects names.
        :type objects_names: list or tuple
        :param copy_by_reference: Return by value or by reference.
        :type copy_by_reference: bool
        :return: A subsample for the requested objects.
        :rtype: World_Trace
        """
        ret = World_Trace(last_updated=self.last_updated)
        for t, state in self.trace.items():
            for oname in objects_names:
                if copy_by_reference:
                    ret.add_object_state_to_trace(state.objects[oname], t)
                else:
                    ret.add_object_state_to_trace(copy.deepcopy(state.objects[oname]), t)
        return ret

    # todo allow finish=None
    def get_for_objects_at_timestamp_range(self, start, finish, objects_names,
                                           copy_by_reference=False, include_finish=True, time_slicing_first=True):
        """Return a subsample for requested objects between start and finish timestamps.

        :param start: The start timestamp.
        :type start: int or float
        :param finish: The finish timestamp.
        :type finish: bool
        :param objects_names: The requested objects names.
        :param copy_by_reference: Return by value or by reference.
        :type copy_by_reference: bool
        :param include_finish: Whether to include or not the world state at the finish timestamp.
        :type include_finish: bool
        :param time_slicing_first: Perform time slicing first or object slicing, can be used to optimize the call.
        :type time_slicing_first: bool
        :return: A subsample for the requested objects between start and finish timestamps.
        :rtype: World_Trace
        """
        if time_slicing_first:
            ret = self.get_at_timestamp_range(start, finish, copy_by_reference, include_finish)
            ret = ret.get_for_objects(objects_names)
        else:
            ret = self.get_for_objects(objects_names, copy_by_reference)
            ret = ret.get_at_timestamp_range(start, finish, include_finish=include_finish)
        return ret
    # *** end of slicing utilities
