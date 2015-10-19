# -*- coding: utf-8 -*-
from __future__ import print_function, division
from numpy import isnan
import copy


class Object_State(object):
    """Data class structure that is holding various information about an object."""

    def __init__(self, name, timestamp,
                 x=float('nan'), y=float('nan'), z=float('nan'),
                 xsize=float('nan'), ysize=float('nan'), zsize=float('nan'),
                 rotation=(),
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
        :param xsize: Total x-size.
        :type xsize: float or int
        :param ysize: Total y-size.
        :type ysize: float or int
        :param zsize: Total z-size.
        :type zsize: float or int
        :param rotation: Rotation of the object in roll-pitch-yaw form or quaternion (x,y,z,w) one.
        :type rotation: tuple or list of floats
        :param args: Optional args.
        :param kwargs: Optional kwargs.
        """
        self.name = name
        """str: Name of the object."""

        self.timestamp = float(timestamp)
        """float: Timestamp of the object state, which matches the corresponding key `t` in `World_Trace.trace[t]`."""

        self.x = x
        """int or float: x-coordinate of the center point."""

        self.y = y
        """int or float: y-coordinate of the center point."""

        self.z = z
        """int or float: z-coordinate of the center point."""

        self.xsize = xsize
        """positive int or float: Total x-size."""

        self.ysize = ysize
        """positive int or float: Total y-size."""

        self.zsize = zsize
        """positive int or float: Total z-size."""

        self.rotation = rotation
        """tuple or list of float: Rotation of the object in roll-pitch-yaw form (r,p,y) or quaternion (x,y,z,w) one."""

        self.args = args
        """Optional args."""

        self.kwargs = kwargs
        """Optional kwargs."""

    @property
    def xsize(self):
        return self.__xsize

    @xsize.setter
    def xsize(self, v):
        if v < 0:
            raise ValueError("xsize cannot be negative")
        else:
            self.__xsize = v

    @property
    def ysize(self):
        return self.__ysize

    @ysize.setter
    def ysize(self, v):
        if v < 0:
            raise ValueError("ysize cannot be negative")
        else:
            self.__ysize = v

    @property
    def zsize(self):
        return self.__zsize

    @zsize.setter
    def zsize(self, v):
        if v < 0:
            raise ValueError("zsize cannot be negative")
        else:
            self.__zsize = v

    @property
    def rotation(self):
        return self.__rotation

    @rotation.setter
    def rotation(self, v):
        if not v or len(v) == 3 or len(v) == 4:
            self.__rotation = v
        else:
            raise ValueError("invalid length of rotation, it must be given as a tuple in roll-pitch-yaw form (3 floats) or quaternion one (4 floats: x,y,z,w) or empty")

    def return_bounding_box_2d(self, xsize_minimal=0, ysize_minimal=0):
        """Compute the 2D bounding box of the object.

        :param xsize_minimal: If object has no x-size (i.e. a point) then compute bounding box based on this minimal x-size.
        :type xsize_minimal: positive int or float
        :param ysize_minimal: If object has no y-size (i.e. a point) then compute bounding box based on this minimal y-size.
        :type ysize_minimal: positive int or float
        :return: The 2D coordinates of the first (closest to origin) and third (farthest from origin) corners of the bounding box.
        :rtype: list of 4 int or float
        """
        xsize = xsize_minimal if isnan(self.xsize) else self.xsize
        ysize = ysize_minimal if isnan(self.ysize) else self.ysize
        return [self.x-xsize/2, self.y-ysize/2, self.x+xsize/2, self.y+ysize/2]


class World_State(object):
    """Data class structure that is holding various information about the world at a particular time."""

    def __init__(self, timestamp, objects=None):
        """Constructor.

        :param timestamp: The timestamp of the world state, which matches the corresponding key `t` in `World_Trace.trace[t]`.
        :type timestamp: int or float
        :param objects: A dictionary holding the state of the objects that exist in this world state, i.e. a dict of objects of type Object_State with the keys being the objects names.
        :type objects: dict
        """
        self.timestamp = float(timestamp)
        """float: Timestamp of the object, which matches the corresponding key `t` in `World_Trace.trace[t]`."""

        self.objects = objects if objects else {}
        """dict: Holds the state of the objects that exist in this world state, i.e. a dict of objects of type
        :class:`Object_State` with the keys being the objects names."""

    def add_object_state(self, object_state):
        """Add/Overwrite an object state.

        :param object_state: Object state to be added in the world state.
        :type object_state: Object_State
        """
        self.objects[object_state.name] = object_state


class World_Trace(object):
    """Data class structure that is holding a time series of the world states."""

    def __init__(self, description="", trace=None):
        """Constructor.

        :param description: Optional description of the world.
        :type description: str
        :param trace: A time series of world states, i.e. a dict of objects of type World_State with the keys being the timestamps.
        :type trace: dict
        :return:
        """
        self.description = description
        """str: Optional description of the world."""

        self.trace = trace if trace else {}
        """dict: Time series of world states, i.e. a dict of objects of type :class:`World_State` with the keys being the timestamps."""

    def get_sorted_timestamps(self):
        """Return a sorted list of the timestamps.

        :return: Sorted list of the timestamps.
        :rtype: list
        """
        return sorted(self.trace.keys())

    # *** data adders
    def add_object_track_from_list(self, track, obj_name, t0=0, **kwargs):
        """Add the objects data to the world_trace from a list of values.

        It is capable of handling 2D and 3D points, 2D and 3D bounding boxes.

        Basically:

        * 2D points: tuples have length of 2 (x, y).
        * 3D points: tuples have length of 3 (x, y, z).
        * 2D bounding boxes: tuples have length of 4 (x, y, xsize,y_size).
        * 3D bounding boxes: tuples have length of 6 (x, y, z, xsize, ysize, zsize).

        :param track:  List of tuples of data.
        :type track: list or tuple of list(s) or tuple(s)
        :param obj_name: Name of the object.
        :type obj_name: str
        :param t0: First timestamp to offset timestamps.
        :type t0: int or float
        :param kwargs: kwargs arguments.
        """
        object_state_series = []
        for t, v in enumerate(track):
            vlen = len(v)
            if vlen == 2:
                object_state_series.append(Object_State(name=obj_name, timestamp=t+t0,
                                                        x=v[0], y=v[1],
                                                        **kwargs))
            elif vlen == 3:
                object_state_series.append(Object_State(name=obj_name, timestamp=t+t0,
                                                        x=v[0], y=v[1], z=v[2],
                                                        **kwargs))
            elif vlen == 4:
                object_state_series.append(Object_State(name=obj_name, timestamp=t+t0,
                                                        x=v[0], y=v[1],
                                                        xsize=v[2], ysize=v[3],
                                                        **kwargs))
            elif vlen == 6:
                object_state_series.append(Object_State(name=obj_name, timestamp=t+t0,
                                                        x=v[0], y=v[1], z=v[2],
                                                        xsize=v[3], ysize=v[4], zsize=v[5],
                                                        **kwargs))
            else:
                raise ValueError("Don't know how to interpret data of length of %d" % vlen)
        self.add_object_state_series(object_state_series)

    def add_object_state(self, object_state, timestamp=None):
        """Add/Overwrite an :class:`Object_State` object.

        :param object_state: The object state.
        :type object_state: Object_State
        :param timestamp: The timestamp where the object state is to be inserted, if not given it is added in the timestamp of the object state.
        :type timestamp: int or float
        """
        timestamp = float(timestamp) if timestamp else object_state.timestamp
        try:
            self.trace[timestamp].add_object_state(object_state)
        except KeyError:
            world_state = World_State(timestamp=timestamp, objects={object_state.name: object_state})
            self.trace[timestamp] = world_state

    def add_object_state_series(self, object_states):
        """Add a series of object states.

        :param object_states: The object states, i.e. a list of :class:`Object_State` objects.
        :type object_states: list or tuple
        """
        for s in object_states:
            self.add_object_state(object_state=s)
    # *** end of data adders

    def get_last_state(self, copy_by_reference=False):
        """ Get the last world state.

        :param copy_by_reference: Return a copy or by reference.
        :type copy_by_reference: bool
        :return: The last state in `self.trace`.
        :rtype: World_State
        """
        t = self.get_sorted_timestamps()[-1]
        return self.trace[t] if copy_by_reference else copy.deepcopy(self.trace[t])

    # *** slicing utilities
    def get_at_timestamp_range(self, start=None, stop=None, istep=1, copy_by_reference=False, include_finish=True):
        """Return a subsample between start and stop timestamps.

        :param start: Start timestamp.
        :type start: int or float
        :param stop: Finish timestamp. If empty then stop is set to the last timestamp.
        :type stop: int or float
        :param istep: subsample based on step measured in timestamps list index
        :type istep: int
        :param copy_by_reference: Return a copy or by reference.
        :type copy_by_reference: bool
        :param include_finish: Whether to include or not the world state at the stop timestamp.
        :type include_finish: bool
        :return: Subsample between start and stop.
        :rtype: World_Trace
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
        ret = World_Trace()
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
        :rtype: World_Trace
        """
        ret = World_Trace()
        for t, state in self.trace.items():
            for oname in objects_names:
                if copy_by_reference:
                    ret.add_object_state(state.objects[oname], t)
                else:
                    ret.add_object_state(copy.deepcopy(state.objects[oname]), t)
        return ret
    # *** end of slicing utilities
