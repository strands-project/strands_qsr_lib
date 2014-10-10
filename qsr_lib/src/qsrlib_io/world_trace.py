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
    def __init__(self, timestamp, objects={}):
        self.timestamp = timestamp
        self.objects = objects

    def add_object_state(self, object_state):
        self.objects[object_state.name] = object_state

class World_Trace(object):
    def __init__(self, description="", last_updated=False, timestamps=[], trace={}):
        self.description = description
        self.last_updated = last_updated
        self.timestamps = timestamps
        self.trace = trace

    def add_object_state_to_trace(self, object_state, timestamp=None):
        if not timestamp:
            timestamp = object_state.timestamp
        try:
            self.trace[timestamp].add_object_state(object_state)
        except KeyError:
            world_state = World_State(timestamp=timestamp, objects={object_state.name: object_state})
            self.trace[timestamp] = world_state
            self.insert_timestamp(timestamp=timestamp, append=False)
        self.last_updated = timestamp

    def add_object_state_series_to_trace(self, object_snapshots):
        for s in object_snapshots:
            self.add_object_state_to_trace(object_state=s)

    def insert_timestamp(self, timestamp, append):
        if append:
            self.timestamps.append(timestamp)
        else: # for now always append
            self.timestamps.append(timestamp)

    def get_last(self):
        timestamp = self.timestamps[-1]
        return World_Trace(last_updated=self.last_updated,
                           timestamps=[timestamp],
                           trace=copy.deepcopy(self.trace[timestamp]))

    def get_at_timestamp(self, timestamp):
        try:
            trace = copy.deepcopy(self.trace[timestamp])
            return World_Trace(last_updated=self.last_updated, timestamps=[timestamp], trace=trace)
        except KeyError:
            print("ERROR: Timestamp not in trace")
            return False

    def get_at_timestamp_range(self, start, finish):
        ret = World_Trace(last_updated=self.last_updated, timestamps=[], trace={})
        try:
            iStart = self.timestamps.index(start)
        except ValueError:
            print("ERROR: start not found")
            return False
        try:
            iFinish = self.timestamps.index(finish)
        except ValueError:
            print("ERROR: finish not found")
            return False
        if iStart > iFinish:
            print("ERROR: start after finish")
            return False
        ret.timestamps = self.timestamps[iStart:iFinish] + [self.timestamps[iFinish]]
        for timestamp in ret.timestamps:
            ret.trace[timestamp] = copy.deepcopy(self.trace[timestamp])
        return ret


    def get_for_objects(self, objects_names):
        ret = World_Trace(last_updated=self.last_updated,
                          timestamps=copy.deepcopy(self.timestamps),
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

if __name__ == "__main__":
    world = World_Trace()

    o1 = [Object_State(name="o1", timestamp=0, x=1., y=1., width=5., length=8.),
          Object_State(name="o1", timestamp=1, x=1., y=2., width=5., length=8.),
          Object_State(name="o1", timestamp=2, x=1., y=3., width=5., length=8.)]
    o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., width=5., length=8.),
          Object_State(name="o2", timestamp=1, x=11., y=2., width=5., length=8.),
          Object_State(name="o2", timestamp=2, x=11., y=3., width=5., length=8.),
          Object_State(name="o2", timestamp=3, x=11., y=4., width=5., length=8.)]
    o3 = [Object_State(name="o3", timestamp=0, x=1., y=11., width=5., length=8.),
          Object_State(name="o3", timestamp=1, x=2., y=11., width=5., length=8.),
          Object_State(name="o3", timestamp=2, x=3., y=11., width=5., length=8.)]
    world.add_object_state_series_to_trace(o1)
    world.add_object_state_series_to_trace(o2)
    world.add_object_state_series_to_trace(o3)
