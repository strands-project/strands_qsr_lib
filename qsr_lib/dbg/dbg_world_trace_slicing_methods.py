#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_io.world_trace import Object_State, World_Trace


def print_world_trace(world_trace):
    for t in world_trace.get_sorted_timestamps():
        print("-t:", t)
        for oname, o in world_trace.trace[t].objects.items():
            print("%s\t%f\t%f\t%f\t%f\t%f\t%f" % (oname, o.x, o.y, o.z, o.xsize, o.ysize, o.xsize))

def print_world_state(world_state):
    for oname, o in world_state.objects.items():
            print("%s\t%f\t%f\t%f\t%f\t%f\t%f" % (oname, o.x, o.y, o.z, o.xsize, o.ysize, o.xsize))

if __name__ == "__main__":
    world = World_Trace()
    o1 = [Object_State(name="o1", timestamp=0, x=1., y=1., xsize=5., ysize=8.),
          Object_State(name="o1", timestamp=1, x=1., y=2., xsize=5., ysize=8.),
          Object_State(name="o1", timestamp=2, x=1., y=3., xsize=5., ysize=8.),
          Object_State(name="o1", timestamp=3, x=1., y=4., xsize=5., ysize=8.),
          Object_State(name="o1", timestamp=4, x=1., y=5., xsize=5., ysize=8.)]

    o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., xsize=5., ysize=8.),
          Object_State(name="o2", timestamp=1, x=11., y=2., xsize=5., ysize=8.),
          Object_State(name="o2", timestamp=2, x=11., y=3., xsize=5., ysize=8.),
          Object_State(name="o2", timestamp=3, x=11., y=4., xsize=5., ysize=8.),
          Object_State(name="o2", timestamp=4, x=11., y=5., xsize=5., ysize=8.)]
    world.add_object_state_series(o1)
    world.add_object_state_series(o2)

    # world_new = world.get_at_timestamp_range(2, 3, include_finish=False)
    # world_new.trace[2.0].objects["o1"].x = 100.0
    # print(">> original")
    # print_world_trace(world)
    # print()
    # print(">> new")
    # print_world_trace(world_new)
    #
    # world_state_latest = world.get_last_state(copy_by_reference=True)
    # world_state_latest.objects["o1"].x = 100.0
    # print(">> original")
    # print("-t:", world.get_sorted_timestamps()[-1])
    # print_world_state(world.trace[world.get_sorted_timestamps()[-1]])
    # print()
    # print(">> new")
    # print("-t:", world_state_latest.timestamp)
    # print_world_state(world_state_latest)
    #
    # world_new = world.get_for_objects(["o1"])
    # world_new.trace[4.0].objects["o1"].x = 100.0
    # print(">> original")
    # print_world_trace(world)
    # print()
    # print(">> new")
    # print_world_trace(world_new)
    #
    # world_new = world.get_for_objects_at_timestamp_range(2, 3, ["o1"], include_finish=False, time_slicing_first=True)
    # world_new.trace[2.0].objects["o1"].x = 100.0
    # print(">> original")
    # print_world_trace(world)
    # print()
    # print(">> new")
    # print_world_trace(world_new)
