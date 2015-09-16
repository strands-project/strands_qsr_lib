#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message, QSRlib_Response_Message
from qsrlib_io.world_trace import Object_State, World_Trace


def print_world_trace(world_trace):
    for t in world_trace.get_sorted_timestamps():
        print("-t:", t)
        for oname, o in world_trace.trace[t].objects.items():
            print("%s\t%f\t%f\t%f\t%f\t%f\t%f" % (oname, o.x, o.y, o.z, o.xsize, o.ysize, o.xsize))

def pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message):
    print(which_qsr, "request was made at ", str(qsrlib_response_message.req_made_at)
          + " and received at " + str(qsrlib_response_message.req_received_at)
          + " and finished at " + str(qsrlib_response_message.req_finished_at))
    print("---")
    print("Response is:")
    for t in qsrlib_response_message.qsrs.get_sorted_timestamps():
        foo = str(t) + ": "
        for k, v in zip(qsrlib_response_message.qsrs.trace[t].qsrs.keys(),
                        qsrlib_response_message.qsrs.trace[t].qsrs.values()):
            foo += str(k) + ":" + str(v.qsr) + "; "
        print(foo)

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

    #### WORLD_QSR_TRACE DBG
    # which_qsr = "rcc2"
    which_qsr = ["mos", "rcc2", "cardir"]
    qsrlib = QSRlib()
    qsrlib_request_message = QSRlib_Request_Message(which_qsr, world)
    qsrlib_response_message = qsrlib.request_qsrs(qsrlib_request_message)
    qsrs = qsrlib_response_message.qsrs
    print(">> original")
    pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message)
    print()

    # last_state = qsrs.get_last_state(return_by_reference=True)
    # t = last_state.timestamp
    # last_state.qsrs["o1,o2"].qsr["rcc2"] = "c"
    # foo = str(t) + ": "
    # for k, v in zip(last_state.qsrs.keys(), last_state.qsrs.values()):
    #     foo += str(k) + ":" + str(v.qsr) + "; "
    # print(foo)

    # qsrs_new = qsrs.get_for_objects(["o1,o2"])
    # qsrs_new = qsrs.get_for_objects(["o1"])
    # qsrs_new = qsrs.get_at_timestamp_range(0, 2)
    # qsrs_new = qsrs.get_for_objects_at_timestamp_range(1, 3, ["o1,o2"])
    qsrs_new = qsrs.get_for_objects_at_timestamp_range(1, 3, ["o1"])
    # qsrs_new = qsrs.get_for_qsrs(["mos", "rcc2"])

    # qsrs_new.trace[2].qsrs["o1,o2"].qsr["rcc2"] = "c"
    # qsrs_new.trace[2].qsrs["o1"].qsr["mos"] = "whatever"
    qsrlib_response_message_new = QSRlib_Response_Message(qsrs_new,
                                                          qsrlib_response_message.req_made_at,
                                                          qsrlib_response_message.req_received_at,
                                                          qsrlib_response_message.req_finished_at)
    print(">> new")
    pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message_new)
    print()
    print(">> original")
    pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message)
