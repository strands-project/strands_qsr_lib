#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace
import argparse

def pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message):
    print(which_qsr, "request was made at ", str(qsrlib_response_message.timestamp_request_made)
          + " and received at " + str(qsrlib_response_message.timestamp_request_received)
          + " and computed at " + str(qsrlib_response_message.timestamp_qsrs_computed))
    print("---")
    print("Response is:")
    for t in qsrlib_response_message.qsrs.get_sorted_timestamps():
        foo = str(t) + ": "
        for k, v in zip(qsrlib_response_message.qsrs.trace[t].qsrs.keys(),
                        qsrlib_response_message.qsrs.trace[t].qsrs.values()):
            foo += str(k) + ":" + str(v.qsr) + "; "
        print(foo)

if __name__ == "__main__":
    # ****************************************************************************************************
    # create a QSRlib object if there isn't one already
    qsrlib = QSRlib()

    # ****************************************************************************************************
    # parse command line arguments
    options = sorted(qsrlib.get_qsrs_registry().keys())
    parser = argparse.ArgumentParser()
    parser.add_argument("qsr", help="choose qsr: %s" % options, type=str)
    args = parser.parse_args()
    if args.qsr in options:
        which_qsr = args.qsr
    else:
        raise ValueError("qsr not found, keywords: %s" % options)

    # ****************************************************************************************************
    # make some input data
    world = World_Trace()
    o1 = [Object_State(name="o1", timestamp=0, x=1., y=1., width=5., length=8.),
          Object_State(name="o1", timestamp=1, x=1., y=2., width=5., length=8.)]

    o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., width=5., length=8.),
          Object_State(name="o2", timestamp=1, x=1., y=2., width=5., length=8.)]
    world.add_object_state_series(o1)
    world.add_object_state_series(o2)

    # ****************************************************************************************************
    # make a QSRlib request message
    qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world)
    # request your QSRs
    qsrlib_response_message = qsrlib.request_qsrs(request_message=qsrlib_request_message)

    # ****************************************************************************************************
    # print out your QSRs
    pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message)
