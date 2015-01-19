#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""QSRlib ROS client example

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 22 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""

from __future__ import print_function, division
import sys
try:
    import cPickle as pickle
except:
    import pickle
from qsrlib.qsrlib import QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client


if __name__ == "__main__":
    options = {"rcc3": "rcc3_rectangle_bounding_boxes_2d",
               "qtcb": "qtc_b_simplified",
               "rcc3a": "rcc3_rectangle_bounding_boxes_2d"}
    try:
        which_qsr_argv = sys.argv[1]
        which_qsr = options[which_qsr_argv]
    except IndexError:
        print("ERROR: provide which qsr you want to test")
        print("keywords:", options.keys())
        sys.exit(1)
    except KeyError:
        print("ERROR: qsr not found")
        print("keywords:", options.keys())
        sys.exit(1)

    world = World_Trace()

    if which_qsr_argv == "rcc3":
        print("rcc3")
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

        world.add_object_state_series(o1)
        world.add_object_state_series(o2)
        world.add_object_state_series(o3)

    elif which_qsr_argv == "rcc3a":
        print("rcc3a")
        o1 = [Object_State(name="o1", timestamp=0, x=1., y=1., width=5., length=8.),
              Object_State(name="o1", timestamp=1, x=1., y=2., width=5., length=8.)]

        o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., width=5., length=8.)]

        world.add_object_state_series(o1)
        world.add_object_state_series(o2)

    elif which_qsr_argv == "qtcb":
        o1 = [Object_State(name="o1", timestamp=0, x=1., y=1.),
              Object_State(name="o1", timestamp=1, x=2., y=1.),
              Object_State(name="o1", timestamp=2, x=1., y=1.)]

        o2 = [Object_State(name="o2", timestamp=0, x=4., y=1.),
              Object_State(name="o2", timestamp=1, x=4., y=1.),
              Object_State(name="o2", timestamp=2, x=5., y=1.)]

        world.add_object_state_series(o1)
        world.add_object_state_series(o2)

    qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world, include_missing_data=True)
    cln = QSRlib_ROS_Client()
    print("bye")
    req = cln.make_ros_request_message(qsrlib_request_message)
    res = cln.request_qsrs(req)
    out = pickle.loads(res.data)
    print("--------------")
    print("Response is:")
    print("Request was made at ", str(out.timestamp_request_made) + " and received at " + str(out.timestamp_request_received) + " and computed at " + str(out.timestamp_qsrs_computed) )
    for t in out.qsrs.get_sorted_timestamps():
        foo = str(t) + ": "
        for k, v in zip(out.qsrs.trace[t].qsrs.keys(), out.qsrs.trace[t].qsrs.values()):
            foo += str(k) + ":" + str(v.qsr) + "; "
        print(foo)

