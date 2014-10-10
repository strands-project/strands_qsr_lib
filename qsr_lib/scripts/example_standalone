#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Usage example.

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 22 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""

from __future__ import print_function, division
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message

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
          Object_State(name="o3", timestamp=2, x=2., y=11., width=5., length=8.),
          Object_State(name="o3", timestamp=3, x=3., y=11., width=5., length=8.)]

    world.add_object_state_series_to_trace(o1)
    world.add_object_state_series_to_trace(o2)
    world.add_object_state_series_to_trace(o3)


    # make a QSRlib object
    qsrlib = QSRlib()
    # make a request message
    request_message = QSRlib_Request_Message(which_qsr="rcc3_rectangle_bounding_boxes_2d", input_data=world)
    # request QSRs
    out = qsrlib.request_qsrs(request_message=request_message)
    # some print some nice data
    print("Request was made at ", str(out.timestamp_request_made) + " and received at " + str(out.timestamp_request_received) + " and computed at " + str(out.timestamp_qsrs_computed) )
    for t in out.qsrs.timestamps:
        foo = ""
        for k, v in zip(out.qsrs.trace[t].qsrs.keys(), out.qsrs.trace[t].qsrs.values()):
            foo += str(k) + ":" + str(v.qsr) + "; "
        print(foo)
