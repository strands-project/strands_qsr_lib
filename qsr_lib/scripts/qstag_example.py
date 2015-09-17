#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import argparse
import os, sys
import cPickle as pickle
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace



def pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message):
    print(which_qsr, "request was made at ", str(qsrlib_response_message.req_made_at)
          + " and received at " + str(qsrlib_response_message.req_received_at)
          + " and finished at " + str(qsrlib_response_message.req_finished_at))
    print("---")
    print(qsrlib_response_message.qsrs.get_sorted_timestamps())
    print("Response is:")
    for t in qsrlib_response_message.qsrs.get_sorted_timestamps():
        foo = str(t) + ": "
        for k, v in zip(qsrlib_response_message.qsrs.trace[t].qsrs.keys(),
                        qsrlib_response_message.qsrs.trace[t].qsrs.values()):
            foo += str(k) + ":" + str(v.qsr) + "; "
        print(foo)



if __name__ == "__main__":

    options = ["rcc2", "rcc3", "rcc8", "cardir", "qtcbs", "qtccs", "qtcbcs", "argd", "argprobd", "mos", "multiple"]

    parser = argparse.ArgumentParser()
    parser.add_argument("qsr", help="choose qsr: %s" % options, type=str, default='qtcbs')
    parser.add_argument("--ros", action="store_true", default=False, help="Use ROS eco-system")
    #parser.add_argument("-i", "--input", help="file from which to read object states", type=str)
    #parser.add_argument("--validate", help="validate state chain. Only QTC", action="store_true", default=False)
    #parser.add_argument("--quantisation_factor", help="quantisation factor for 0-states in qtc, or 's'-states in mos", type=float, default=0.01)
    #parser.add_argument("--no_collapse", help="does not collapse similar adjacent states. Only QTC", action="store_true", default=True)
    #parser.add_argument("--distance_threshold", help="distance threshold for qtcb <-> qtcc transition. Only QTCBC", type=float)
    #parser.add_argument("-c", "--config", help="config file", type=str)

    args = parser.parse_args()
    args.validate = False
    args.quantisation_factor = 0.01
    args.no_collapse = True

    args.distance_threshold = {"touch":1, "near":3, "medium":5, "far":10}

    qtcbs_qsrs_for = [("o1", "o2"),("o1", "o3")]
    argd_qsrs_for = [("o1", "o2")]
    mos_qsrs_for = [("o1"), ("o2")]

    object_types = {"o1": "Human",
                    "o2": "Chair"}

    if args.qsr in options:
        which_qsr = args.qsr
    else:
        raise ValueError("qsr not found, keywords: %s" % options)

    world = World_Trace()

    which_qsr = ["qtcbs", "argd", "mos"]

    dynamic_args = {"qtcbs": {"quantisation_factor": args.quantisation_factor,
                              "validate": args.validate,
                              "no_collapse": args.no_collapse,
                              "qsrs_for": qtcbs_qsrs_for},

                    "argd": {"qsr_relations_and_values": args.distance_threshold,
                              "qsrs_for": argd_qsrs_for},

                    "mos": {"qsrs_for": mos_qsrs_for},

                    "qstag": {"object_types" : object_types} }


    o1 = [Object_State(name="o1", timestamp=0, x=2., y=2., object_type="Person"),  #accessed first using try: kwargs["object_type"] except:
          Object_State(name="o1", timestamp=1, x=1., y=1., object_type="Person")]
          #Object_State(name="o1", timestamp=2, x=2., y=2., object_type="Human"),
          #Object_State(name="o1", timestamp=3, x=4., y=1., object_type="Human"),
          #Object_State(name="o1", timestamp=4, x=4., y=1., object_type="Human"),
          #Object_State(name="o1", timestamp=5, x=4., y=2., object_type="Human")]

    o2 = [Object_State(name="o2", timestamp=0, x=1., y=1., object_type="Chair"),
          Object_State(name="o2", timestamp=1, x=2., y=2., object_type="Chair")]
          #Object_State(name="o2", timestamp=2, x=1., y=5., object_type="Chair"),
          #Object_State(name="o2", timestamp=3, x=1., y=5., object_type="Chair"),
          #Object_State(name="o2", timestamp=4, x=2., y=5., object_type="Chair"),
          #Object_State(name="o2", timestamp=5, x=2., y=5., object_type="Chair")]

    o3 = [Object_State(name="o3", timestamp=0, x=0., y=0., object_type="Desk"),
          Object_State(name="o3", timestamp=1, x=0., y=0., object_type="Desk")]
          #Object_State(name="o3", timestamp=2, x=0., y=0., object_type="Desk"),
          #Object_State(name="o3", timestamp=3, x=0., y=0., object_type="Desk"),
          #Object_State(name="o3", timestamp=4, x=0., y=0., object_type="Desk"),
          #Object_State(name="o3", timestamp=5, x=0., y=0., object_type="Desk")]


    world.add_object_state_series(o1)
    world.add_object_state_series(o2)
    world.add_object_state_series(o3)
    qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world, dynamic_args=dynamic_args)


    if args.ros:
        try:
            import rospy
            from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client
        except ImportError:
            raise ImportError("ROS not found")
        client_node = rospy.init_node("qsr_lib_ros_client_example")
        cln = QSRlib_ROS_Client()
        req = cln.make_ros_request_message(qsrlib_request_message)
        res = cln.request_qsrs(req)
        qsrlib_response_message = pickle.loads(res.data)
    else:
        qsrlib = QSRlib()
        qsrlib_response_message = qsrlib.request_qsrs(req_msg=qsrlib_request_message)

    pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message)


    #params={'MAX_ROWS':1, 'MIN_ROWS':None, 'MAX_EPI':4, 'num_cores':8}
    qstag = qsrlib_response_message.qstag

    print("Episodes...")
    for i in qstag.episodes:
        print(i)

    qstag.graph2dot('/tmp/act_gr.dot')
    os.system('dot -Tpng /tmp/act_gr.dot -o /tmp/act_gr.png')

    ########   PRINT THE GRAPH   #########
    print("QSTAG Graph:\n", qstag.graph)
    print("NODES:")
    for node in qstag.graph.vs():
        print(node)
    print("EDGES:")
    for edge in qstag.graph.es():
        print(edge, " from: ", edge.source, " to: ", edge.target)
