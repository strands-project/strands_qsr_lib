#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""QSRlib ROS client example

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>, Christan Dondrup <cdondrup@lincoln.ac.uk>
:Organization: University of Leeds
:Date: 22 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""

from __future__ import print_function, division
import rospy
import sys
try:
    import cPickle as pickle
except:
    import pickle
from qsrlib.qsrlib import QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client
import argparse
import csv


if __name__ == "__main__":
    options = {"rcc2": "rcc2_rectangle_bounding_boxes_2d",
               "rcc3": "rcc3_rectangle_bounding_boxes_2d",
               "rcc8": "rcc8_rectangle_bounding_boxes_2d",
               "coneDir": "cone_direction_bounding_boxes_centroid_2d",
               "qtcbs": "qtc_b_simplified",
               "qtccs": "qtc_c_simplified",
               "qtcbcs": "qtc_bc_simplified",
               "rcc3a": "rcc3_rectangle_bounding_boxes_2d",
               "argd": "arg_relations_distance",
               "argprobd": "arg_prob_relations_distance",
               "mos": "moving_or_stationary"}

    # options["multiple"] = ("cone_direction_bounding_boxes_centroid_2d", "rcc3_rectangle_bounding_boxes_2d", "moving_or_stationary", "qtc_b_simplified")
    options["multiple"] = options.values()
    options["multiple"].pop(options["multiple"].index("arg_relations_distance"))

    parser = argparse.ArgumentParser()
    parser.add_argument("qsr", help="choose qsr: %s" % options.keys(), type=str)
    parser.add_argument("-i", "--input", help="file from which to read object states", type=str)
    parser.add_argument("--validate", help="validate state chain. Only QTC", action="store_true")
    parser.add_argument("--quantisation_factor", help="quantisation factor for 0-states in qtc, or 's'-states in mos", type=float)
    parser.add_argument("--no_collapse", help="does not collapse similar adjacent states. Only QTC", action="store_true")
    parser.add_argument("--distance_threshold", help="distance threshold for qtcb <-> qtcc transition. Only QTCBC", type=float)
    parser.add_argument("--future", help="QSRs as dict", action="store_true")
    parser.add_argument("-c", "--config", help="config file", type=str)
    args = parser.parse_args()

    client_node = rospy.init_node("qsr_lib_ros_client_example")

    try:
        which_qsr_argv = args.qsr
        which_qsr = options[which_qsr_argv]
    except KeyError:
        print("ERROR: qsr not found")
        print("keywords:", options.keys())
        sys.exit(1)

    world = World_Trace()

    dynamic_args = {}

    if which_qsr_argv == "rcc3" or which_qsr_argv == "rcc2":
        dynamic_args = {which_qsr_argv: {"quantisation_factor": args.quantisation_factor}}
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

    elif which_qsr_argv == "rcc8":
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

        o4 = [Object_State(name="o4", timestamp=0, x=1., y=11., width=7., length=9.),
              Object_State(name="o4", timestamp=1, x=2., y=11., width=7., length=9.),
              Object_State(name="o4", timestamp=2, x=3., y=11., width=7., length=9.)]

        world.add_object_state_series(o1)
        world.add_object_state_series(o2)
        world.add_object_state_series(o3)
        world.add_object_state_series(o4)

    elif which_qsr_argv == "mos":
        dynamic_args = {which_qsr_argv: {"quantisation_factor": args.quantisation_factor}}

        o1 = [Object_State(name="o1", timestamp=0, x=1., y=1., width=5., length=8.),
              Object_State(name="o1", timestamp=1, x=2., y=1., width=5., length=8.),
              Object_State(name="o1", timestamp=2, x=3., y=1., width=5., length=8.)]

        o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., width=5., length=8.),
              Object_State(name="o2", timestamp=1, x=11., y=10., width=5., length=8.),
              Object_State(name="o2", timestamp=2, x=11., y=20., width=5., length=8.),
              Object_State(name="o2", timestamp=3, x=11., y=30., width=5., length=8.)]

        world.add_object_state_series(o1)
        world.add_object_state_series(o2)

    elif which_qsr_argv == "argd" or which_qsr_argv == "argprobd":
        qsr_relations_and_values = {"0": 5., "1": 15., "2": 100.} if which_qsr_argv == "argd" else {"0": (2.5,2.5/2), "1": (7.5,7.5/2), "2": [50,50/2]}
        dynamic_args = {which_qsr_argv: {"qsr_relations_and_values": qsr_relations_and_values}}

        o1 = [Object_State(name="o1", timestamp=0, x=1., y=1., width=5., length=8.),
              Object_State(name="o1", timestamp=1, x=1., y=2., width=5., length=8.),
              Object_State(name="o1", timestamp=2, x=1., y=2., width=5., length=8.)]

        o2 = [Object_State(name="o2", timestamp=0, x=1., y=1., width=5., length=8.),
              Object_State(name="o2", timestamp=1, x=11., y=2., width=5., length=8.),
              Object_State(name="o2", timestamp=2, x=21., y=2., width=5., length=8.)]

        o3 = [Object_State(name="o3", timestamp=0, x=21., y=1., width=5., length=8.),
              Object_State(name="o3", timestamp=1, x=31., y=2., width=5., length=8.),
              Object_State(name="o3", timestamp=2, x=41., y=2., width=5., length=8.)]

        world.add_object_state_series(o1)
        world.add_object_state_series(o2)
        world.add_object_state_series(o3)

    elif which_qsr_argv == "coneDir":
        o1 = [Object_State(name="o1", timestamp=0, x=5., y=5., width=2., length=2.),
              Object_State(name="o1", timestamp=1, x=5., y=5., width=2., length=2.),
              Object_State(name="o1", timestamp=2, x=5., y=5., width=2., length=2.)]

        o2 = [Object_State(name="o2", timestamp=0, x=8., y=8., width=2., length=2.),
              Object_State(name="o2", timestamp=1, x=6., y=8., width=2., length=2.),
              Object_State(name="o2", timestamp=2, x=4., y=8., width=2., length=2.),
              Object_State(name="o2", timestamp=3, x=2., y=8., width=2., length=2.)]

        o3 = [Object_State(name="o3", timestamp=0, x=3., y=3., width=2., length=2.),
              Object_State(name="o3", timestamp=1, x=4., y=3., width=2., length=2.),
              Object_State(name="o3", timestamp=2, x=6., y=3., width=2., length=2.)]

        o4 = [Object_State(name="o4", timestamp=0, x=4., y=11., width=7., length=9.),
              Object_State(name="o4", timestamp=1, x=6., y=11., width=7., length=9.),
              Object_State(name="o4", timestamp=2, x=8., y=11., width=7., length=9.)]

        world.add_object_state_series(o1)
        world.add_object_state_series(o2)
        world.add_object_state_series(o3)
        world.add_object_state_series(o4)

    elif which_qsr_argv == "rcc3a":
        o1 = [Object_State(name="o1", timestamp=0, x=1., y=1., width=5., length=8.),
              Object_State(name="o1", timestamp=1, x=1., y=2., width=5., length=8.)]

        o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., width=5., length=8.)]

        world.add_object_state_series(o1)
        world.add_object_state_series(o2)

    elif which_qsr_argv == "qtcbs":
        dynamic_args = {which_qsr_argv: {
            "quantisation_factor": args.quantisation_factor,
            "validate": args.validate,
            "no_collapse": args.no_collapse
        }}

        if args.input:
            ob = []
            with open(args.input) as csvfile:
                reader = csv.DictReader(csvfile)
                print("Reading file '%s':" % args.input)
                for idx,row in enumerate(reader):
                    ob.append(Object_State(
                        name=row['agent1'],
                        timestamp=idx,
                        x=float(row['x1']),
                        y=float(row['y1'])
                    ))
                    ob.append(Object_State(
                        name=row['agent2'],
                        timestamp=idx,
                        x=float(row['x2']),
                        y=float(row['y2'])
                    ))

            world.add_object_state_series(ob)
        else:
            o1 = [Object_State(name="o1", timestamp=0, x=1., y=1.),
                  Object_State(name="o1", timestamp=1, x=2., y=1.),
                  Object_State(name="o1", timestamp=2, x=1., y=1.)]

            o2 = [Object_State(name="o2", timestamp=0, x=4., y=1.),
                  Object_State(name="o2", timestamp=1, x=4., y=1.),
                  Object_State(name="o2", timestamp=2, x=5., y=1.)]

            world.add_object_state_series(o1)
            world.add_object_state_series(o2)

    elif which_qsr_argv == "qtccs":
        dynamic_args = {which_qsr_argv: {
            "quantisation_factor": args.quantisation_factor,
            "validate": args.validate,
            "no_collapse": args.no_collapse
        }}

        if args.input:
            ob = []
            with open(args.input) as csvfile:
                reader = csv.DictReader(csvfile)
                print("Reading file '%s':" % args.input)
                for idx,row in enumerate(reader):
                    ob.append(Object_State(
                        name=row['agent1'],
                        timestamp=idx,
                        x=float(row['x1']),
                        y=float(row['y1'])
                    ))
                    ob.append(Object_State(
                        name=row['agent2'],
                        timestamp=idx,
                        x=float(row['x2']),
                        y=float(row['y2'])
                    ))

            world.add_object_state_series(ob)
        else:
            o1 = [Object_State(name="o1", timestamp=0, x=1., y=1.),
                  Object_State(name="o1", timestamp=1, x=2., y=2.),
                  Object_State(name="o1", timestamp=2, x=1., y=2.)]

            o2 = [Object_State(name="o2", timestamp=0, x=4., y=1.),
                  Object_State(name="o2", timestamp=1, x=4., y=1.),
                  Object_State(name="o2", timestamp=2, x=5., y=1.)]

            world.add_object_state_series(o1)
            world.add_object_state_series(o2)

    elif which_qsr_argv == "qtcbcs":
        dynamic_args = {which_qsr_argv: {
            "quantisation_factor": args.quantisation_factor,
            "distance_threshold": args.distance_threshold,
            "validate": args.validate,
            "no_collapse": args.no_collapse
        }}

        if args.input:
            ob = []
            with open(args.input) as csvfile:
                reader = csv.DictReader(csvfile)
                print("Reading file '%s':" % args.input)
                for idx,row in enumerate(reader):
                    ob.append(Object_State(
                        name=row['agent1'],
                        timestamp=idx,
                        x=float(row['x1']),
                        y=float(row['y1'])
                    ))
                    ob.append(Object_State(
                        name=row['agent2'],
                        timestamp=idx,
                        x=float(row['x2']),
                        y=float(row['y2'])
                    ))

            world.add_object_state_series(ob)
        else:
            o1 = [Object_State(name="o1", timestamp=0, x=1., y=1.),
                  Object_State(name="o1", timestamp=1, x=2., y=2.),
                  Object_State(name="o1", timestamp=2, x=1., y=2.)]

            o2 = [Object_State(name="o2", timestamp=0, x=4., y=1.),
                  Object_State(name="o2", timestamp=1, x=4., y=1.),
                  Object_State(name="o2", timestamp=2, x=5., y=1.)]

            world.add_object_state_series(o1)
            world.add_object_state_series(o2)

    elif which_qsr_argv == "multiple":
        traj = [Object_State(name="traj", timestamp=0, x=1., y=1., width=5., length=8.),
            Object_State(name="traj", timestamp=1, x=1., y=2., width=5., length=8.)]
        o1 = [Object_State(name="o1", timestamp=0, x=11., y=1., width=5., length=8.),
              Object_State(name="o1", timestamp=1, x=11., y=2., width=5., length=8.)]
        o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., width=5., length=8.),
              Object_State(name="o2", timestamp=1, x=11., y=2., width=5., length=8.)]

        world.add_object_state_series(traj)
        world.add_object_state_series(o1)

    # uncomment this to test qsrs_for (and comment out the next line)
    # qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world, include_missing_data=True,
    #                                                 dynamic_args=dynamic_args,
    #                                                 qsrs_for=[("o1", "o3"), ("o2", "o3")])
    # qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world, include_missing_data=True,
    #                                                 dynamic_args=dynamic_args, future=args.future, config=args.config)
    qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world, include_missing_data=True,
                                                    dynamic_args=dynamic_args,
                                                    future=args.future)

    cln = QSRlib_ROS_Client()
    req = cln.make_ros_request_message(qsrlib_request_message)
    res = cln.request_qsrs(req)
    out = pickle.loads(res.data)
    print(which_qsr_argv, "request was made at ", str(out.timestamp_request_made) + " and received at " + str(out.timestamp_request_received) + " and computed at " + str(out.timestamp_qsrs_computed) )
    print("---")
    print("Response is:")
    for t in out.qsrs.get_sorted_timestamps():
        foo = str(t) + ": "
        for k, v in zip(out.qsrs.trace[t].qsrs.keys(), out.qsrs.trace[t].qsrs.values()):
            foo += str(k) + ":" + str(v.qsr) + "; "
            # print(type(v.qsr))
        print(foo)
