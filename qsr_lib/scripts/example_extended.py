#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import sys
try:
    import cPickle as pickle
except:
    import pickle
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace
import argparse
import csv


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


if __name__ == "__main__":
    options = sorted(QSRlib().qsrs_registry.keys()) + ["multiple"]
    multiple = options[:]; multiple.remove("multiple"); multiple.remove("argd"); multiple.remove("argprobd")

    parser = argparse.ArgumentParser()
    parser.add_argument("qsr", help="choose qsr: %s" % options, type=str)
    parser.add_argument("-i", "--input", help="file from which to read object states", type=str)
    parser.add_argument("--validate", help="validate state chain. Only QTC", action="store_true")
    parser.add_argument("--quantisation_factor", help="quantisation factor for 0-states in qtc, or 's'-states in mos", type=float)
    parser.add_argument("--no_collapse", help="does not collapse similar adjacent states. Only QTC", action="store_true")
    parser.add_argument("--distance_threshold", help="distance threshold for qtcb <-> qtcc transition. Only QTCBC", type=float)
    parser.add_argument("-c", "--config", help="config file", type=str)
    parser.add_argument("--ros", action="store_true", default=False, help="Use ROS eco-system")
    args = parser.parse_args()

    if args.qsr in options:
        which_qsr = args.qsr
    else:
        raise ValueError("qsr not found, keywords: %s" % options)

    world = World_Trace()

    dynamic_args = {}

    if which_qsr in ["rcc2", "rcc3", "ra"]:
        dynamic_args = {which_qsr: {"quantisation_factor": args.quantisation_factor}}
        o1 = [Object_State(name="o1", timestamp=0, x=1., y=1., xsize=5., ysize=8.),
              Object_State(name="o1", timestamp=1, x=1., y=2., xsize=5., ysize=8.),
              Object_State(name="o1", timestamp=2, x=1., y=3., xsize=5., ysize=8.)]

        o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., xsize=5., ysize=8.),
              Object_State(name="o2", timestamp=1, x=11., y=2., xsize=5., ysize=8.),
              Object_State(name="o2", timestamp=2, x=11., y=3., xsize=5., ysize=8.),
              Object_State(name="o2", timestamp=3, x=11., y=4., xsize=5., ysize=8.)]

        o3 = [Object_State(name="o3", timestamp=0, x=1., y=11., xsize=5., ysize=8.),
              Object_State(name="o3", timestamp=1, x=2., y=11., xsize=5., ysize=8.),
              Object_State(name="o3", timestamp=2, x=3., y=11., xsize=5., ysize=8.)]

        world.add_object_state_series(o1)
        world.add_object_state_series(o2)
        world.add_object_state_series(o3)

    elif which_qsr == "tpcc":
        # Then we need three objects to do a test...
        # Don't bother with more than one timestamp.
        o1 = [Object_State(name="o1", timestamp=0, x=0., y=0., xsize=5., ysize=8.)]
        o2 = [Object_State(name="o2", timestamp=0, x=5., y=0., xsize=5., ysize=8.)]
        o3 = [Object_State(name="o3", timestamp=0, x=5., y=0., xsize=5., ysize=8.)]

        world.add_object_state_series(o1)
        world.add_object_state_series(o2)
        world.add_object_state_series(o3)

    elif which_qsr == "rcc8" or which_qsr == "rcc5":
        # dynamic_args = {which_qsr: {"quantisation_factor": args.quantisation_factor}}
        o1 = [Object_State(name="o1", timestamp=0, x=1., y=1., xsize=5., ysize=8.),
              Object_State(name="o1", timestamp=1, x=1., y=2., xsize=5., ysize=8.),
              Object_State(name="o1", timestamp=2, x=1., y=3., xsize=5., ysize=8.)]

        o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., xsize=5., ysize=8.),
              Object_State(name="o2", timestamp=1, x=11., y=2., xsize=5., ysize=8.),
              Object_State(name="o2", timestamp=2, x=11., y=3., xsize=5., ysize=8.),
              Object_State(name="o2", timestamp=3, x=11., y=4., xsize=5., ysize=8.)]

        o3 = [Object_State(name="o3", timestamp=0, x=1., y=11., xsize=5., ysize=8.),
              Object_State(name="o3", timestamp=1, x=2., y=11., xsize=5., ysize=8.),
              Object_State(name="o3", timestamp=2, x=3., y=11., xsize=5., ysize=8.)]

        o4 = [Object_State(name="o4", timestamp=0, x=1., y=11., xsize=7., ysize=9.),
              Object_State(name="o4", timestamp=1, x=2., y=11., xsize=7., ysize=9.),
              Object_State(name="o4", timestamp=2, x=3., y=11., xsize=7., ysize=9.)]

        world.add_object_state_series(o1)
        world.add_object_state_series(o2)
        world.add_object_state_series(o3)
        world.add_object_state_series(o4)

    elif which_qsr == "mos":
        dynamic_args = {which_qsr: {"quantisation_factor": args.quantisation_factor}}

        o1 = [Object_State(name="o1", timestamp=0, x=1., y=1., xsize=5., ysize=8.),
              Object_State(name="o1", timestamp=1, x=2., y=1., xsize=5., ysize=8.),
              Object_State(name="o1", timestamp=2, x=3., y=1., xsize=5., ysize=8.)]

        o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., xsize=5., ysize=8.),
              Object_State(name="o2", timestamp=1, x=11., y=10., xsize=5., ysize=8.),
              Object_State(name="o2", timestamp=2, x=11., y=20., xsize=5., ysize=8.),
              Object_State(name="o2", timestamp=3, x=11., y=30., xsize=5., ysize=8.)]

        world.add_object_state_series(o1)
        world.add_object_state_series(o2)

    elif which_qsr == "argd" or which_qsr == "argprobd":
        qsr_relations_and_values = {"0": 5., "1": 15., "2": 100.} if which_qsr == "argd" else {"0": (2.5,2.5/2), "1": (7.5,7.5/2), "2": [50,50/2]}
        dynamic_args = {which_qsr: {"qsr_relations_and_values": qsr_relations_and_values}}

        o1 = [Object_State(name="o1", timestamp=0, x=1., y=1., xsize=5., ysize=8.),
              Object_State(name="o1", timestamp=1, x=1., y=2., xsize=5., ysize=8.),
              Object_State(name="o1", timestamp=2, x=1., y=2., xsize=5., ysize=8.)]

        o2 = [Object_State(name="o2", timestamp=0, x=1., y=1., xsize=5., ysize=8.),
              Object_State(name="o2", timestamp=1, x=11., y=2., xsize=5., ysize=8.),
              Object_State(name="o2", timestamp=2, x=21., y=2., xsize=5., ysize=8.)]

        o3 = [Object_State(name="o3", timestamp=0, x=21., y=1., xsize=5., ysize=8.),
              Object_State(name="o3", timestamp=1, x=31., y=2., xsize=5., ysize=8.),
              Object_State(name="o3", timestamp=2, x=41., y=2., xsize=5., ysize=8.)]

        world.add_object_state_series(o1)
        world.add_object_state_series(o2)
        world.add_object_state_series(o3)

    elif which_qsr == "cardir":
        o1 = [Object_State(name="o1", timestamp=0, x=5., y=5., xsize=2., ysize=2.),
              Object_State(name="o1", timestamp=1, x=5., y=5., xsize=2., ysize=2.),
              Object_State(name="o1", timestamp=2, x=5., y=5., xsize=2., ysize=2.)]

        o2 = [Object_State(name="o2", timestamp=0, x=8., y=8., xsize=2., ysize=2.),
              Object_State(name="o2", timestamp=1, x=6., y=8., xsize=2., ysize=2.),
              Object_State(name="o2", timestamp=2, x=4., y=8., xsize=2., ysize=2.),
              Object_State(name="o2", timestamp=3, x=2., y=8., xsize=2., ysize=2.)]

        o3 = [Object_State(name="o3", timestamp=0, x=3., y=3., xsize=2., ysize=2.),
              Object_State(name="o3", timestamp=1, x=4., y=3., xsize=2., ysize=2.),
              Object_State(name="o3", timestamp=2, x=6., y=3., xsize=2., ysize=2.)]

        o4 = [Object_State(name="o4", timestamp=0, x=4., y=11., xsize=7., ysize=9.),
              Object_State(name="o4", timestamp=1, x=6., y=11., xsize=7., ysize=9.),
              Object_State(name="o4", timestamp=2, x=8., y=11., xsize=7., ysize=9.)]

        world.add_object_state_series(o1)
        world.add_object_state_series(o2)
        world.add_object_state_series(o3)
        world.add_object_state_series(o4)

    elif which_qsr == "qtcbs":
        dynamic_args = {which_qsr: {
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

            o3 = [Object_State(name="o3", timestamp=0, x=4., y=1.),
                  Object_State(name="o3", timestamp=1, x=4., y=1.),
                  Object_State(name="o3", timestamp=2, x=5., y=1.)]

            o4 = [Object_State(name="o4", timestamp=0, x=14., y=11.),
                  Object_State(name="o4", timestamp=1, x=14., y=11.)]

            world.add_object_state_series(o1)
            world.add_object_state_series(o2)
            world.add_object_state_series(o3)
            # world.add_object_state_series(o4)  # test for missing values

    elif which_qsr == "qtccs":
        dynamic_args = {which_qsr: {
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

            o4 = [Object_State(name="o4", timestamp=0, x=14., y=11.),
                  Object_State(name="o4", timestamp=1, x=14., y=11.)]

            world.add_object_state_series(o1)
            world.add_object_state_series(o2)
            # world.add_object_state_series(o4)  # test for missing values

    elif which_qsr == "qtcbcs":
        dynamic_args = {which_qsr: {
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

            o4 = [Object_State(name="o4", timestamp=0, x=14., y=11.),
                  Object_State(name="o4", timestamp=1, x=14., y=11.)]

            world.add_object_state_series(o1)
            world.add_object_state_series(o2)
            # world.add_object_state_series(o4)  # test for missing values

    elif which_qsr == "multiple":
        which_qsr = multiple
        traj = [Object_State(name="traj", timestamp=0, x=1., y=1., xsize=5., ysize=8.),
            Object_State(name="traj", timestamp=1, x=1., y=2., xsize=5., ysize=8.)]
        o1 = [Object_State(name="o1", timestamp=0, x=11., y=1., xsize=5., ysize=8.),
              Object_State(name="o1", timestamp=1, x=11., y=2., xsize=5., ysize=8.)]
        o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., xsize=5., ysize=8.),
              Object_State(name="o2", timestamp=1, x=11., y=2., xsize=5., ysize=8.)]

        world.add_object_state_series(traj)
        world.add_object_state_series(o1)

    # # DBG: testing qsrs_for
    # try:
    #     dynamic_args[which_qsr]["qsrs_for"] = [("o1", "o2"), ("o1", "o3")]
    # except KeyError:
    #     dynamic_args[which_qsr] = {"qsrs_for": [("o1", "o3"), ("o1", "o3")]}
    # try:
    #     dynamic_args[which_qsr]["qsrs_for"] = ["o1"]
    # except KeyError:
    #     dynamic_args[which_qsr] = {"qsrs_for": ["o1"]}
    # dynamic_args["for_all_qsrs"] = {"qsrs_for": [("o1", "o2"), "o2"]}
    # try:
    #     print(dynamic_args[which_qsr]["qsrs_for"])
    # except KeyError:
    #     print("qsrs_for not set in which_qsr namespace")
    # print(dynamic_args["for_all_qsrs"]["qsrs_for"])
    # # DBG: eof

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
