#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import sys
import os
import argparse
import random
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from unittests_data_loaders import *
from unittests_utils import *

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
    load_by_world_name = {"data1": load_input_data1,
                          "data2": load_input_data2,
                          "data3": load_input_data3,
                          "data4": load_input_data4,
                          "data2_first100": load_input_data2_first100,
                          "data3_first100": load_input_data3_first100,
                          "data4_first100": load_input_data4_first100}

    # ****************************************************************************************************
    # create a QSRlib object if there isn't one already
    qsrlib = QSRlib()

    # ****************************************************************************************************
    # parse command line arguments
    options = sorted(qsrlib.qsrs_registry.keys())
    parser = argparse.ArgumentParser()
    parser.add_argument("qsr", help="choose qsr: %s" % options, type=str)
    parser.add_argument("-i", "--input", required=True, type=str,
                        help="world name %s" % sorted(load_by_world_name.keys()))
    parser.add_argument("-o", "--output", required=True, type=str, help="where to write json file")
    args = parser.parse_args()
    if args.qsr in options:
        which_qsr = args.qsr
    elif args.qsr == "multiple":
        which_qsr = options[:]
    else:
        raise ValueError("qsr not found, keywords: %s" % options)
    if args.input not in load_by_world_name:
        raise ValueError("world name not found, options: %s" % sorted(load_by_world_name.keys()))

    # ****************************************************************************************************
    # load input data
    print("> Reading input from:", args.input)
    world = load_by_world_name[args.input]()

    # ****************************************************************************************************
    # make a QSRlib request message
    if args.qsr == "argprobd" or args.qsr == "multiple":
        random.seed(100)

    # defaults
    if args.qsr == "argd":
        dynamic_args = {"argd": {"qsr_relations_and_values": {"close": 10.0, "near": 20.0, "far": 30.0, "veryfar": 40.0}}}
    elif args.qsr == "argprobd":
        dynamic_args = {"argprobd": {"qsr_relations_and_values": {"close": (10, 10/2), "near": (20, 20/2),
                                                                  "far": (30, 30/2), "veryfar": (40, 40/2)}}}
    elif args.qsr == "multiple":
        dynamic_args = {"argd": {"qsr_relations_and_values": {"close": 10.0, "near": 20.0,
                                                              "far": 30.0, "veryfar": 40.0}},
                        "argprobd": {"qsr_relations_and_values": {"close": (10, 10/2), "near": (20, 20/2),
                                                                  "far": (30, 30/2), "veryfar": (40, 40/2)}},
                        "qtcbs": {"validate": False, "no_collapse": True},
                        "qtccs": {"validate": False, "no_collapse": True},
                        "qtcbcs": {"validate": False, "no_collapse": True}}
    else:
        dynamic_args = {}

    # quantisation_factor
    # dynamic_args = {args.qsr: {"quantisation_factor": 2.0}}

    ### monadic
    # dynamic_args = {"for_all_qsrs": {"qsrs_for": ["o2"]}}  # qsrs_for_global_namespace
    # dynamic_args = {args.qsr: {"qsrs_for": ["o1"]}}  # qsrs_for_qsr_namespace, qsrs_for_qsr_namespace_over_global_namespace
    # dynamic_args = {args.qsr: {"qsrs_for": ["o1"], "quantisation_factor": 2.0}}  # custom

    #### dyadic (except argd and argprobd, see below for these two
    # dynamic_args = {"for_all_qsrs": {"qsrs_for": [("o2", "o1")]}}  # qsrs_for_global_namespace
    # dynamic_args = {args.qsr: {"qsrs_for": [("o1", "o2")]}}  # qsrs_for_qsr_namespace, qsrs_for_qsr_namespace_over_global_namespace
    # dynamic_args = {args.qsr: {"qsrs_for": [("o1", "o2")], "quantisation_factor": 2.0}}  # custom

    #### triadic
    # dynamic_args = {"for_all_qsrs": {"qsrs_for": [("o3", "o2", "o1")]}}  # qsrs_for_global_namespace
    # dynamic_args = {args.qsr: {"qsrs_for": [("o1", "o2", "o3")]}}  # qsrs_for_qsr_namespace, qsrs_for_qsr_namespace_over_global_namespace

    #### qtcs special case
    # dynamic_args = {args.qsr: {"validate": True, "no_collapse": True}}
    # dynamic_args = {args.qsr: {"validate": True, "no_collapse": False}}
    # dynamic_args = {args.qsr: {"validate": False, "no_collapse": True}}
    # dynamic_args = {args.qsr: {"validate": False, "no_collapse": False}}

    #### argd special case
    # qsrs_for_global_namespace
    # dynamic_args = {"for_all_qsrs": {"qsrs_for": [("o2", "o1")]},
    #                 "argd": {"qsr_relations_and_values": {"close": 10.0, "near": 20.0, "far": 30.0, "veryfar": 40.0}}}
    # qsrs_for_qsr_namespace, qsrs_for_qsr_namespace_over_global_namespace
    # dynamic_args = {"for_all_qsrs": {"qsrs_for": [("o2", "o1")]},
    #                 "argd": {"qsr_relations_and_values": {"close": 10.0, "near": 20.0, "far": 30.0, "veryfar": 40.0},
    #                          "qsrs_for": [("o1", "o2")]}}

    #### argprobd special case
    # qsrs_for_global_namespace
    # dynamic_args = {"for_all_qsrs": {"qsrs_for": [("o2", "o1")]},
    #                 "argprobd": {"qsr_relations_and_values": {"close": (10, 10/2), "near": (20, 20/2),
    #                                                           "far": (30, 30/2), "veryfar": (40, 40/2)}}}
    # dynamic_args = {"for_all_qsrs": {"qsrs_for": [("o2", "o1")]},
    #                 "argprobd": {"qsr_relations_and_values": {"close": (10, 10/2), "near": (20, 20/2),
    #                                                           "far": (30, 30/2), "veryfar": (40, 40/2)},
    #                              "qsrs_for": [("o1", "o2")]}}

    #### multiple
    # qsrs_for_global_namespace
    # dynamic_args["for_all_qsrs"] = {"qsrs_for": [("o3", "o2", "o1"), ("o2", "o1"), "o2"]}
    # qsrs_for_qsr_namespace - cherry pick
    # dynamic_args["rcc2"] = {"qsrs_for": [("o1", "o2")]}
    # dynamic_args["qtcbs"]["qsrs_for"] = [("o1", "o2")]
    # dynamic_args["mwe"] = {"qsrs_for": [("o1", "o2")]}
    # dynamic_args["mos"] = {"qsrs_for": ["o1"]}
    # dynamic_args["tpcc"] = {"qsrs_for": [("o1", "o2", "o3")]}
    # qsrs_for_qsr_namespace_over_global - cherry pick
    # dynamic_args["for_all_qsrs"] = {"qsrs_for": [("o3", "o2", "o1"), ("o2", "o1"), "o2"]}
    # dynamic_args["rcc2"] = {"qsrs_for": [("o1", "o2")]}
    # dynamic_args["qtcbs"]["qsrs_for"] = [("o1", "o2")]
    # dynamic_args["mwe"] = {"qsrs_for": [("o1", "o2")]}
    # dynamic_args["mos"] = {"qsrs_for": ["o1"]}
    # dynamic_args["tpcc"] = {"qsrs_for": [("o1", "o2", "o3")]}

    print("> Computing QSRs", which_qsr, dynamic_args)
    qsrlib_request_message = QSRlib_Request_Message(which_qsr, world, dynamic_args)
    # request your QSRs
    qsrlib_response_message = qsrlib.request_qsrs(qsrlib_request_message)

    # ****************************************************************************************************
    # save
    qsrs = qsrlib_response_message.qsrs
    print(len(qsrs.trace))
    t = qsrs.get_sorted_timestamps()[1]
    print(qsrs.trace[t].qsrs)
    if args.qsr != "multiple":
        qsrs_list = unittest_get_qsrs_as_one_long_list(qsrs)
    else:
        qsrs_list = unittest_get_multiple_qsrs_as_one_long_list(qsrs, which_qsr)
    print("> Saving to:", args.output)
    unittest_write_qsrs_as_one_long_list(qsrs_list, args.output)
    print("> Validating...")
    qsrs_list_r = unittest_read_qsrs_as_one_long_list(args.output)
    print(">", len(world.trace), len(qsrs_list), len(qsrs_list_r))
    if qsrs_list != qsrs_list_r:
        raise RuntimeError("file written does not match the data, this should not have happened")

    # # q-factor makes a difference
    # foo = unittest_read_qsrs_as_one_long_list(args.output)
    # filename = os.path.join(os.path.split(args.output)[0], "_".join([args.input, which_qsr, "defaults"]) + ".txt")  # quantisation_factor
    # # filename = os.path.join(os.path.split(args.output)[0], "_".join(["data1", which_qsr, "qsrs_for_qsr_namespace"]) + ".txt")  # custom
    # bar = unittest_read_qsrs_as_one_long_list(filename)
    # print(foo == bar)
