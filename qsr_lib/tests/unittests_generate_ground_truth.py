#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import argparse
import csv
import os
import json
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace


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

def unittest_get_qsrs_as_one_long_list(world_qsrs):
    ret = []
    for t in world_qsrs.get_sorted_timestamps():
        for oname in sorted(world_qsrs.trace[t].qsrs.keys()):
            ret.append(",".join([str(t), oname, str(world_qsrs.trace[t].qsrs[oname].qsr)]))
    return ret

def unittest_write_qsrs_as_one_long_list(qsrs_list, filename):
    with open(filename, "w") as f:
        json.dump(qsrs_list, f)

def unittest_read_qsrs_as_one_long_list(filename):
    with open(filename, "r") as f:
        return json.load(f)

def load_file(TEST_FILE):
    world = World_Trace()
    ob = []
    with open(TEST_FILE) as csvfile:
        reader = csv.DictReader(csvfile)
        for idx, row in enumerate(reader):
            ob.append(Object_State(
                name=row['o1'],
                timestamp=idx+1,
                x=float(row['x1']),
                y=float(row['y1']),
                width=float(row['w1']),
                length=float(row['l1'])
            ))
            ob.append(Object_State(
                name=row['o2'],
                timestamp=idx+1,
                x=float(row['x2']),
                y=float(row['y2']),
                width=float(row["w2"]),
                length=float(row["l2"])
            ))
    world.add_object_state_series(ob)
    return world


if __name__ == "__main__":
    # ****************************************************************************************************
    # create a QSRlib object if there isn't one already
    qsrlib = QSRlib()

    # ****************************************************************************************************
    # parse command line arguments
    options = sorted(qsrlib.get_qsrs_registry().keys())
    parser = argparse.ArgumentParser()
    parser.add_argument("qsr", help="choose qsr: %s" % options, type=str)
    parser.add_argument("-i", "--input", required=True, help="input data csv file")
    parser.add_argument("-o", "--output", required=True, help="where to write json file")
    args = parser.parse_args()
    if args.qsr in options:
        which_qsr = args.qsr
    else:
        raise ValueError("qsr not found, keywords: %s" % options)

    # ****************************************************************************************************
    # load input data
    print("> Reading input from:", args.input)
    world = load_file(args.input)

    # ****************************************************************************************************
    # make a QSRlib request message
    dynamic_args = {}  # defaults
    # dynamic_args = {which_qsr: {"quantisation_factor": 2.0}}  # quantisation_factor
    ### monadic
    # dynamic_args = {"for_all_qsrs": {"qsrs_for": ["o2"]}}  # qsrs_for_global_namespace
    # dynamic_args = {which_qsr: {"qsrs_for": ["o1"]}}  # qsrs_for_qsr_namespace, qsrs_for_qsr_namespace_over_global_namespace
    # dynamic_args = {which_qsr: {"qsrs_for": ["o1"], "quantisation_factor": 2.0}}  # custom
    #### dyadic
    # dynamic_args = {"for_all_qsrs": {"qsrs_for": [("o2", "o1")]}}  # qsrs_for_global_namespace
    # dynamic_args = {which_qsr: {"qsrs_for": [("o1", "o2")]}}  # qsrs_for_qsr_namespace, qsrs_for_qsr_namespace_over_global_namespace
    # dynamic_args = {which_qsr: {"qsrs_for": [("o1", "o2")], "quantisation_factor": 2.0}}  # custom

    print("> Computing QSRs", which_qsr, dynamic_args)
    qsrlib_request_message = QSRlib_Request_Message(which_qsr, world, dynamic_args)
    # request your QSRs
    qsrlib_response_message = qsrlib.request_qsrs(qsrlib_request_message)

    # ****************************************************************************************************
    # save
    qsrs = qsrlib_response_message.qsrs
    print(len(qsrs.trace))
    t = qsrs.get_sorted_timestamps()[0]
    print(qsrs.trace[t].qsrs)
    qsrs_list = unittest_get_qsrs_as_one_long_list(qsrlib_response_message.qsrs)
    print("> Saving to:", args.output)
    unittest_write_qsrs_as_one_long_list(qsrs_list, args.output)
    print("> Validating...")
    qsrs_list_r = unittest_read_qsrs_as_one_long_list(args.output)
    print(">", len(world.trace), len(qsrs_list), len(qsrs_list_r))
    if qsrs_list != qsrs_list_r:
        raise RuntimeError("file written does not match the data, this should not have happened")

    # q-factor makes a difference
    foo = unittest_read_qsrs_as_one_long_list(args.output)
    filename = os.path.join(os.path.split(args.output)[0], "_".join(["data1", which_qsr, "defaults"]) + ".txt")  # quantisation_factor
    # filename = os.path.join(os.path.split(args.output)[0], "_".join(["data1", which_qsr, "qsrs_for_qsr_namespace"]) + ".txt")  # custom
    bar = unittest_read_qsrs_as_one_long_list(filename)
    print(foo == bar)

