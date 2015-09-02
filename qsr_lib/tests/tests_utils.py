#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import json
import csv
from roslib.packages import find_resource
from qsrlib_io.world_trace import Object_State, World_Trace

PKG = 'qsr_lib'

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

def load_input_data1():
    world = World_Trace()
    ob = []
    from roslib.packages import find_resource
    TEST_INPUT_DATA = find_resource(PKG, 'data1.csv')[0]
    with open(TEST_INPUT_DATA) as f:
        reader = csv.DictReader(f)
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
