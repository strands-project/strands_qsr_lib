#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
from unittests_utils import *

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
                xsize=float(row['w1']),
                ysize=float(row['l1'])
            ))
            ob.append(Object_State(
                name=row['o2'],
                timestamp=idx+1,
                x=float(row['x2']),
                y=float(row['y2']),
                xsize=float(row["w2"]),
                ysize=float(row["l2"])
            ))

    world.add_object_state_series(ob)
    return world

def load_input_data2():
    world = World_Trace()
    ob = []
    from roslib.packages import find_resource
    TEST_INPUT_DATA = find_resource(PKG, 'data2.csv')[0]
    with open(TEST_INPUT_DATA) as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            ob.append(Object_State(
                name=row['o1'],
                timestamp=idx+1,
                x=float(row['x1']),
                y=float(row['y1']),
                xsize=float(row['w1']),
                ysize=float(row['l1'])
            ))
            ob.append(Object_State(
                name=row['o2'],
                timestamp=idx+1,
                x=float(row['x2']),
                y=float(row['y2']),
                xsize=float(row["w2"]),
                ysize=float(row["l2"])
            ))
            ob.append(Object_State(
                name=row['o3'],
                timestamp=idx+1,
                x=float(row['x3']),
                y=float(row['y3']),
                xsize=float(row["w3"]),
                ysize=float(row["l3"])
            ))

    world.add_object_state_series(ob)
    return world

def load_input_data3():
    world = World_Trace()
    ob = []
    from roslib.packages import find_resource
    TEST_INPUT_DATA = find_resource(PKG, 'data3.csv')[0]
    with open(TEST_INPUT_DATA) as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            ob.append(Object_State(
                name=row['o1'],
                timestamp=idx+1,
                x=float(row['x1']),
                y=float(row['y1'])
            ))
            ob.append(Object_State(
                name=row['o2'],
                timestamp=idx+1,
                x=float(row['x2']),
                y=float(row['y2'])
            ))
            ob.append(Object_State(
                name=row['o3'],
                timestamp=idx+1,
                x=float(row['x3']),
                y=float(row['y3'])
            ))

    world.add_object_state_series(ob)
    return world

def load_input_data4():
    world = World_Trace()
    ob = []
    from roslib.packages import find_resource
    TEST_INPUT_DATA = find_resource(PKG, 'data4.csv')[0]
    with open(TEST_INPUT_DATA) as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            ob.append(Object_State(
                name=row['o1'],
                timestamp=idx+1,
                x=float(row['x1']),
                y=float(row['y1']),
                xsize=float(row['w1']),
                ysize=float(row['l1'])
            ))
            ob.append(Object_State(
                name=row['o2'],
                timestamp=idx+1,
                x=float(row['x2']),
                y=float(row['y2']),
                xsize=float(row["w2"]),
                ysize=float(row["l2"])
            ))
            ob.append(Object_State(
                name=row['o3'],
                timestamp=idx+1,
                x=float(row['x3']),
                y=float(row['y3']),
                xsize=float(row["w3"]),
                ysize=float(row["l3"])
            ))

    world.add_object_state_series(ob)
    return world








def load_input_data2_first100():
    world = World_Trace()
    ob = []
    from roslib.packages import find_resource
    TEST_INPUT_DATA = find_resource(PKG, 'data2_first100.csv')[0]
    with open(TEST_INPUT_DATA) as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            ob.append(Object_State(
                name=row['o1'],
                timestamp=idx+1,
                x=float(row['x1']),
                y=float(row['y1']),
                xsize=float(row['w1']),
                ysize=float(row['l1'])
            ))
            ob.append(Object_State(
                name=row['o2'],
                timestamp=idx+1,
                x=float(row['x2']),
                y=float(row['y2']),
                xsize=float(row["w2"]),
                ysize=float(row["l2"])
            ))
            ob.append(Object_State(
                name=row['o3'],
                timestamp=idx+1,
                x=float(row['x3']),
                y=float(row['y3']),
                xsize=float(row["w3"]),
                ysize=float(row["l3"])
            ))

    world.add_object_state_series(ob)
    return world

def load_input_data3_first100():
    world = World_Trace()
    ob = []
    from roslib.packages import find_resource
    TEST_INPUT_DATA = find_resource(PKG, 'data3_first100.csv')[0]
    with open(TEST_INPUT_DATA) as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            ob.append(Object_State(
                name=row['o1'],
                timestamp=idx+1,
                x=float(row['x1']),
                y=float(row['y1'])
            ))
            ob.append(Object_State(
                name=row['o2'],
                timestamp=idx+1,
                x=float(row['x2']),
                y=float(row['y2'])
            ))
            ob.append(Object_State(
                name=row['o3'],
                timestamp=idx+1,
                x=float(row['x3']),
                y=float(row['y3'])
            ))

    world.add_object_state_series(ob)
    return world

def load_input_data4_first100():
    world = World_Trace()
    ob = []
    from roslib.packages import find_resource
    TEST_INPUT_DATA = find_resource(PKG, 'data4_first100.csv')[0]
    with open(TEST_INPUT_DATA) as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            ob.append(Object_State(
                name=row['o1'],
                timestamp=idx+1,
                x=float(row['x1']),
                y=float(row['y1']),
                xsize=float(row['w1']),
                ysize=float(row['l1'])
            ))
            ob.append(Object_State(
                name=row['o2'],
                timestamp=idx+1,
                x=float(row['x2']),
                y=float(row['y2']),
                xsize=float(row["w2"]),
                ysize=float(row["l2"])
            ))
            ob.append(Object_State(
                name=row['o3'],
                timestamp=idx+1,
                x=float(row['x3']),
                y=float(row['y3']),
                xsize=float(row["w3"]),
                ysize=float(row["l3"])
            ))

    world.add_object_state_series(ob)
    return world
