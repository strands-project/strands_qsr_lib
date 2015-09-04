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

def unittest_get_multiple_qsrs_as_one_long_list(world_qsrs, which_qsrs):
    ret = []
    for t in world_qsrs.get_sorted_timestamps():
        for oname in sorted(world_qsrs.trace[t].qsrs.keys()):
            qsrs_str_list = []
            for which_qsr in which_qsrs:
                try:
                    qsrs_str_list.append(which_qsr + ":" + str(world_qsrs.trace[t].qsrs[oname].qsr[which_qsr]))
                except KeyError:
                    pass
            ret.append(",".join([str(t), oname, str(qsrs_str_list)]))
    return ret


def unittest_write_qsrs_as_one_long_list(qsrs_list, filename):
    with open(filename, "w") as f:
        json.dump(qsrs_list, f)

def unittest_read_qsrs_as_one_long_list(filename):
    with open(filename, "r") as f:
        return json.load(f)
