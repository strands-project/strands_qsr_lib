# -*- coding: utf-8 -*-
from __future__ import print_function, division
import numpy
import yaml
from qsrlib_io.world_qsr_trace import World_QSR_Trace


def merge_world_qsr_traces(world_qsr_traces, qsr_type=""):
    """
    Merge a list of traces into one world_qsr_trace. It offers no protection versus overwriting previously
    existing relation.
    :param world_qsr_traces: list of World_QSR_Trace objects
        :type world_qsr_traces: list or tuple
    :param qsr_type: the qsr_type of the returned merged World_QSR_Trace object
        :type qsr_type: str
    :return: a World_QSR_Trace that is the merge of all World_QSR_Trace objects in traces
        :rtype: World_QSR_Trace
    """
    ret_world_qsr_trace = World_QSR_Trace(qsr_type=qsr_type)
    for world_qsr_trace in world_qsr_traces:
        for t, s in world_qsr_trace.trace.items():
            for k, qsr_obj in s.qsrs.items():
                for qsr_k, qsr_v in qsr_obj.qsr.items():
                    try:
                        ret_world_qsr_trace.trace[t].qsrs[k].qsr[qsr_k] = qsr_v
                    except KeyError:
                        ret_world_qsr_trace.add_qsr(qsr_obj, t)
    return ret_world_qsr_trace

def isnan(x):
    return numpy.isnan(x)

def flatten_list(l):
    """Flattens an irregular list, i.e. a list containing a mixture of iteratable and non-iteratable items, returning a generator object.

    :param l: The list to flatten.
    :type l: list or tuple
    :return: Flattened list as a generator. Use `list(flatten_list(l))` to get a list back.
    :rtype: generator
    """
    for el in l:
        if isinstance(el, (list, tuple)):
            for sub in flatten_list(el):
                yield sub
        else:
            yield el

def load_dynamic_args_from_file(path):
    with open(path, "r") as f:
        return yaml.load(f)
