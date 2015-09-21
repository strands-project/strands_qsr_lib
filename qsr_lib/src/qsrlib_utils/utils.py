# -*- coding: utf-8 -*-
from __future__ import print_function, division
import numpy
import yaml
from qsrlib_io.world_qsr_trace import World_QSR_Trace


def merge_world_qsr_traces(world_qsr_traces, qsr_type=""):
    """Merge a list of traces into one world_qsr_trace. It offers no protection versus overwriting previously
    existing relations.

    :param world_qsr_traces:  World QSR traces to be merged.
    :type world_qsr_traces: list or tuple of :class:`World_QSR_Trace <qsrlib_io.world_qsr_trace.World_QSR_Trace>` objects
    :param qsr_type: The QSR type of the merged object.
    :type qsr_type: str
    :return: Merged World QSR trace.
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
    """.. warning::
        Planned for removal. Use `numpy.isnan` method directly. Does not raise deprecation warning in case it
        shadows numpy's method.

    Check if nan. Uses `numpy.isnan` so can be omitted.

    :param x: Value to be checked.
    :type x: int or float
    :return: Whether nan or not.
    :rtype: bool
    """
    return numpy.isnan(x)

def flatten_list(l):
    """Flatten an irregular list, i.e. a list containing a mixture of iteratable and non-iteratable items, returning a generator object.

    .. note::
        Use `list(flatten_list(l))` to get a list back.

    :param l: List to be flattened.
    :type l: list or tuple
    :return: Flattened list as a generator.
    :rtype: generator
    """
    for el in l:
        if isinstance(el, (list, tuple)):
            for sub in flatten_list(el):
                yield sub
        else:
            yield el

def load_dynamic_args_from_file(path):
    """Load `dynamic_args` from a yaml file.

    :param path: Filename including its path.
    :type path: str
    :return: yaml document.
    :rtype: dict
    """
    with open(path, "r") as f:
        return yaml.load(f)
