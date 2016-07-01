# -*- coding: utf-8 -*-
from __future__ import print_function
import sys
import numpy as np
from qsrlib_io.world_qsr_trace import World_QSR_Trace


def apply_median_filter(qsr_world, params):
    """
    Function to apply a median filter to the QSRLib World Trace
    ..seealso:: For further details about Filters, refer to its :doc:`description. <../handwritten/filters/>`


    :param qsr_world: A World_QSR_Trace object containing the data to be filtered
    :type qsrlib_io.World_QSR_Trace
    :param params: A dictionary with key = "window" and value as an int
    :type dict

    :return: A World_QSR_Trace object containing the filtered data
    :rtype: qsrlib_io.World_QSR_Trace
    """
    if not isinstance(qsr_world, World_QSR_Trace):
        raise RuntimeError("Applying Median Filter. qsr_world should be of type qsrlib_io.World_QSR_Trace")

    frames = qsr_world.get_sorted_timestamps()
    requested_qsrs = qsr_world.qsr_type.split(",")
    # print("all frames:", len(frames))
    # print("qsrs requested:", qsr_world.qsr_type)

    # Obtain the QSR data for each object set, and each qsr type.
    obj_based_qsr_world = {}
    for frame in frames:
        for objs, qsrs in qsr_world.trace[frame].qsrs.items():
            if objs not in obj_based_qsr_world:
                obj_based_qsr_world[objs] = {}
                for qsr_type in requested_qsrs:
                    # print("adding data:", qsr_type)
                    obj_based_qsr_world[objs][qsr_type] = []
                    obj_based_qsr_world[objs][qsr_type+"_frames"] = []

            for qsr_type, qsr in qsrs.qsr.items():
                obj_based_qsr_world[objs][qsr_type].append(qsr)
                obj_based_qsr_world[objs][qsr_type+"_frames"].append(frame)

    # Apply the Median Filter to each list of QSR seperately
    for objs, data in obj_based_qsr_world.items():
        for qsr_type in requested_qsrs:
            # print("filtering:", qsr_type)

            if data[qsr_type] != []:
                obj_based_qsr_world[objs][qsr_type+"_filtered"] = median_filter(data[qsr_type], params["window"])

    # check the filtering works:
    # for cnt, (i,j) in enumerate(zip(obj_based_qsr_world[objs]['qtcbs'], obj_based_qsr_world[objs]['qtcbs_filtered'])):
    #     print(cnt, "(",i,")", "   ", "(",j,")")

    # Overwrite the original QSR data with the filtered data, at the appropriate timepoints (merging QSR types back together in the process)
    for frame in frames:
        for objs, data in obj_based_qsr_world.items():
            new_qsrs = {}
            for qsr_type in requested_qsrs:
                if frame in data[qsr_type+"_frames"]:
                    ind = data[qsr_type+"_frames"].index(frame)
                    # print("frame:", frame, qsr_type, "index:", ind)
                    new_qsrs[qsr_type] = data[qsr_type+"_filtered"][ind]

            # print("frame:", frame, "prev:", qsr_world.trace[frame].qsrs[objs].qsr, "new:", new_qsrs)
            if new_qsrs != {}:
                qsr_world.trace[frame].qsrs[objs].qsr = new_qsrs
    return qsr_world



def median_filter(data, n=3):
    """
    Function to filter over 1 dimensional data, using window of size n
    n must be odd and >2, or the tail size will be 0; and will be floor(n/2).

    :param data: one dimensional list of QSR states
    :type list
    :param n: the window the median filter is applied to
    :type int

    :return: a one dimensional list of filtered QSR states
    :rtype: list
    """
    if len(data) < 2*n+1:
        #RuntimeWarning("Median Filter Window is larger than the data (will return data)")
        print("something stupid. Mean Filter window larget than number of frames...")
        return data

    """initial window < size of window. Add most common to all."""
    initial_window = data[0:n]
    counts, value = get_counts_from_window(initial_window)
    #if the max counted relation is unique, fill the window with this.
    if counts.count(max(counts)) is 1:
        ret = [value]*n
    #otherwise, just pick one as they have no ordinal information
    else:
        print("relation is not unique in the initial window of size %s, selecting the latest relation: %s" % (n, initial_window[-1]))
        ret = [initial_window[-1]]*n

    """continue with windows of size 2*n"""

    for i in range(n, len(data)):
        #window should be next n values. With previous n values (already filtered)
        next_n = data[i: i+n+1]
        prev_n = ret[i-n: i]
        window = prev_n + next_n
        # print(window)
        counts, value = get_counts_from_window(window)

        # If the max counted relation is unique, add it
        if counts.count(max(counts)) is 1:
            ret.append(value)
        # If multiple relations have same count, - add the previous relation
        else:
            ret.append(ret[-1])
    return ret

def get_counts_from_window(window):
    """count the number of different relations in a window. return the counds and winning value"""
    counts, values = [], []
    elms = [p for p in window]
    for x in set(elms):
        counts.append(elms.count(x))
        values.append(x)
    value = values[np.argmax(counts)]
    return counts, value
