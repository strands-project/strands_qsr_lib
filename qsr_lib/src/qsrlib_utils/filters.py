# -*- coding: utf-8 -*-
from __future__ import print_function
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
            obj_based_qsr_world[objs][qsr_type+"_filtered"] = median_filter(data[qsr_type], params["window"])


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
            qsr_world.trace[frame].qsrs[objs].qsr = new_qsrs
    return qsr_world


def median_filter(data, n=3):
    """
    Function to filter over 1 dimensional data, using window 2*n

    :param data: one dimensional list of QSR states
    :type list
    :param n: the window the median filter is applied to
    :type int

    :return: a one dimensional list of filtered QSR states
    :rtype: list
    """
    if len(data) < 2*n+1:
        #RuntimeWarning("Median Filter Window is larger than the data (will return data)")
        print("something stupid...")
        return data

    ret = data[0:n]
    for i in range(n, len(data)):
        window = data[i-n: i+n]
        elms = [p for p in window]
        counts, values = [], []
        for x in set(elms):
            counts.append(elms.count(x))
            values.append(x)
        value = values[np.argmax(counts)]

        # print("pre",ret[-1])
        # print(i-n, i+n, ":", elms)
        # print("c:", counts)
        # print("v:", value)

        # If ambiguity over which relation to add. Add previous.
        if counts.count(max(counts)) is 1:
            ret.append(value)
        else:
            # ambiguous - adding previous state
            ret.append(ret[-1])
    return ret
