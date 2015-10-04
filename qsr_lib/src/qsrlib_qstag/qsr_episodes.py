# -*- coding: utf-8 -*-
""" Compute QSR Episodes
    (compresses repeating QSRs into an interval they hold)
"""
from __future__ import print_function
import copy

def compute_episodes(world_qsr):
    """Compute a long list of episodes with the format:
       `[(objects), {spatial relations}, (start_frame, end_frame)]`.

       .. seealso:: For further details about QSR Episodes, refer to its :doc:`description. <../handwritten/qsrs/qstag/>`
       """
    episodes  = []
    obj_based_qsr_world = {}
    frames = world_qsr.get_sorted_timestamps()
    for frame in frames:
        for objs, qsrs in world_qsr.trace[frame].qsrs.items():
            if objs not in obj_based_qsr_world: obj_based_qsr_world[objs] = []
            obj_based_qsr_world[objs].append((frame, qsrs.qsr))

    for objs, frame_tuples in obj_based_qsr_world.items():
        epi_start, epi_rel = frame_tuples[0]
        epi_end  = copy.copy(epi_start)

        objects = objs.split(',')
        for (frame, rel) in obj_based_qsr_world[objs]:
            if rel == epi_rel:
                epi_end = frame
            else:
                episodes.append( (objects, epi_rel, (epi_start, epi_end)) )
                epi_start = epi_end = frame
                epi_rel = rel
        episodes.append( (objects, epi_rel, (epi_start, epi_end)) )
    return episodes
