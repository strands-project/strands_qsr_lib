# -*- coding: utf-8 -*-
from __future__ import print_function, division
from qsrlib_qsrs.qsr_triadic_abstractclass import QSR_Triadic_1t_Abstractclass
import math


import sys
import math

NUMBER_OF_PARTITIONS = 8
PARTITION_SIZE = (2.0 * math.pi) / NUMBER_OF_PARTITIONS  

def relative_angle(a, b, c):
    # compute relative angle (left/right/straight, front/back/straight)
    angle_BA = math.atan2((b.y - a.y),(b.x - a.x))

    if angle_BA < 0:
      angle_BA += 2 * math.pi

    angle_CB = math.atan2((c.y - b.y), (c.x - b.x))
    if angle_CB < 0:
       angle_CB += 2 * math.pi

    angle_rel = angle_CB - angle_BA
    if angle_rel < 0:
        angle_rel += 2 * math.pi

    return angle_rel

def partition(angle):
    return int(angle / PARTITION_SIZE)

def partition_name(p):
    if p == 0:
        return 'bl'
    elif p == 1:
        return 'lb'
    elif p == 2:
        return 'lf'
    elif p == 3:
        return 'fl'
    elif p == 4:
        return 'fr'
    elif p == 5:
        return 'rf'
    elif p == 6:
        return 'rb'
    else: # p ==7
        return 'br'
  

def calc_TPCC(origin, relatum, objct):

    base_distance = math.sqrt((origin.x-relatum.x)**2 + (origin.y-relatum.y)**2)
    object_distance  = math.sqrt((objct.x-relatum.x)**2 + (objct.y-relatum.y)**2)
    if base_distance == object_distance:
        return "sam"
    
    relation = "d" if object_distance > base_distance else "c" # is it far or close: first letter
    
    rela = relative_angle(origin, relatum, objct)
        
    part = partition(rela) #TODO: the "*s*" relations
    relation+=partition_name(part)

    return relation


class QSR_TPCC(QSR_Triadic_1t_Abstractclass):
    """ TPCC QSRs.
    .. seealso:: For further details about TPCC, see http://www.sfbtr8.spatial-cognition.de/project/r3/QualitativeCalculi/TPCC/index.html.
    """
    _unique_id = "tpcc"
    _all_possible_relations = ('dlf', 'dfl', 'dsl', 'dbl', 'dlb', 'dsb', 'drb', 'dbr',
                               'dsr', 'dfr', 'drf', 'dsf', 'clf', 'cfl', 'csl', 'cbl',
                               'clb', 'csb', 'crb', 'cbr', 'csr', 'cfr', 'crf', 'csf',
                               'sam')
    _dtype = "points"

    def __init__(self):
        """Constructor."""
        super(QSR_TPCC, self).__init__()

    def _compute_qsr(self, origin, relatum, objct, qsr_params, **kwargs):
        return calc_TPCC(origin, relatum, objct)
