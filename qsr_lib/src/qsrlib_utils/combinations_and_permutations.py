# -*- coding: utf-8 -*-
from __future__ import print_function, division
import itertools

def possible_pairs(l, mirrors=True, sort_first=False, sort_return=False):
    if sort_first:
        l = sorted(l)
    ret = list(itertools.permutations(l, 2)) if mirrors else list(itertools.combinations(l, 2))
    return sorted(ret) if sort_return else ret

def possible_pairs_between_two_lists(l1, l2, mirrors=True, sort_return=False):
    ret = list(itertools.product(l1, l2)) + list(itertools.product(l2, l1)) if mirrors else list(itertools.product(l1, l2))
    return sorted(ret) if sort_return else ret

