# -*- coding: utf-8 -*-
from __future__ import print_function, division
import itertools


# todo refact l to s representing set, maybe add `s = set(s)` at start
def possible_pairs(l, mirrors=True, sort_first=False, sort_return=False):
    """Return possible pairs from a set of values.

    Assume `l = ['a', 'b']`. Then return examples for the following calls are:
        * `possible_pairs(l)` returns `[('a', 'b'), ('b', 'a')]`
        * `possible_pairs(l, mirros=False)` returns `[('a', 'b')]`

    :param l: Names of the elements from which the pairs will be created.
    :type l: set or list or tuple
    :param mirrors: Include mirrors or not.
    :type mirrors: bool
    :param sort_first: Whether to sort `l` first or not.
    :type sort_return: bool
    :return: A list of pairs as tuples.
    :rtype: list
    """
    if sort_first:
        l = sorted(l)
    ret = list(itertools.permutations(l, 2)) if mirrors else list(itertools.combinations(l, 2))
    return sorted(ret) if sort_return else ret

# todo refactor l1, l2 to s1, s2 representing set, maybe add `s = set(s)` at start
def possible_pairs_between_two_lists(l1, l2, mirrors=True, sort_return=False):
    """Return possible pairs between the elements of two sets.

    Assume `l1 = ['a', 'b']` and `l2 = ['c', 'd']`. Then return examples for the following calls are:
        * `possible_pairs_between_two_lists(l1, l2)` returns `[('a', 'c'), ('a', 'd'), ('b', 'c'), ('b', 'd'), ('c', 'a'), ('c', 'b'), ('d', 'a'), ('d', 'b')]`.
        * `possible_pairs_between_two_lists(l1, l2, mirrors=False)` returns [('a', 'c'), ('a', 'd'), ('b', 'c'), ('b', 'd')].

    :param l1: Names of the first elements.
    :type l1: set or list or tuple
    :param l2: Names of the second elements.
    :param l2: set or list or tuple
    :param mirrors: Include mirrors or not.
    :param sort_return: Whether to sort returned list.
    :return: A list of pairs as tuples.
    :rtype: list
    """
    ret = list(itertools.product(l1, l2)) + list(itertools.product(l2, l1)) if mirrors else list(itertools.product(l1, l2))
    return sorted(ret) if sort_return else ret

