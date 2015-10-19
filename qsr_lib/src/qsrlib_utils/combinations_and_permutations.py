# -*- coding: utf-8 -*-
from __future__ import print_function, division
import itertools


def possible_pairs(s, mirrors=True):
    """Return possible pairs from a set of values.

    Assume `s = ['a', 'b']`. Then return examples for the following calls are:

        * `possible_pairs(s)` returns `[('a', 'b'), ('b', 'a')]`
        * `possible_pairs(s, mirros=False)` returns `[('a', 'b')]`

    :param s: Names of the elements from which the pairs will be created.
    :type s: set or list or tuple
    :param mirrors: Include mirrors or not.
    :type mirrors: bool
    :return: List of pairs as tuples.
    :rtype: list of tuples of str
    """
    if len(s) != len(set(s)):
        raise ValueError('duplicates found, but not allowed')
    return list(itertools.permutations(s, 2)) if mirrors else list(itertools.combinations(s, 2))

def possible_pairs_between_two_lists(s1, s2, mirrors=True):
    """Return possible pairs between the elements of two sets.

    Assume `s1 = ['a', 'b']` and `s2 = ['c', 'd']`. Then return examples for the following calls are:

        * `possible_pairs_between_two_lists(s1, s2)` returns `[('a', 'c'), ('a', 'd'), ('b', 'c'), ('b', 'd'), ('c', 'a'), ('c', 'b'), ('d', 'a'), ('d', 'b')]`.
        * `possible_pairs_between_two_lists(s1, s2, mirrors=False)` returns [('a', 'c'), ('a', 'd'), ('b', 'c'), ('b', 'd')].

    :param s1: Names of the first elements.
    :type s1: set or list or tuple
    :param s2: Names of the second elements.
    :type s2: set or list or tuple
    :param mirrors: Include mirrors or not.
    :type mirrors: bool
    :return: List of pairs as tuples.
    :rtype: list of tuples of str
    """
    if len(s1) != len(set(s1)) or len(s2) != len(set(s2)):
        raise ValueError('duplicates found, but not allowed')
    return list(itertools.product(s1, s2)) + list(itertools.product(s2, s1)) if mirrors else list(itertools.product(s1, s2))

def possible_triplets(s, mirrors=True):
    """
    Return the possible triplets from the list s.
    """
    if len(s) != len(set(s)):
        raise ValueError('duplicates found, but not allowed')
    return list(itertools.permutations(s, 3)) if mirrors else list(itertools.combinations(s, 3))
