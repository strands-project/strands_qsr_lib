#!/usr/bin/env python
from __future__ import print_function, division
import random
import csv
import argparse
from collections import OrderedDict


def gen_random_2d_points_for_3_objects(N, MIN, MAX):
    rng = random.randint if isinstance(MIN, int) and isinstance(MAX, int) else random.uniform
    ordered_fieldnames = OrderedDict([('o1', None),
                                      ('x1', None),
                                      ('y1', None),
                                      ('o2', None),
                                      ('x2', None),
                                      ('y2', None),
                                      ('o3', None),
                                      ('x3', None),
                                      ('y3', None)])
    data = []
    for i in range(N):
        data.append({"o1": "o1",
                     "x1": round(rng(MIN, MAX), 2),
                     "y1": round(rng(MIN, MAX), 2),
                     "o2": "o2",
                     "x2": round(rng(MIN, MAX), 2),
                     "y2": round(rng(MIN, MAX), 2),
                     "o3": "o3",
                     "x3": round(rng(MIN, MAX), 2),
                     "y3": round(rng(MIN, MAX), 2)})
    return data, ordered_fieldnames


def gen_random_2d_bbs_for_3_objects(N, MIN, MAX, DMIN=3, DMAX=6):
    if DMIN > DMAX:
        raise ValueError("DMIN can't be less than DMAX")
    rng = random.randint if isinstance(MIN, int) and isinstance(MAX, int) else random.uniform
    ordered_fieldnames = OrderedDict([('o1', None),
                                      ('x1', None),
                                      ('y1', None),
                                      ('w1', None),
                                      ('l1', None),
                                      ('o2', None),
                                      ('x2', None),
                                      ('y2', None),
                                      ('w2', None),
                                      ('l2', None),
                                      ('o3', None),
                                      ('x3', None),
                                      ('y3', None),
                                      ('w3', None),
                                      ('l3', None)])
    data = []
    for i in range(N):
        data.append({"o1": "o1",
                     "x1": round(rng(MIN, MAX), 2),
                     "y1": round(rng(MIN, MAX), 2),
                     "w1": random.randint(DMIN, DMAX),
                     "l1": random.randint(DMIN, DMAX),
                     "o2": "o2",
                     "x2": round(rng(MIN, MAX), 2),
                     "y2": round(rng(MIN, MAX), 2),
                     "w2": random.randint(DMIN, DMAX),
                     "l2": random.randint(DMIN, DMAX),
                     "o3": "o3",
                     "x3": round(rng(MIN, MAX), 2),
                     "y3": round(rng(MIN, MAX), 2),
                     "w3": random.randint(DMIN, DMAX),
                     "l3": random.randint(DMIN, DMAX),})
    return data, ordered_fieldnames



if __name__ == '__main__':
    options = {"2dp3o": gen_random_2d_points_for_3_objects,
               "2dbb3o": gen_random_2d_bbs_for_3_objects}
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode", required=True, type=str,
                        help="what data to generate %s" % sorted(options.keys()))
    parser.add_argument("-o", "--output", required=True, type=str, help="output filename")
    parser.add_argument("-n", default=1000, type=int, help="number of lines")
    parser.add_argument("--min", default=0, type=int, help="min value of the random generator")
    parser.add_argument("--max", default=50, type=int, help="max value of the random generator")
    parser.add_argument("-f", "--floats", default=False, action="store_true", help="whether min/max are floats")
    args = parser.parse_args()
    if args.mode not in options:
        raise ValueError("mode not found, options are: %s" % sorted(options.keys()))

    N = args.n
    MIN = float(args.min) if args.floats else int(args.min)
    MAX = float(args.max) if args.floats else int(args.max)
    print("Generating for N, MIN, MAX, FLOATS:", N, MIN, MAX, args.floats)
    data, ordered_fieldnames = options[args.mode](N, MIN, MAX)
    with open(args.output, "w") as f:
        print("writing to", args.output)
        writer = csv.DictWriter(f, ordered_fieldnames)
        writer.writeheader()
        writer.writerows(data)
    print("done")
