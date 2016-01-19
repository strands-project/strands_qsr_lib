#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
from  qsrrep_utils.hmm_model_creation import HMMModelCreation
from  qsrrep_utils.qtc_model_creation import QTCModelCreation
import numpy as np
import os
import json

types = {
    "hmm": HMMModelCreation,
    "qtc": QTCModelCreation
}

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('action', help="please choose an action: {%s}." % ', '.join(types.values()[0]().execute_function.keys()), type=str)
    parser.add_argument('type', help="please choose the type to create the model from/for: {%s}." % ', '.join(types.keys()), type=str)
    parser.add_argument('-o', '--output', help="output filename.", type=str, required=True)
    parser.add_argument('-i', '--input', help="reads a *.hmm file. Only required if 'type' is set to 'hmm'.", type=str, required=False)
    parser.add_argument('-f', '--force', help="override existing files with the same name.", action="store_true", required=False)
    parser.add_argument('--qtc_type', help="the qtc_type for which the model should be created.", type=str, required=False)
    parser.add_argument('--start_end', help="add a start and end state to qtc.", action="store_true", required=False, default=False)

    args = parser.parse_args()

    r = types[args.type]().execute_function[args.action](**vars(args))
    if not args.force:
        if os.path.isfile(args.output):
            print "'%s' exists, please choose a different file name or use [-f/--force] to override." % args.output
            exit()
#    print r, type(r)
    with open(args.output, 'w') as f:
        if isinstance(r, np.ndarray):
            np.savetxt(f, r)
        else:
            json.dump(r, f)
