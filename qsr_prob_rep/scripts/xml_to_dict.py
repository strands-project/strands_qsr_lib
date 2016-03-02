#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import ghmm as gh
import json
from qsrrep_lib.rep_hmm import RepHMM

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", help="The xml HMM file to convert", type=str)
    parser.add_argument("-o", "--output", help="The new filename", type=str)
    args = parser.parse_args()

    hmm = gh.HMMOpen(fileName=args.input, filetype=gh.GHMM_FILETYPE_XML)

    trans, emi, start = hmm.asMatrices()

    ret = {
        RepHMM.TRANS: trans,
        RepHMM.EMI: emi,
        RepHMM.START: start,
    }

    with open(args.output, 'w') as f:
        json.dump(ret, f)
