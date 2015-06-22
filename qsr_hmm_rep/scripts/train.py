#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 30 15:28:58 2015

@author: cdondrup
"""

import rospy
import argparse
from hmmrep_lib.hmmrep_lib import HMMRepLib, HMMRepRequestCreate
import json
import os
import numpy as np


class Train(object):
    """trains hmm from qtc data files"""

    __hmm_types_available = {
        "qtcc": "create_qtcc_hmm",
        "qtcb": "create_qtcb_hmm",
        "qtcbc": "create_qtcbc_hmm"
    }

    def __init__(self, name, args):
        """Creates a new instance of the train class
        """
        rospy.loginfo("Starting %s", name)
#        self.q = args.quantisation_factor
#        self.v = True # Has to be validated
#        self.n = args.no_collapse
#        self.d = args.distance_threshold
        self.i = args.input
        self.qsr = args.qsr

        self.hmm_lib = HMMRepLib()

    def _load_files(self, path):
        ret = []
        for f in os.listdir(path):
            if f.endswith(".qtc"):
                filename = path + '/' + f
                with open(filename, 'r') as qtc:
                    ret.append(np.array(json.load(qtc)))

        return ret

    def train(self, path):
        rospy.loginfo("Reading file: '%s'" % self.i)
        qtc = self._load_files(path)
        req = HMMRepRequestCreate(qsr_seq=qtc, qsr_type=self.__hmm_types_available[self.qsr])
        xml, hmm = self.hmm_lib.request(req)
        #_ ,hmm = self.hmm_lib.create_hmm(qsr_seq=qtc, qsr_type=self.__hmm_types_available[self.qsr])
        return hmm, qtc


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("qsr", help="choose qsr: qtcc|qtcc|qtcbc", type=str)
    parser.add_argument("-i", "--input", help="path under which to read csv files", type=str)
#    parser.add_argument("--quantisation_factor", help="quantisation factor for 0-states.", type=float)
#    parser.add_argument("--no_collapse", help="does not collapse similar adjacent states.", action="store_true")
#    parser.add_argument("--distance_threshold", help="distance threshold for qtcb <-> qtcc transition. Only QTCBC", type=float)
    args = parser.parse_args()

    rospy.init_node("train")
    t = Train(rospy.get_name(), args)
    hmm, qtc = t.train(args.input)
    hmm.write('/home/cdondrup/test.xml')
    print qtc
    print hmm
