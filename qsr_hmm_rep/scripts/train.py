#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 30 15:28:58 2015

@author: cdondrup
"""

import rospy
import argparse
#from hrsi_representation.file_input import FileInput
#from hrsi_representation.create_hmm import QTCHMM
from hmmrep_hmms.create_qtc_hmm import CreateQTCHMM
import json
import os
import numpy as np


class Train(object):
    """trains hmm from qtc data files"""

    def __init__(self, name, args):
        """Creates a new instance of the train class
        """
        rospy.loginfo("Starting %s", name)
        self.q = args.quantisation_factor
        self.v = True  # args.validate # Has to be validated
        self.n = args.no_collapse
        self.d = args.distance_threshold
        self.i = args.input
        self.qsr = args.qsr

#        self.file_input = FileInput()
        self.qtc_hmm = CreateQTCHMM()

    def _load_files(self, path):
        ret = []
        for f in os.listdir(path):
            if f.endswith(".qtc"):
                filename = path + '/' + f
                with open(filename, 'r') as qtc:
                    ret.append(np.array(json.load(qtc)))

        return ret



    def train(self, path):
        if self.qsr == 'qtcb':
            state_num = 11
        elif self.qsr == 'qtcc':
            state_num = 83
        elif self.qsr == 'qtcbc':
            state_num = 92

        rospy.loginfo("Reading file: '%s'" % self.i)
        qtc = self._load_files(path)
        hmm = self.qtc_hmm.create(qsr_seq=qtc, num_possible_states=state_num, qtc_type=self.qsr)
        return hmm, qtc


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("qsr", help="choose qsr: qtcc|qtcc|qtcbc", type=str)
    parser.add_argument("-i", "--input", help="path under which to read csv files", type=str)
    #parser.add_argument("--validate", help="validate state chain. Only QTC", action="store_true")
    parser.add_argument("--quantisation_factor", help="quantisation factor for 0-states.", type=float)
    parser.add_argument("--no_collapse", help="does not collapse similar adjacent states.", action="store_true")
    parser.add_argument("--distance_threshold", help="distance threshold for qtcb <-> qtcc transition. Only QTCBC", type=float)
    args = parser.parse_args()

    rospy.init_node("train")
    t = Train(rospy.get_name(), args)
    hmm, qtc = t.train(args.input)
    print qtc
    print hmm
    #rospy.spin()