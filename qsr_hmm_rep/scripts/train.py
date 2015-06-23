#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 30 15:28:58 2015

@author: cdondrup
"""

import rospy
import argparse
from hmmrep_lib.hmmrep_lib import HMMRepLib
from hmmrep_lib.hmmrep_requests import HMMRepRequestCreate, HMMRepRequestSample, HMMRepRequestLogLikelihood
import json
import os
import numpy as np


class Train(object):
    """trains hmm from qtc data files"""

    __hmm_types_available = {
        "qtcc": "qtcc_hmm",
        "qtcb": "qtcb_hmm",
        "qtcbc": "qtcbc_hmm"
    }

    def __init__(self, name, args):
        """Creates a new instance of the train class
        """
        rospy.loginfo("Starting %s", name)
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
        xml = self.hmm_lib.request(req).get_xml()
        return xml, qtc

    def sample(self, hmm):
        req = HMMRepRequestSample(qsr_type=self.__hmm_types_available[self.qsr], xml=hmm, max_length=10, num_samples=5)
        return self.hmm_lib.request(req).get_sample()

    def log_likelihood(self, hmm, samples):
        req = HMMRepRequestLogLikelihood(qsr_type=self.__hmm_types_available[self.qsr], xml=hmm, qsr_seq=samples)
        return self.hmm_lib.request(req).get_log_likelihood()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("qsr", help="choose qsr: qtcc|qtcc|qtcbc", type=str)
    parser.add_argument("-i", "--input", help="path under which to read csv files", type=str)
    args = parser.parse_args()

    rospy.init_node("train")
    t = Train(rospy.get_name(), args)
    hmm, qtc = t.train(args.input)
    s = t.sample(hmm=hmm)
    print "SAMPLES", s
    l = t.log_likelihood(hmm=hmm, samples=s)
    print "LOGLIKELIHOOD", l
