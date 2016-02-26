#!/usr/bin/env python
PKG = 'qsr_prob_rep'
NAME = 'hmm_tester'

import rospy
import sys
import unittest
from roslib.packages import find_resource
from qsrrep_ros.ros_client import ROSClient
from qsrrep_lib.rep_io_hmm import HMMRepRequestCreate, HMMRepRequestSample, HMMRepRequestLogLikelihood
import json
import hashlib


class TestHMM(unittest.TestCase):
    QTCB_SAMPLE_TEST_HMM  = find_resource(PKG, 'qtcb_sample_test.hmm')[0]
    QTCC_SAMPLE_TEST_HMM  = find_resource(PKG, 'qtcc_sample_test.hmm')[0]
    QTCBC_SAMPLE_TEST_HMM = find_resource(PKG, 'qtcbc_sample_test.hmm')[0]
    QTCB_PASSBY_LEFT_HMM  = find_resource(PKG, 'qtcb_passby_left.hmm')[0]
    QTCC_PASSBY_LEFT_HMM  = find_resource(PKG, 'qtcc_passby_left.hmm')[0]
    QTCBC_PASSBY_LEFT_HMM = find_resource(PKG, 'qtcbc_passby_left.hmm')[0]
    RCC3_TEST_HMM         = find_resource(PKG, 'rcc3_test.hmm')[0]
    QTCB_QSR              = find_resource(PKG, 'qtcb.qsr')[0]
    QTCC_QSR              = find_resource(PKG, 'qtcc.qsr')[0]
    QTCBC_QSR             = find_resource(PKG, 'qtcbc.qsr')[0]
    RCC3_QSR              = find_resource(PKG, 'rcc3.qsr')[0]

    correct_samples = {
        "qtcb": [[u'--', u'0-', u'+-', u'+0', u'++']],
        "qtcc": [[u'--+-', u'--0-', u'----', u'0---', u'+---', u'+0--', u'++--', u'---']],
        "qtcbc": [[u'--', u'----', u'0---', u'+---', u'+0--', u'++--', u'++']]
    }

    correct_hashsum = {
        "qtcb": "de1ea75d1b0d6c9ff8249a24583fedb9",
        "qtcc": "5d74fb27e53ba00f84014d3be70e2740",
        "qtcbc": "f73e2c85f1447a5109d6ab3d5201fb76",
        "rcc3": "402c53a1cc004a5f518d0b607eb7ac38"
    }

    correct_loglikelihoods ={
        "qtcb": -2.16887,
        "qtcc": -6.07188,
        "qtcbc": -2.23475,
        "rcc3": -4.15914
    }


    def __init__(self, *args):
        super(TestHMM, self).__init__(*args)

        rospy.init_node(NAME)

        self.r = ROSClient()

    def _create_hmm(self, qsr_file, qsr_type, start_at_zero=True):
        with open(qsr_file, 'r') as f: qsr_seq = json.load(f)
        d = self.r.call_service(
            HMMRepRequestCreate(
                qsr_seq=qsr_seq,
                qsr_type=qsr_type,
                start_at_zero=start_at_zero
            )
        )
        return d

    def _create_sample(self, hmm_file, qsr_type):
        with open(hmm_file, 'r') as f: hmm = json.load(f)
        s = self.r.call_service(
            HMMRepRequestSample(
                qsr_type=qsr_type,
                dictionary=hmm,
                max_length=10,
                num_samples=1
            )
        )
        return s

    def _calculate_loglikelihood(self, hmm_file, qsr_file, qsr_type):
        with open(qsr_file, 'r') as f: qsr_seq = json.load(f)
        with open(hmm_file, 'r') as f: hmm = json.load(f)
        l = self.r.call_service(
            HMMRepRequestLogLikelihood(
                qsr_type=qsr_type,
                dictionary=hmm,
                qsr_seq=qsr_seq
            )
        )
        return round(l, 5)

    def _to_strings(self, array):
        return [x.values()[0] for x in array]

    def test_qtcb_create(self):
        res = self._create_hmm(self.QTCB_QSR, 'qtcb')
        self.assertEqual(hashlib.md5(json.dumps(res)).hexdigest(), self.correct_hashsum["qtcb"])

    def test_qtcb_sample(self):
        res = self._create_sample(self.QTCB_SAMPLE_TEST_HMM, 'qtcb')
        self.assertEqual(res, self.correct_samples["qtcb"])

    def test_qtcb_loglikelihood(self):
        res = self._calculate_loglikelihood(self.QTCB_PASSBY_LEFT_HMM, self.QTCB_QSR, 'qtcb')
        self.assertEqual(res, self.correct_loglikelihoods["qtcb"])

    def test_qtcc_create(self):
        res = self._create_hmm(self.QTCC_QSR, 'qtcc')
        self.assertEqual(hashlib.md5(json.dumps(res)).hexdigest(), self.correct_hashsum["qtcc"])

    def test_qtcc_sample(self):
        res = self._create_sample(self.QTCC_SAMPLE_TEST_HMM, 'qtcc')
        self.assertEqual(res, self.correct_samples["qtcc"])

    def test_qtcc_loglikelihood(self):
        res = self._calculate_loglikelihood(self.QTCC_PASSBY_LEFT_HMM, self.QTCC_QSR, 'qtcc')
        self.assertEqual(res, self.correct_loglikelihoods["qtcc"])

    def test_qtcbc_create(self):
        res = self._create_hmm(self.QTCBC_QSR, 'qtcbc')
        self.assertEqual(hashlib.md5(json.dumps(res)).hexdigest(), self.correct_hashsum["qtcbc"])

    def test_qtcbc_sample(self):
        res = self._create_sample(self.QTCBC_SAMPLE_TEST_HMM, 'qtcbc')
        self.assertEqual(res, self.correct_samples["qtcbc"])

    def test_qtcbc_loglikelihood(self):
        res = self._calculate_loglikelihood(self.QTCBC_PASSBY_LEFT_HMM, self.QTCBC_QSR, 'qtcbc')
        self.assertEqual(res, self.correct_loglikelihoods["qtcbc"])

    def test_rcc3_create(self):
        res = self._create_hmm(self.RCC3_QSR, 'rcc3')
        self.assertEqual(hashlib.md5(json.dumps(res)).hexdigest(), self.correct_hashsum["rcc3"])

    def test_rcc3_sample(self):
        res = self._create_sample(self.RCC3_TEST_HMM, 'rcc3')
        self.assertTrue(type(res) == list and len(res) > 0)

    def test_rcc3_loglikelihood(self):
        res = self._calculate_loglikelihood(self.RCC3_TEST_HMM, self.RCC3_QSR, 'rcc3')
        self.assertEqual(res, self.correct_loglikelihoods["rcc3"])


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestHMM, sys.argv)
