#!/usr/bin/env python
PKG = 'qsr_prob_rep'
NAME = 'hmm_tester'

import rospy
import sys
import unittest
from roslib.packages import find_resource
from qsrrep_ros.ros_client import ROSClient
from qsrrep_lib.rep_io import HMMRepRequestCreate, HMMRepRequestSample, HMMRepRequestLogLikelihood
#from qsrlib_io.world_trace import Object_State, World_Trace
#from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client
import json
import hashlib


#   if args.action == "create":
#        qsr_seq = load_files(args.input)
#        q, d = r.call_service(
#            HMMRepRequestCreate(
#                qsr_seq=qsr_seq,
#                qsr_type=args.qsr_type
#            )
#        )
#        with open(args.output, 'w') as f: f.write(d)
#
#    elif args.action == "sample":
#        with open(args.input, 'r') as f: hmm = f.read()
#        q, s = r.call_service(
#            HMMRepRequestSample(
#                qsr_type=args.qsr_type,
#                xml=hmm,
#                max_length=args.max_length,
#                num_samples=args.num_samples
#            )
#        )
#        try:
#            with open(args.output, 'w') as f: json.dump(s, f)
#        except TypeError:
#            print s
#
#    elif args.action == "loglikelihood":
#        with open(args.qsr_seq, 'r') as f: qsr_seq = json.load(f)
#        with open(args.input, 'r') as f: hmm = f.read()
#        q, l = r.call_service(
#            HMMRepRequestLogLikelihood(
#                qsr_type=args.qsr_type,
#                xml=hmm,
#                qsr_seq=qsr_seq
#            )
#        )
#        print l


class TestHMM(unittest.TestCase):
    QTCB_SAMPLE_TEST_HMM  = find_resource(PKG, 'qtcb_sample_test.hmm')[0]
    QTCC_SAMPLE_TEST_HMM  = find_resource(PKG, 'qtcc_sample_test.hmm')[0]
    QTCBC_SAMPLE_TEST_HMM = find_resource(PKG, 'qtcbc_sample_test.hmm')[0]
    QTCB_PASSBY_LEFT_HMM  = find_resource(PKG, 'qtcb_passby_left.hmm')[0]
    QTCC_PASSBY_LEFT_HMM  = find_resource(PKG, 'qtcc_passby_left.hmm')[0]
    QTCBC_PASSBY_LEFT_HMM = find_resource(PKG, 'qtcbc_passby_left.hmm')[0]
    QTCB_QSR              = find_resource(PKG, 'qtcb.qsr')[0]
    QTCC_QSR              = find_resource(PKG, 'qtcc.qsr')[0]
    QTCBC_QSR             = find_resource(PKG, 'qtcbc.qsr')[0]

    correct_samples = {
        "qtcb": [[u'--', u'0-', u'+-', u'+0', u'++']],
        "qtcc": [[u'--+-', u'--0-', u'----', u'0---', u'+---', u'+0--', u'++--', u'---']],
        "qtcbc": [[u'--', u'----', u'0---', u'+---', u'+0--', u'++--', u'++']]
    }

    correct_hashsum = {
        "qtcb": "3fb65b50d0f7631a300132e8bca9ca13",
        "qtcc": "dbf1529cb0b0c90aaebbe7eafe0e9b05",
        "qtcbc": "0a3acf7b48c4c1155931442c86317ce4"
    }

    correct_loglikelihoods ={
        "qtcb": -2.16887,
        "qtcc": -6.07188,
        "qtcbc": -2.23475
    }


    def __init__(self, *args):
        super(TestHMM, self).__init__(*args)

        rospy.init_node(NAME)

        self.r = ROSClient()

#    def _load_qsr_file(self, filename):
#        with open(filename, 'r') as qsr:
#            ret = json.load(qsr)
#
#        return [ret]

#    def _load_file(self):
#        ob = []
#        with open(self.TEST_FILE) as csvfile:
#            reader = csv.DictReader(csvfile)
#            for idx,row in enumerate(reader):
#                ob.append(Object_State(
#                    name=row['agent1'],
#                    timestamp=idx,
#                    x=float(row['x1']),
#                    y=float(row['y1'])
#                ))
#                ob.append(Object_State(
#                    name=row['agent2'],
#                    timestamp=idx,
#                    x=float(row['x2']),
#                    y=float(row['y2'])
#                ))
#
#        self.world.add_object_state_series(ob)

#    def _request(self, msg):
#        cln = QSRlib_ROS_Client()
#        req = cln.make_ros_request_message(msg)
#        res = cln.request_qsrs(req)
#        out = pickle.loads(res.data)
#        foo = []
#        for t in out.qsrs.get_sorted_timestamps():
#            for k, v in zip(out.qsrs.trace[t].qsrs.keys(), out.qsrs.trace[t].qsrs.values()):
#                foo.append(v.qsr)
#
#        return foo

    def _create_hmm(self, qsr_file, qsr_type):
        with open(qsr_file, 'r') as f: qsr_seq = json.load(f)
        _, d = self.r.call_service(
            HMMRepRequestCreate(
                qsr_seq=qsr_seq,
                qsr_type=qsr_type
            )
        )
        return d

    def _create_sample(self, hmm_file, qsr_type):
        with open(hmm_file, 'r') as f: hmm = f.read()
        _, s = self.r.call_service(
            HMMRepRequestSample(
                qsr_type=qsr_type,
                xml=hmm,
                max_length=10,
                num_samples=1
            )
        )
        return s

    def _calculate_loglikelihood(self, hmm_file, qsr_file, qsr_type):
        with open(qsr_file, 'r') as f: qsr_seq = json.load(f)
        with open(hmm_file, 'r') as f: hmm = f.read()
        q, l = self.r.call_service(
            HMMRepRequestLogLikelihood(
                qsr_type=qsr_type,
                xml=hmm,
                qsr_seq=qsr_seq
            )
        )
        return round(l, 5)

    def _to_strings(self, array):
        return [x.values()[0] for x in array]

    def test_qtcb_create(self):
        res = self._create_hmm(self.QTCB_QSR, 'qtcb')
        self.assertEqual(hashlib.md5(res).hexdigest(), self.correct_hashsum["qtcb"])
#        print hashlib.md5(res).hexdigest()

    def test_qtcb_sample(self):
        res = self._create_sample(self.QTCB_SAMPLE_TEST_HMM, 'qtcb')
        self.assertEqual(res, self.correct_samples["qtcb"])
#        print res

    def test_qtcb_loglikelihood(self):
        res = self._calculate_loglikelihood(self.QTCB_PASSBY_LEFT_HMM, self.QTCB_QSR, 'qtcb')
        self.assertEqual(res, self.correct_loglikelihoods["qtcb"])
#        print res

    def test_qtcc_create(self):
        res = self._create_hmm(self.QTCC_QSR, 'qtcc')
        self.assertEqual(hashlib.md5(res).hexdigest(), self.correct_hashsum["qtcc"])
#        print hashlib.md5(res).hexdigest()

    def test_qtcc_sample(self):
        res = self._create_sample(self.QTCC_SAMPLE_TEST_HMM, 'qtcc')
        self.assertEqual(res, self.correct_samples["qtcc"])
#        print res

    def test_qtcc_loglikelihood(self):
        res = self._calculate_loglikelihood(self.QTCC_PASSBY_LEFT_HMM, self.QTCC_QSR, 'qtcc')
        self.assertEqual(res, self.correct_loglikelihoods["qtcc"])
#        print res

    def test_qtcbc_create(self):
        res = self._create_hmm(self.QTCBC_QSR, 'qtcbc')
        self.assertEqual(hashlib.md5(res).hexdigest(), self.correct_hashsum["qtcbc"])
#        print hashlib.md5(res).hexdigest()

    def test_qtcbc_sample(self):
        res = self._create_sample(self.QTCBC_SAMPLE_TEST_HMM, 'qtcbc')
        self.assertEqual(res, self.correct_samples["qtcbc"])
#        print res

    def test_qtcbc_loglikelihood(self):
        res = self._calculate_loglikelihood(self.QTCBC_PASSBY_LEFT_HMM, self.QTCBC_QSR, 'qtcbc')
        self.assertEqual(res, self.correct_loglikelihoods["qtcbc"])
#        print res

#    def test_qtcb_future(self):
#        res = self._create_qsr("qtcb", future=True)
#        self.assertEqual(res, self.correct["qtcb"])
#
#    def test_qtcc(self):
#        res = self._create_qsr("qtcc")
#        self.assertEqual(res, self._to_strings(self.correct["qtcc"]))
#
#    def test_qtcc_future(self):
#        res = self._create_qsr("qtcc", future=True)
#        self.assertEqual(res, self.correct["qtcc"])
#
#    def test_qtcbc(self):
#        res = self._create_qsr("qtcbc")
#        self.assertEqual(res, self._to_strings(self.correct["qtcbc"]))
#
#    def test_qtcbc_future(self):
#        res = self._create_qsr("qtcbc", future=True)
#        self.assertEqual(res, self.correct["qtcbc"])



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestHMM, sys.argv)
#    a = TestHMM([])
#    a.test_qtcb_create()