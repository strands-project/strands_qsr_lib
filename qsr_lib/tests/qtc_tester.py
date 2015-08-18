#!/usr/bin/env python
PKG = 'qsr_lib'
NAME = 'qtc_test'

import rospy
import sys
import unittest
from roslib.packages import find_resource
import csv
try:
    import cPickle as pickle
except:
    import pickle
from qsrlib.qsrlib import QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client
from copy import deepcopy
import json


class TestQTC(unittest.TestCase):
    TEST_FILE = find_resource(PKG, 'qtc.csv')[0]
    dynamic_args = {"qtcs": {
        "quantisation_factor": 0.01,
        "validate": True,
        "no_collapse": False,
        "distance_threshold": 1.2
    }}

    correct = {
        "qtcbs": [{'qtcbs': '-,-'}, {'qtcbs': '-,0'}, {'qtcbs': '0,0'}, {'qtcbs': '0,+'}, {'qtcbs': '+,+'}],
        "qtcbs_nocollapse": lambda: json.load(open(find_resource(PKG, 'qtcbs_nocollapse.txt')[0],'r')),
        "qtccs": [{'qtccs': '-,-,0,0'}, {'qtccs': '-,-,+,0'}, {'qtccs': '-,0,+,0'}, {'qtccs': '-,0,+,+'}, {'qtccs': '0,0,+,+'}, {'qtccs': '0,0,+,0'}, {'qtccs': '0,+,+,0'}, {'qtccs': '+,+,+,0'}, {'qtccs': '+,+,0,0'}],
        "qtccs_nocollapse": lambda: json.load(open(find_resource(PKG, 'qtccs_nocollapse.txt')[0],'r')),
        "qtcbcs": [{'qtcbcs': '-,-'}, {'qtcbcs': '-,0'}, {'qtcbcs': '-,0,+,0'}, {'qtcbcs': '-,0,+,+'}, {'qtcbcs': '0,0,+,+'}, {'qtcbcs': '0,0,+,0'}, {'qtcbcs': '0,+,+,0'}, {'qtcbcs': '+,+,+,0'}, {'qtcbcs': '+,+'}],
        "qtcbcs_nocollapse": lambda: json.load(open(find_resource(PKG, 'qtcbcs_nocollapse.txt')[0],'r')),
    }


    def __init__(self, *args):
        super(TestQTC, self).__init__(*args)

        rospy.init_node(NAME)

        self.world = World_Trace()
        self._load_file()

    def _load_file(self):
        ob = []
        with open(self.TEST_FILE) as csvfile:
            reader = csv.DictReader(csvfile)
            for idx,row in enumerate(reader):
                ob.append(Object_State(
                    name=row['agent1'],
                    timestamp=idx+1,
                    x=float(row['x1']),
                    y=float(row['y1'])
                ))
                ob.append(Object_State(
                    name=row['agent2'],
                    timestamp=idx+1,
                    x=float(row['x2']),
                    y=float(row['y2'])
                ))

        self.world.add_object_state_series(ob)

    def _request(self, msg):
        cln = QSRlib_ROS_Client()
        req = cln.make_ros_request_message(msg)
        res = cln.request_qsrs(req)
        out = pickle.loads(res.data)
        foo = []
        for t in out.qsrs.get_sorted_timestamps():
            for k, v in zip(out.qsrs.trace[t].qsrs.keys(), out.qsrs.trace[t].qsrs.values()):
                foo.append(v.qsr)

        return foo

    def _create_qsr(self, qsr, future=False, dynamic_args=None):
        dynamic_args = self.dynamic_args if not dynamic_args else dynamic_args
        print dynamic_args
        qsrlib_request_message = QSRlib_Request_Message(
            which_qsr=qsr,
            input_data=self.world,
            include_missing_data=True,
            dynamic_args=dynamic_args,
            future=future,
            qsrs_for=[("human", "robot")]
        )
        return self._request(qsrlib_request_message)

    def _to_strings(self, array):
        return [x.values()[0] for x in array]

    def test_qtcb(self):
        res = self._create_qsr("qtcbs")
        self.assertEqual(res, self._to_strings(self.correct["qtcbs"]))

    def test_qtcb_future(self):
        res = self._create_qsr("qtcbs", future=True)
        self.assertEqual(res, self.correct["qtcbs"])

    def test_qtcb_nocollapse(self):
        dynamic_args = deepcopy(self.dynamic_args)
        dynamic_args["qtcs"]["validate"] = False
        dynamic_args["qtcs"]["no_collapse"] = True
        res = self._create_qsr("qtcbs", dynamic_args=dynamic_args)
        self.assertEqual(res, self.correct["qtcbs_nocollapse"]())

    def test_qtcc(self):
        res = self._create_qsr("qtccs")
        self.assertEqual(res, self._to_strings(self.correct["qtccs"]))

    def test_qtcc_future(self):
        res = self._create_qsr("qtccs", future=True)
        self.assertEqual(res, self.correct["qtccs"])

    def test_qtcc_nocollapse(self):
        dynamic_args = deepcopy(self.dynamic_args)
        dynamic_args["qtcs"]["validate"] = False
        dynamic_args["qtcs"]["no_collapse"] = True
        res = self._create_qsr("qtccs", dynamic_args=dynamic_args)
        self.assertEqual(res, self.correct["qtccs_nocollapse"]())

    def test_qtcbc(self):
        res = self._create_qsr("qtcbcs")
        self.assertEqual(res, self._to_strings(self.correct["qtcbcs"]))

    def test_qtcbc_future(self):
        res = self._create_qsr("qtcbcs", future=True)
        self.assertEqual(res, self.correct["qtcbcs"])

    def test_qtcbc_nocollapse(self):
        dynamic_args = deepcopy(self.dynamic_args)
        dynamic_args["qtcs"]["validate"] = False
        dynamic_args["qtcs"]["no_collapse"] = True
        res = self._create_qsr("qtcbcs", dynamic_args=dynamic_args)
        self.assertEqual(res, self.correct["qtcbcs_nocollapse"]())



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestQTC, sys.argv)
