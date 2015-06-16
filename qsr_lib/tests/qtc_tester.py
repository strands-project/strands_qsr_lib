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


class TestQTC(unittest.TestCase):
    TEST_FILE = find_resource(PKG, 'qtc.csv')[0]
    options = {"qtcb": "qtc_b_simplified",
               "qtcc": "qtc_c_simplified",
               "qtcbc": "qtc_bc_simplified"}
    dynamic_args = {
        "quantisation_factor": 0.01,
        "validate": True,
        "no_collapse": False,
        "distance_threshold": 1.2
    }
    future = True

    correct = {
        "qtcb": [{'qtcbs': '-,-'}, {'qtcbs': '-,0'}, {'qtcbs': '0,0'}, {'qtcbs': '0,+'}, {'qtcbs': '+,+'}],
        "qtcc": [{'qtccs': '-,-,0,0'}, {'qtccs': '-,-,+,0'}, {'qtccs': '-,0,+,0'}, {'qtccs': '-,0,+,+'}, {'qtccs': '0,0,+,+'}, {'qtccs': '0,0,+,0'}, {'qtccs': '0,+,+,0'}, {'qtccs': '+,+,+,0'}, {'qtccs': '+,+,0,0'}]
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
                    timestamp=idx,
                    x=float(row['x1']),
                    y=float(row['y1'])
                ))
                ob.append(Object_State(
                    name=row['agent2'],
                    timestamp=idx,
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

    def _create_qsr(self, qsr):
        qsrlib_request_message = QSRlib_Request_Message(
            which_qsr=self.options[qsr],
            input_data=self.world,
            include_missing_data=True,
            dynamic_args=self.dynamic_args,
            future=self.future,
            qsrs_for=[("human", "robot")]
        )
        return self._request(qsrlib_request_message)

    def test_qtcb(self):
        res = self._create_qsr("qtcb")
        self.assertEqual(res, self.correct["qtcb"])

    def test_qtcc(self):
        res = self._create_qsr("qtcc")
        self.assertEqual(res, self.correct["qtcc"])



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestQTC, sys.argv)