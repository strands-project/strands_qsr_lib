#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import argparse
import os, sys
import numpy as np
import cPickle as pickle
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace
import qsrlib_qstag.utils as utils


def pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message):
	print(which_qsr, "request was made at ", str(qsrlib_response_message.req_made_at)
		  + " and received at " + str(qsrlib_response_message.req_received_at)
		  + " and finished at " + str(qsrlib_response_message.req_finished_at))
	print("---")
	print("timestamps:", qsrlib_response_message.qsrs.get_sorted_timestamps())
	print("Response is:")
	for t in qsrlib_response_message.qsrs.get_sorted_timestamps():
		foo = str(t) + ": "
		for k, v in zip(qsrlib_response_message.qsrs.trace[t].qsrs.keys(),
						qsrlib_response_message.qsrs.trace[t].qsrs.values()):
			foo += str(k) + ":" + str(v.qsr) + "; "
		print(foo)



if __name__ == "__main__":

	options = sorted(QSRlib().qsrs_registry.keys()) + ["multiple"]
	multiple = options[:]; multiple.remove("multiple"); multiple.remove("argd"); multiple.remove("argprobd")
	multiple.remove("ra"); multiple.remove("mwe");

	parser = argparse.ArgumentParser()
	parser.add_argument("qsr", help="choose qsr: %s" % options, type=str, default='qtcbs')
	parser.add_argument("--ros", action="store_true", default=False, help="Use ROS eco-system")

	parser.add_argument("--validate", help="validate state chain. Only QTC", action="store_true", default=False)
	parser.add_argument("--quantisation_factor", help="quantisation factor for 0-states in qtc, or 's'-states in mos", type=float, default=0.01)
	parser.add_argument("--no_collapse", help="does not collapse similar adjacent states. Only QTC", action="store_true", default=True)
	#parser.add_argument("--distance_threshold", help="distance threshold for qtcb <-> qtcc transition. Only QTCBC", type=float)

	args = parser.parse_args()
	args.distance_threshold = {"touch":1, "near":3, "medium":5, "far":10}

	qtcbs_qsrs_for = [("o1", "o2"), ("o1", "o3"), ("o2", "o3")]
	argd_qsrs_for = [("o1", "o2")]
	mos_qsrs_for = ["o1", "o2"]
	tpcc_qsrs_for = [("o1", "o2", "o3")]

	object_types = {"o1": "Human",
					"o2": "Chair",
					"o1-o2" : "tpcc-plane"}

	if args.qsr in options:
		if args.qsr != "multiple":
			which_qsr = args.qsr
		else:
			which_qsr = multiple
	elif args.qsr == "hardcoded":
		which_qsr = ["qtcbs", "argd", "mos"]
	else:
		raise ValueError("qsr not found, keywords: %s" % options)

	world = World_Trace()

	dynamic_args = {"qtcbs": {"quantisation_factor": args.quantisation_factor,
							  "validate": args.validate,
							  "no_collapse": args.no_collapse,
							  "qsrs_for": qtcbs_qsrs_for},

					"argd": {"qsr_relations_and_values": args.distance_threshold,
							  "qsrs_for": argd_qsrs_for},

					"mos": {"qsrs_for": mos_qsrs_for},

                    "qstag": {"object_types" : object_types,
                              "params" : {"min_rows" : 1, "max_rows" : 1, "max_eps" : 3}},

					"tpcc" : {"qsrs_for": tpcc_qsrs_for}

                    #"filters": {"median_filter" : {"window": 2}}
					}

	o1 = [Object_State(name="o1", timestamp=0, x=2., y=2., object_type="Person"),  #accessed first using try: kwargs["object_type"] except:
		  Object_State(name="o1", timestamp=1, x=1., y=1., object_type="Person"),
		  Object_State(name="o1", timestamp=2, x=4., y=1., object_type="Human"),
		  Object_State(name="o1", timestamp=3, x=4., y=1., object_type="Human"),
		  Object_State(name="o1", timestamp=4, x=4., y=2., object_type="Human"),
		  Object_State(name="o1", timestamp=5, x=4., y=3., object_type="Human")]

	o2 = [Object_State(name="o2", timestamp=0, x=1., y=1., object_type="Chair"),
		  Object_State(name="o2", timestamp=1, x=2., y=2., object_type="Chair"),
		  Object_State(name="o2", timestamp=2, x=2., y=5., object_type="Chair"),
		  Object_State(name="o2", timestamp=3, x=2., y=5., object_type="Chair"),
		  Object_State(name="o2", timestamp=4, x=2., y=5., object_type="Chair"),
		  Object_State(name="o2", timestamp=5, x=2., y=5., object_type="Chair")]

	o3 = [Object_State(name="o3", timestamp=0, x=0., y=0., object_type="Desk"),
		  Object_State(name="o3", timestamp=1, x=0., y=0., object_type="Desk"),
		  Object_State(name="o3", timestamp=2, x=0., y=0., object_type="Desk"),
		  Object_State(name="o3", timestamp=3, x=0., y=0., object_type="Desk"),
		  Object_State(name="o3", timestamp=4, x=0., y=0., object_type="Desk"),
		  Object_State(name="o3", timestamp=5, x=0., y=0., object_type="Desk")]


	world.add_object_state_series(o1)
	world.add_object_state_series(o2)
	world.add_object_state_series(o3)
	qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world, dynamic_args=dynamic_args)

	print(which_qsr)
	print(dynamic_args["tpcc"])

	if args.ros:
		try:
			import rospy
			from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client
		except ImportError:
			raise ImportError("ROS not found")
		client_node = rospy.init_node("qsr_lib_ros_client_example")
		cln = QSRlib_ROS_Client()
		req = cln.make_ros_request_message(qsrlib_request_message)
		res = cln.request_qsrs(req)
		qsrlib_response_message = pickle.loads(res.data)
	else:
		qsrlib = QSRlib()
		qsrlib_response_message = qsrlib.request_qsrs(req_msg=qsrlib_request_message)

	pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message)

	qstag = qsrlib_response_message.qstag

	"""PRINT THE GRAPH TO FILE"""
	#print("QSTAG Graph:\n", qstag.graph)
	utils.graph2dot(qstag, "/tmp/graph.dot")
	os.system('dot -Tpng /tmp/graph.dot -o /tmp/graph.png')

	print("Episodes:")
	for i in qstag.episodes:
		print(i)

	print("\n,Graphlets:")
	for i, j in qstag.graphlets.graphlets.items():
		print("\n", j)

	print("\nHistogram of Graphlets:")
	print(qstag.graphlets.histogram)
