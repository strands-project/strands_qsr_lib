#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
from hmmrep_lib.hmmrep_lib import HMMRepLib
# Wildard import necessary because I don't know which services exist, they are defined in 'hmmrep_lib.hmmrep_io.available_services'
from hmmrep_lib.hmmrep_io import *
from qsr_hmm_rep.srv import QSRHMMRepResponse, QSRHMMRep


class HMMRepROSServer(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s" % name)
        self._hmm_lib = HMMRepLib()
        self._hmm_lib.print_hmms_available()
        self.services = {}
        # Automatically creating a service for all the entries in 'hmmrep_lib.hmmrep_io.available_services'
        # Passing the key of the dict entry to the service to identify the right function to call
        for i, k in enumerate(available_services.keys()):
            # The outer lambda function creates a new scope for the inner lambda function
            # This is necessary to preserve the value of k, otherwise it will have the same value for all services
            # x will be substituded by the service request
            self.services[k] = rospy.Service("~"+k, QSRHMMRep, (lambda b: lambda x: self.callback(x, b))(k))

    def callback(self, req, srv_type):
        print "SERVICCE", type(req.data)
        r = available_services[srv_type][0](qsr_type=req.qsr_type, **json.loads(req.data))
        return QSRHMMRepResponse(qsr_type=req.qsr_type, data=self._hmm_lib.request(r).get())


if __name__ == '__main__':
    rospy.init_node("hmm_rep_ros_server")
    h = HMMRepROSServer(rospy.get_name())
    rospy.spin()
