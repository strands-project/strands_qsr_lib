#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
from qsrrep_lib.rep_lib import ProbRepLib
from qsrrep_lib.rep_io import ServiceManager
from qsr_prob_rep.srv import QSRProbRepResponse, QSRProbRep


class HMMRepROSServer(object):
    """This class provides a service for all requests specified in
    'qsrrep_lib.rep_io.available_services'. The service name will be the key
    of the entry."""

    def __init__(self, name):
        """
        :param name: The name of the node
        """
        rospy.logdebug( "[" + rospy.get_name() + "]: " + "Starting " )
        self._lib = ProbRepLib()
        self.services = {}
        for namespace, services in ServiceManager.available_services.items():
            # Automatically creating a service for all the entries in 'qsrrep_lib.rep_io.available_services'
            # Passing the key of the dict entry to the service to identify the right function to call
            for i, service in enumerate(services):
                # The outer lambda function creates a new scope for the inner lambda function
                # This is necessary to preserve the value of k, otherwise it will have the same value for all services
                # x will be substituded by the service request
                rospy.logdebug( "[" + rospy.get_name() + "]: " + "Creating service: ["+namespace+"]["+service+"]" )
                self.services[service] = rospy.Service("~"+namespace+"/"+service, QSRProbRep, (lambda a,b: lambda x: self.callback(x, a, b))(namespace,service))
                rospy.logdebug( "[" + rospy.get_name() + "]: " + "Created" )
        rospy.logdebug( "[" + rospy.get_name() + "]: " + "Done" )

    def callback(self, req, rep, srv_type):
        r = ServiceManager.available_services[rep][srv_type][0](**json.loads(req.data))
        rr = self._lib.request(r)
        dat = rr.get()
        ans = QSRProbRepResponse(data=dat)
        return ans



if __name__ == '__main__':
    rospy.init_node("prob_rep_ros_server")
    h = HMMRepROSServer(rospy.get_name())
    rospy.spin()
