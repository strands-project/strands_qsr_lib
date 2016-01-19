# -*- coding: utf-8 -*-

import rospy
import json
from qsrrep_lib.rep_io import RepRequestAbstractclass, ServiceManager
from qsr_prob_rep.srv import QSRProbRep, QSRProbRepRequest

class ROSClient(object):
    """This class provides a convenience method to encapsulate the ROS service
    architecture."""

    def __init__(self, server_name="prob_rep_ros_server"):
        """
        :param server_name: The name of the hmm_rep_ros server. Default: 'prob_rep_ros_server'
        """
        self.services = {} # Creating a dictonary containing all the service names and classes
        for namespace, services in ServiceManager().get_available_services().items():
            for k, v in services.items():
                self.services[v[0]] = server_name + "/" + namespace + "/" + k

    def call_service(self, req):
        """Calling the appropriate service depending on the given request type.
        Requests have to inherit from 'HMMRepRequestAbstractclass'.

        :param req: The request class instance for the request you want to make

        :return: The qsr_type and resulting data tuple. The data is in the data type produced by the response.
        """
        assert(issubclass(req.__class__, RepRequestAbstractclass))
        s = rospy.ServiceProxy(self.services[req.__class__],QSRProbRep)
        try:
            s.wait_for_service(timeout=10.)
            res = s(QSRProbRepRequest(data=json.dumps(req.kwargs)))
        except (rospy.ROSException, rospy.ROSInterruptException, rospy.ServiceException) as e:
            rospy.logerr(e)
            return None
        try:
            data = json.loads(res.data)
        except ValueError:
            data = str(res.data)
        return data
