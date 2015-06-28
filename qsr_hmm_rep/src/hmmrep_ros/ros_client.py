# -*- coding: utf-8 -*-

import rospy
import json
from hmmrep_lib.hmmrep_io import available_services, HMMRepRequestAbstractclass
from qsr_hmm_rep.srv import QSRHMMRep, QSRHMMRepRequest

class ROSClient(object):
    def __init__(self, server_name="hmm_rep_ros_server"):
        self.services = {}
        for k, v in available_services.items():
            self.services[v[0]] = server_name + "/" + k

    def call_service(self, req):
        assert(issubclass(req.__class__, HMMRepRequestAbstractclass))
        s = rospy.ServiceProxy(self.services[req.__class__],QSRHMMRep)
        res = s(QSRHMMRepRequest(qsr_type=req.kwargs.pop("qsr_type"), data=json.dumps(req.kwargs)))
        try:
            data = json.loads(res.data)
        except ValueError:
            data = str(res.data)
        return res.qsr_type, data
