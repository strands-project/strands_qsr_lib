#!/usr/bin/env python
# -*- coding: utf-8 -*-

from hmmrep_ros.ros_client import ROSClient
from hmmrep_lib.hmmrep_io import HMMRepRequestCreate, HMMRepRequestSample, HMMRepRequestLogLikelihood
import os
import rospy
import json

def load_files(path):
    ret = []
    for f in os.listdir(path):
        if f.endswith(".qtc"):
            filename = path + '/' + f
            with open(filename, 'r') as qtc:
                ret.append(json.load(qtc))

    return ret


if __name__ == "__main__":
    rospy.init_node("ros_client")
    r = ROSClient()
    qtc = load_files("/home/cdondrup/Desktop/")
    data={"qsr_seq": qtc}
    q,d = r.call_service(HMMRepRequestCreate(qsr_seq=qtc, qsr_type='qtcc'))
    _, s = r.call_service(HMMRepRequestSample(qsr_type=q, xml=d, max_length=10, num_samples=5))
#    print d
    print s
    _, l = r.call_service(HMMRepRequestLogLikelihood(qsr_type=q, xml=d, qsr_seq=s))
    print l
