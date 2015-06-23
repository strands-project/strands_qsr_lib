# -*- coding: utf-8 -*-

import json

class HMMReqResponseBaseclass(object):
    def __init__(self, qsr_type, data):
        self.qsr_type = qsr_type
        self.data = data

    def get_type(self):
        return self.qsr_type

    def get_data(self):
        return self.data

class HMMReqResponseCreate(HMMReqResponseBaseclass):

    def get_xml(self):
        return str(self.get_data())

class HMMReqResponseSample(HMMReqResponseBaseclass):

    def get_sample(self):
        return json.loads(self.get_data())

class HMMReqResponseLogLikelihood(HMMReqResponseBaseclass):

    def get_log_likelihood(self):
        return json.loads(self.get_data())