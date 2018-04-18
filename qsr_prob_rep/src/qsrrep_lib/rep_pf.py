# -*- coding: utf-8 -*-

#from rep_abstractclass import RepAbstractclass
from rep_io import ServiceManager
from rep_io_pf import PfRepRequestCreate, PfRepRequestUpdate, PfRepRequestPredict, PfRepRequestRemove, PfRepRequestList
from rep_io_pf import PfReqResponseCreate, PfReqResponseUpdate, PfReqResponsePredict, PfReqResponseRemove, PfReqResponseList
from qsrrep_pf.filter import ParticleFilterPredictor
import uuid
import json
import numpy as np


class RepPf(object):

    __filter_bank = {}

    namespace = "pf"

    def __init__(self):
        self.pf = ParticleFilterPredictor()
        pass

    @ServiceManager.service_function(namespace, PfRepRequestCreate, PfReqResponseCreate)
    def create(self, **kwargs):
        """
        TODO: Create description
        """
        if kwargs["uuid"] == None:
            kwargs["uuid"] = uuid.uuid1().hex

        self.__filter_bank[kwargs["uuid"]] = self.pf.create(**kwargs)

        if kwargs["debug"]:
            p = self.__filter_bank[kwargs["uuid"]]["filter"].emp.particles
            print "[RepPf] INIT MODEL SIZES:", np.bincount(map(int,p[:,1].flatten()), minlength=2)
            print "[RepPf] INIT STATES:", np.bincount(map(int,p[:,0].flatten())), len(np.bincount(map(int,p[:,0].flatten())))
            for i in range(int(np.max(p[:,1]))+1):
                print "[RepPf] INIT STATES model %i:"%i, np.bincount(map(int,p[np.where(p[:,1] == float(i)),0].flatten())), len(np.bincount(map(int,p[np.where(p[:,1] == float(i)),0].flatten())))

        return PfReqResponseCreate(data=kwargs["uuid"])

    @ServiceManager.service_function(namespace, PfRepRequestUpdate, PfReqResponseUpdate)
    def update(self, **kwargs):
        """
        TODO: Create description
        """

        z = kwargs.copy()
        z.update(self.__filter_bank[kwargs["uuid"]])
        res = self.pf.update(**z)
        datum =  json.dumps(res)
        return PfReqResponseUpdate(data=datum)

    @ServiceManager.service_function(namespace, PfRepRequestPredict, PfReqResponsePredict)
    def predict(self, **kwargs):
        """
        TODO: Create description
        """

        z = kwargs.copy()
        z.update(self.__filter_bank[kwargs["uuid"]])

        res = self.pf.predict(**z)

        return PfReqResponsePredict(data=json.dumps(res))

    @ServiceManager.service_function(namespace, PfRepRequestRemove, PfReqResponseRemove)
    def remove(self, **kwargs):
        """
        TODO: Create description
        """

        res = 0
        try:
            del self.__filter_bank[kwargs["uuid"]]
        except KeyError:
            pass

        if kwargs["uuid"] not in self.__filter_bank:
            res = 1

        return PfReqResponseRemove(data=json.dumps(res))

    @ServiceManager.service_function(namespace, PfRepRequestList, PfReqResponseList)
    def list_filters(self, **kwargs):
        res = []
        for k, v in self.__filter_bank.items():
            res.append((k,v["models"]))
        return PfReqResponseList(data=json.dumps(res))

