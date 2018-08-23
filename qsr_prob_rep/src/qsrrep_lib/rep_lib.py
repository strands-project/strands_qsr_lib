# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 14:19:25 2015

@author: cdondrup

"""

from rep_io import ServiceManager, RepRequestAbstractclass
from rep_hmm import RepHMM
from rep_pf import RepPf


class ProbRepLib(object):
    """This class provides the main functionalities of the library by calling
    the appropriate functions in the HMM implementations. None of the providing
    functions should be called directly but via the function pointer '_const_function_pointer'
    in rep_io.py in the request classes. The function 'request' in here will
    take care of executing it.

    Adding a new HMM type:
        * Add an entry to 'hmm_types_available', following the examples given and import the correct class

    """

    modules = {
        RepHMM.namespace: RepHMM(),
        RepPf.namespace: RepPf()
    }

    def __init__(self):
        self.isDebug = False

        if self.isDebug:
            print("[ProbRepLib]" + "Currently available services:")
            for k, v in ServiceManager.available_services.items():
                print("[ProbRepLib] Namespace: " +  str(k) )
                for k in v.keys():
                    print("[ProbRepLib] services: " +  str(k) )
        else:
            pass
        
    def request(self, request_message):
        """The magic function that calls the appropiate function beased on
        '_const_function_pointer' in the requestclass definition.

        :param request_message: The request message object which has to be an instance of one of the request classes in rep_io.py and has to inherit from 'HMMRepRequestAbstractclass'

        :return: The resulting respons message that corresponds to the request message.
        """
        assert(issubclass(request_message.__class__, RepRequestAbstractclass))

        func_name = ""
        for k,v in ServiceManager.available_services.items():
            if self.isDebug:
                print("[ProbRepLib] key:    ["+str(k)+"]")
                print("[ProbRepLib] value:  ["+str(v)+"]")
            try:
                func_name = v.keys()[map(lambda x:x[0], v.values()).index(request_message.__class__)]
            except ValueError:
                continue
            namespace = k
            
            if func_name != "":
                break
        
        used_module = self.modules[namespace]

        rmsg = request_message.call_function(used_module, func_name)
        return rmsg
