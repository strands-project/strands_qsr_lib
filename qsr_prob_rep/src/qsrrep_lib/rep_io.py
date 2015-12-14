# -*- coding: utf-8 -*-

"""This file defines classes for the interaction with the HMMs. To create your
own please follow these steps:

TODO: Update description

    * Create a request class following the naming scheme: HMMRepRequestFunctionality
    where 'Functionality' is replaced by your new functionality name.
        * Inherit from 'HMMRepRequestAbstractclass'
        * Define the '_const_function_pointer' to use your function in qsrrep_lib.py.
            * Make sure you implemented such a function in there
            * Make th pointer look like the one in 'HMMRepRequestAbstractclass'
            and replace 'my_function' with the function name in qsrrep_lib.py
        * Override '__init__' defiinig a custom finction header and adding the
        variables to the variable 'self.kwargs'
    * Create a response class following the naming scheme 'HMMReqResponseFunctionality'
    where 'Funtionality' should be the same as for the request class.
        * Override the 'get' function to make sure it returns a string (str or
        json dump)
    * Add you new functionality to 'available_services' in the bottom of the file.
        * The string key will be used to create the service name
        * The value should be a list where the first entry is your request class
        and the second the response class.
"""

from abc import ABCMeta, abstractmethod


class RepRequestAbstractclass(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self):
        self.rep_type = ""
        self.kwargs = {}

    def call_function(self, c, f):
        return getattr(c, f)(**self.kwargs)


class ReqResponseAbstractclass(object):
    __metaclass__ = ABCMeta

    qsr_type = ""

    @abstractmethod
    def get(self):
        return


class ServiceManager():

    available_services = {}

    def get_available_services(self):
        __import__("qsrrep_lib.rep_lib") # This calls all the decorators. Necessary in ros_client.py to generate dictionary.
        return self.available_services

    class service_function(object):
        def __init__(self, namespace, request_class, response_class):
            self.namespace = namespace
            self.request_class = request_class
            self.response_class = response_class

        def __call__(self, func):
            if self.namespace not in ServiceManager.available_services:
                ServiceManager.available_services[self.namespace] = {}
            ServiceManager.available_services[self.namespace][func.__name__] = (self.request_class, self.response_class)
            def wrapper(self, *args, **kwargs):
                return func(self, *args, **kwargs)
            return wrapper
