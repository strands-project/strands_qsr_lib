# -*- coding: utf-8 -*-
"""Provides for input data

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 10 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""
from __future__ import print_function, division

class Input_Data_One(object):
    def __init__(self, id, data):
        self.id = id
        self.data = data

class Input_Data_Block(object):
    def __init__(self, data=[], fields=[], timesteps=1, description=""):
        """Holds a block of data in a list of Input_Data_One

        :param data:
        :param description: optional human readable description
        :param fields: a dictionary of indexes, its length should be equal to the step, passed manually, could be automated according to datatype
        """
        self.description = description
        self.fields = fields
        self.timesteps = timesteps
        self.data = self.process_input(data)

    def process_input(self, data):
        """Will be decided once the raw format is known..."""
        ret = []
        if len(data) > 0:
            ret = data
        else:
            print("Warning (Input_Data_Block.process_input): argument data is empty")
        return ret

    def convert_fields_to_dict(self):
        d = {}
        for i, v in zip(range(len(self.fields)), self.fields):
            d[v] = i
        return d


if __name__ == "__main__":
    input_data = Input_Data_Block(data=[Input_Data_One("1", [1.0, 1.0, 2.0, 2.0]),
                                        Input_Data_One("2", [11.0, 11.0, 22.0, 22.0])],
                                  fields=["x", "y"],
                                  description="some optional description")
