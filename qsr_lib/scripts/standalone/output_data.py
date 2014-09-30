# -*- coding: utf-8 -*-
"""Provides for output data

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 10 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""
from __future__ import print_function, division
from datetime import datetime


class Output_Data(object):
    def __init__(self, ids=[], data=[], qsr_type="", all_possible_relations=[],
                 timestamp_request_received=None, timestamp_qsrs_processed=None):
        """

        :param ids: list of ids identifying the data rows
        :param data: list of qsrs, number of rows should be equal len(ids)
        :param type: qsr type
        :param timestamp: when processed
        """
        self.ids = ids
        self.data = data
        self.qsr_type = qsr_type
        self.all_possible_relations = all_possible_relations
        self.timestamp_request_received = timestamp_request_received if timestamp_request_received else datetime.now()
        self.timestamp_qsrs_processed = timestamp_qsrs_processed if timestamp_qsrs_processed else datetime.now()

    def check(self):
        foo = len(self.ids) == len(self.data)
        if foo:
            print("Warning (Output_Data_One.check): length of data does not match the number of given ids")
        return foo
