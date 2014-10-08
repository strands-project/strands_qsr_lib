#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Usage example.

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
:Date: 22 September 2014
:Version: 0.1
:Status: Development
:Copyright: STRANDS default
"""

from __future__ import print_function, division
from qsrlib.input_data import Input_Data_One, Input_Data_Block
from qsrlib.qsrlib import QSRlib

if __name__ == "__main__":
    # define some dummy sample data
    input_data = Input_Data_Block(data=[Input_Data_One("1", [1.0, 1.0, 2.0, 2.0, 1.0, 1.0, 2.0, 2.0, 1.0, 1.0, 2.0, 2.0]),
                                        Input_Data_One("2", [11.0, 11.0, 22.0, 22.0, 1.5, 1.5, 22.0, 22.0, 1.5, 1.5, 1.5, 1.5])],
                                  fields=["x1", "y1", "x2", "y2"],
                                  timesteps=3,
                                  description="some 2d bounding boxes")

    # make a QSRlib object
    qsrlib = QSRlib()
    # request QSRs
    out = qsrlib.request_qsrs(which_qsr="rcc3_rectangle_bounding_boxes_2d", input_data=input_data)
    # print the timestamps, ids and qsrs
    print("Request was received at", out.timestamp_request_received, "and finished processing at", out.timestamp_qsrs_processed)
    print("Objects:", out.ids)
    print("QSRs:", out.data)

    print()
    print("--- Testing reusing previous data ---")
    out = qsrlib.request_qsrs(which_qsr="rcc3_rectangle_bounding_boxes_2d")
    # print the timestamps, ids and qsrs
    print("Request was received at", out.timestamp_request_received, "and finished processing at", out.timestamp_qsrs_processed)
    print("Objects:", out.ids)
    print("QSRs:", out.data)
