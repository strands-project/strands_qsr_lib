#!/usr/bin/env python
from __future__ import print_function, division
import unittest
from qsrlib.qsrlib import QSRlib

class QSRlib_Unique_Developed_QSRs(unittest.TestCase):
    def test(self):
        QSRlib(help=False)

if __name__ == '__main__':
    unittest.main()
