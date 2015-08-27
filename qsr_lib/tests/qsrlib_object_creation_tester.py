#!/usr/bin/env python
from __future__ import print_function, division
import unittest
from qsrlib.qsrlib import QSRlib


class QSRlib_Object_Creation_Test(unittest.TestCase):
    def test(self):
        QSRlib(help=False)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("qsr_lib", "qsrlib_object_creation_test", QSRlib_Object_Creation_Test)
