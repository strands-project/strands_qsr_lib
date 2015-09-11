For QSR developers
==================

In order to extend QSRlib with a new QSR the following two steps are
needed: \* Implement a new QSR \* Register the new QSR with QSRlib

Howto: Implement a new QSR
--------------------------

Find below a minimally working example:

.. code:: python

    # -*- coding: utf-8 -*-
    from __future__ import print_function, division
    from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_1t_Abstractclass


    class QSR_MWE(QSR_Dyadic_1t_Abstractclass):
        def __init__(self):
            super(QSR_MWE, self).__init__()
            self._unique_id = "mwe"
            self.all_possible_relations = ["left", "together", "right"]
            self._dtype = "points"

        def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
            return {
                data1.x < data2.x: "left",
                data1.x > data2.x: "right"
            }.get(True, "together")

Line by line explanation.

.. code:: python

    class QSR_MWE(QSR_Dyadic_1t_Abstractclass):

Our class inherits from one of the special case abstract classes (more
to this later).

.. code:: python

        def __init__(self):
            super(QSR_MWE, self).__init__()
            self._unique_id = "mwe"
            self.all_possible_relations = ["left", "together", "right"]
            self._dtype = "points"

In the constructor we need to define our QSR members. In this MWE the
minimum parameters that need to be defined are: \* `self._unique_id`:
This is the name of the QSR. You can call it what you want but it must
be unique among all QSRs and preferably as short as possible with
camelCase names preferred. \* `self.all_possible_relations`: A list
(or tuple) of all the possible values that the QSR can take. It can be
anything you like. \* `self._dtype`: With what type of data your
`_compute_qsr` methods works with. Are they points or bounding boxes
for example. For what you can use look in the parent class member
`self._allowed_dtypes`.

.. code:: python

        def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
            return {
                data1.x < data2.x: "left",
                data1.x > data2.x: "right"
            }.get(True, "together")

The only method you need to implement that computes the QSR you are
implementing.

*IMPORTANT NOTE:* There are different types of parent classes that you
can inherit from. You can see them in the
`qsr_monadic_abstractclass.py` and `qsr_dyadic_abstractclass.py`
files. If one of the "special case" classes like in this example the
class `QSR_Dyadic_1t_Abstractclass` does not suit you then you can
inherit from `QSR_Monadic_Abtractclass` or
`QSR_Dyadic_Abstracclass`. In this case you will also have to provide
your own `make_world_qsr_trace` (see the special cases for some
example ideas) but you are not required to specify anymore the member
`self._dtype` and also not required to implement a `_compute_qsr`
method (unless you want to for better code practice in which we
recommend that you use private scope, i.e. name it as
`__compute_qsr`).

Lastly, if none of the monadic and dyadic family classes allow you to
implement your QSR (e.g. you want a triadic QSR) then feel free to
extend it in a similar manner or file an issue in github and we will try
to implement the support infrastructure the quickest possible.

Howto: Register the new QSR with QSRlib
---------------------------------------

Add to `strands_qsr_lib/qsr_lib/src/qsrlib_qsrs/__init__.py` the
following:

Import your class name in the imports (before the `qsrs_registry`
line). E.g. for above QSR add the following line:

.. code:: python

    from qsr_new_mwe import QSR_MWE

Add the new QSR class name in `qsrs_registry`. E.g. for above QSR:

.. code:: python

    qsrs_registry = (<some other QSR class names>,
                     QSR_MWE)
