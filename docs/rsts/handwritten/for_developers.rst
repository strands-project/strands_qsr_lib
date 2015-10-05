For QSR developers
==================

This section provides information for developers that wish extend QSRlib with new QSRs.
This process consists of two steps:

1. Implement a new QSR
2. Register the new QSR with QSRlib

Howto: New QSRs
---------------

Implementation
~~~~~~~~~~~~~~

Find below a minimally working example:

.. code:: python

    from __future__ import print_function, division
    from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_1t_Abstractclass

    class QSR_MWE(QSR_Dyadic_1t_Abstractclass):
        _unique_id = "mwe"
        _all_possible_relations = ("left", "together", "right")
        _dtype = "points"

        def __init__(self):
            super(QSR_MWE, self).__init__()

        def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
            return {
                data1.x < data2.x: "left",
                data1.x > data2.x: "right"
            }.get(True, "together")

Line-by-line explanation
^^^^^^^^^^^^^^^^^^^^^^^^

.. code:: python

    class QSR_MWE(QSR_Dyadic_1t_Abstractclass):

Our class inherits from one of the special case abstract classes (more
to this later). For now, we need to define the following abstract properties.

.. code:: python

        _unique_id = "mwe"
        _all_possible_relations = ("left", "together", "right")
        _dtype = "points"


* `_unique_id`: This is the name of the QSR. You can call it what you want but it must be unique among all QSRs and preferably as short as possible with.
* `all_possible_relations`: A list (or tuple) of all the possible values that the QSR can take. It can be anything you like.
* `_dtype`: With what type of data your QSR works with. For example, they might be points, or they might be bounding boxes. For what you can use look in the :class:`QSR abstractclass <qsrlib_qsrs.qsr_abstractclass.QSR_Abstractclass>` `self._dtype_map`.

Then you need to write one function that computes the QSR.

.. code:: python

        def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
            return {
                data1.x < data2.x: "left",
                data1.x > data2.x: "right"
            }.get(True, "together")

.. note::
    There are different types of parent classes that you
    can inherit from. You can see them in the
    :mod:`qsr_monadic_abstractclass.py <qsrlib_qsrs.qsr_monadic_abstractclass>`,
    :mod:`qsr_dyadic_abstractclass.py <qsrlib_qsrs.qsr_dyadic_abstractclass>`, and,
    :mod:`qsr_triadic_abstractclass.py <qsrlib_qsrs.qsr_triadic_abstractclass>`
    module files.

    If one of the "special case" classes like in this example the
    class :class:`QSR_Dyadic_1t_Abstractclass <qsrlib_qsrs.qsr_dyadic_abstractclass.QSR_Dyadic_1t_Abstractclass>`
    does not suit you then you can inherit from one level higher, i.e. from
    :class:`QSR_Dyadic_Abstractclass <qsrlib_qsrs.qsr_dyadic_abstractclass.QSR_Dyadic_Abstractclass>`
    ( or from :class:`QSR_Monadic_Abstractclass <qsrlib_qsrs.qsr_monadic_abstractclass.QSR_Monadic_Abstractclass>`).
    In this case you will also have to provide
    your own
    :meth:`make_world_qsr_trace <qsrlib_qsrs.qsr_abstractclass.QSR_Abstractclass.make_world_qsr_trace>`
    (see the special cases for some example ideas).

    Lastly, if none of the monadic and dyadic family classes allow you to
    implement your QSR (e.g. you want a triadic QSR) then feel free to
    extend it in a similar manner, or file an issue_ and we will consider
    implementing it the quickest possible.

.. _issue: https://github.com/strands-project/strands_qsr_lib/issues

Registration
~~~~~~~~~~~~

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


Advanced Topics
---------------

QSR specific parameters
~~~~~~~~~~~~~~~~~~~~~~~

It is possible to change the behavior of a QSR via passing dynamically during the request call argument parameters in
one of its fields that is called `dynamic_args`. It is recommended to read first the documentation on how it is used in
:ref:`this page <dynamic_args>`.

In order use QSR specific parameters you will have to overwrite the method
``_process_qsr_parameters_from_request_parameters(self, req_params, **kwargs)`` in your QSR implementation.

Below is an example on how to do it from
:class:`MOS <qsrlib_qsrs.qsr_moving_or_stationary.QSR_Moving_or_Stationary>` QSR.

.. code:: python

    def _process_qsr_parameters_from_request_parameters(self, req_params, **kwargs):
        """Extract QSR specific parameters from the QSRlib request call parameters.

        :param req_params: QSRlib request call parameters.
        :type req_params: dict
        :param kwargs: kwargs arguments.
        :return: QSR specific parameter settings.
        :rtype: dict
        """
        qsr_params = self.__qsr_params_defaults.copy()
        try:
            qsr_params["quantisation_factor"] = float(req_params["dynamic_args"][self._unique_id]["quantisation_factor"])
        except (KeyError, TypeError):
            try:
                qsr_params["quantisation_factor"] = float(req_params["dynamic_args"]["for_all_qsrs"]["quantisation_factor"])
            except (TypeError, KeyError):
                pass
        return qsr_params


.. note::
    Make sure that the QSR namespace **has precedence** over the global `'for_all_qsrs'` one.
