For users
=========

The following figure presents a flowchart with the main step processes for computing QSRs via the library. Raw data first needs to  be converted into the common input data format of QSRlib, which represents a timeseries of the states of the perceived objects, such Cartesian position and rotation, size of the object in each dimension, and allows other custom information about the objects to be kept on a per QSR-need basis. Utility functions are provided that allow easy conversion of the raw data to this standard input data structure. This input data structure, the names of the requested QSRs to be computed and other options that control their behaviours are used to create a request message to the QSRlib server, which upon computation returns a response message that includes the computed QSRs as an output data structure similar to the input one, i.e. a timeseries of the QSRs between the objects, as well as other information.

.. image:: images/flowchart.pdf
    :width: 500px
    :align: center
    :alt: Flowchart showing the main step processes for computing QSRs via the library.

The following minimal working example explains how to conduct these steps and use the library to compute QSRs from your data.

.. _MWE:

Minimal Working Example
-----------------------

Compute QSRs with the MWE script `mwe.py` in `strands_qsr_lib/qsr_lib/scripts/`:

.. code:: bash

    ./mwe.py <qsr_name>

e.g.

.. code:: bash

    ./mwe.py rcc8

MWE source code:

.. code:: python

    #!/usr/bin/env python
    # -*- coding: utf-8 -*-
    from __future__ import print_function, division
    from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
    from qsrlib_io.world_trace import Object_State, World_Trace
    import argparse

    def pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message):
        print(which_qsr, "request was made at ", str(qsrlib_response_message.timestamp_request_made)
              + " and received at " + str(qsrlib_response_message.timestamp_request_received)
              + " and computed at " + str(qsrlib_response_message.timestamp_qsrs_computed))
        print("---")
        print("Response is:")
        for t in qsrlib_response_message.qsrs.get_sorted_timestamps():
            foo = str(t) + ": "
            for k, v in zip(qsrlib_response_message.qsrs.trace[t].qsrs.keys(),
                            qsrlib_response_message.qsrs.trace[t].qsrs.values()):
                foo += str(k) + ":" + str(v.qsr) + "; "
            print(foo)

    if __name__ == "__main__":
        # ****************************************************************************************************
        # create a QSRlib object if there isn't one already
        qsrlib = QSRlib()

        # ****************************************************************************************************
        # parse command line arguments
        options = sorted(qsrlib.qsrs_registry.keys())
        parser = argparse.ArgumentParser()
        parser.add_argument("qsr", help="choose qsr: %s" % options, type=str)
        args = parser.parse_args()
        if args.qsr in options:
            which_qsr = args.qsr
        else:
            raise ValueError("qsr not found, keywords: %s" % options)

        # ****************************************************************************************************
        # make some input data
        world = World_Trace()
        o1 = [Object_State(name="o1", timestamp=0, x=1., y=1., width=5., length=8.),
              Object_State(name="o1", timestamp=1, x=1., y=2., width=5., length=8.),
              Object_State(name="o1", timestamp=2, x=1., y=3., width=5., length=8.)]

        o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., width=5., length=8.),
              Object_State(name="o2", timestamp=1, x=11., y=2., width=5., length=8.),
              Object_State(name="o2", timestamp=2, x=11., y=3., width=5., length=8.)]
        world.add_object_state_series(o1)
        world.add_object_state_series(o2)

        # ****************************************************************************************************
        # make a QSRlib request message
        qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world)
        # request your QSRs
        qsrlib_response_message = qsrlib.request_qsrs(request_message=qsrlib_request_message)

        # ****************************************************************************************************
        # print out your QSRs
        pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message)


Line-by-line explanation
~~~~~~~~~~~~~~~~~~~~~~~~

Basically the above code consists of the following simple steps:

    * Create a `QSRlib` object
    * Convert your data in to QSRlib standard input format
    * Make a request to QSRlib
    * Parse the QSRlib response

With the first three being the necessary ones and the parsing step
provided as an example to give you insight on the QSRlib response data
structure.

Create a `QSRlib` object
^^^^^^^^^^^^^^^^^^^^^^^^

.. code:: python

    qsrlib = QSRlib()

.. note::
    This step can be omitted if you want to use QSRlib with ROS.

Convert your data in to QSRlib standard input format
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Below is one way of creating your input data. You can find more details on how to convert your data
in QSRlib input format in the Section about the :ref:`io`.

.. code:: python

        world = World_Trace()
        o1 = [Object_State(name="o1", timestamp=0, x=1., y=1., width=5., length=8.),
              Object_State(name="o1", timestamp=1, x=1., y=2., width=5., length=8.),
              Object_State(name="o1", timestamp=2, x=1., y=3., width=5., length=8.)]

        o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., width=5., length=8.),
              Object_State(name="o2", timestamp=1, x=11., y=2., width=5., length=8.),
              Object_State(name="o2", timestamp=2, x=11., y=3., width=5., length=8.)]
        world.add_object_state_series(o1)
        world.add_object_state_series(o2)

.. _mwe_request:

Make a request to QSRlib
^^^^^^^^^^^^^^^^^^^^^^^^

.. code:: python

        # make a QSRlib request message
        qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world)
        # request your QSRs
        qsrlib_response_message = qsrlib.request_qsrs(request_message=qsrlib_request_message)

.. _ros-mwe:

Via ROS
'''''''

If you want to use ROS then you need to firstly run the QSRlib ROS
server as follows:

.. code:: bash

    rosrun qsr_lib qsrlib_ros_server.py

and the request is slightly different:

.. code:: python

        try:
            import rospy
            from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client
        except ImportError:
            raise ImportError("ROS not found")
        try:
            import cPickle as pickle
        except:
            import pickle
        client_node = rospy.init_node("qsr_lib_ros_client_example")
        cln = QSRlib_ROS_Client()
        qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world)
        req = cln.make_ros_request_message(qsrlib_request_message)
        res = cln.request_qsrs(req)
        qsrlib_response_message = pickle.loads(res.data)

Parse the QSRlib response
^^^^^^^^^^^^^^^^^^^^^^^^^

.. code:: python

    def pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message):
        print(which_qsr, "request was made at ", str(qsrlib_response_message.timestamp_request_made)
              + " and received at " + str(qsrlib_response_message.timestamp_request_received)
              + " and computed at " + str(qsrlib_response_message.timestamp_qsrs_computed))
        print("---")
        print("Response is:")
        for t in qsrlib_response_message.qsrs.get_sorted_timestamps():
            foo = str(t) + ": "
            for k, v in zip(qsrlib_response_message.qsrs.trace[t].qsrs.keys(),
                            qsrlib_response_message.qsrs.trace[t].qsrs.values()):
                foo += str(k) + ":" + str(v.qsr) + "; "
            print(foo)


Advanced Topics
---------------

.. _io:

I/O data structures
~~~~~~~~~~~~~~~~~~~

.. _World_Trace:

Input: `World_Trace`
^^^^^^^^^^^^^^^^^^^^

QSRlib acccepts input in its own standard format,
which is a :class:`World_Trace <qsrlib_io.world_trace.World_Trace>` object.

`World_Trace` provides a number of convenience methods to convert your data into this format.
One additional handy method, further to the one presented earlier in the MWE_ section, is
:meth:`add_object_track_from_list <qsrlib_io.world_trace.World_Trace.add_object_track_from_list>`,
which allows to add an object's positions stored in a list of tuples.

The main member of `World_Trace` is ``trace``, which is a python dictionary with keys being timestamps
and values being objects of the class
:class:`World_State <qsrlib_io.world_trace.World_State>`. In a `World_State` object the main member is
``objects``, which is again a dictionary with keys being the unique name of the object and values
objects of the class :class:`Object_State <qsrlib_io.world_trace.Object_State>`. An `Object_State` object
holds the information about an object in the world at that particular timestamp.

So for example to get the ``x``-coordinate of an object called ``o1`` at timestamp ``4`` from a `World_Trace`
object called ``world`` we would write:

.. code:: python

    world.trace[4].objects['o1'].x

.. note::
    `World_Trace` should not be confused with the :ref:`QSRlib request message <req_msg>`.

.. _World_QSR_Trace:

Output: `World_QSR_Trace`
^^^^^^^^^^^^^^^^^^^^^^^^^

The standard output data structure is an object of type
:class:`World_QSR_Trace <qsrlib_io.world_qsr_trace.World_QSR_Trace>`.

The main member of `World_QSR_Trace` is ``trace``, which is a dictionary with keys being the timestamps
of the QSRs and usually matching those of `World_Trace`
(depends on QSR type, request parameters, missing values, etc.), and values being objects of the class
:class:`World_QSR_State <qsrlib_io.world_qsr_trace.World_QSR_State>`. In a `World_QSR_State` object the main
member is ``qsrs``, which is a dictionary where the keys are unique combinations of the objects for which
the QSR is, and values being objects of the class :class:`QSR <qsrlib_io.world_qsr_trace.QSR>`. The QSR object
holds, among other information, the QSR value.

For example, for readinging the :doc:`RCC8 <qsrs/rcc8>` relation at timestamp ``4`` between objects ``'o1,o2'``
of a ``world_qsr`` object we would do:

.. code:: python

    world_qsr.trace[4].qsrs['o1,o2'].qsr['rcc8']

A number of convenience slicing functions exist in the :class:`class <qsrlib_io.world_qsr_trace.World_QSR_Trace>`.

.. note::
    `World_QSR_Trace` should not be confused with the :ref:`QSRlib response message <res_msg>`.



.. _req/res:

Request/Response messages
~~~~~~~~~~~~~~~~~~~~~~~~~

.. _req_msg:

Request message
^^^^^^^^^^^^^^^
Once we have our input data in the standard QSRlib input format, i.e. as a World_Trace_ object, the next step is
to create a request message that is passed as argument in the QSRlib request call (as also explained in the
:ref:`MWE example <mwe_request>`).

The request message is an object of the class :class:`QSRlib_Request_Message <qsrlib.qsrlib.QSRlib_Request_Message>`.
The compulsory arguments are `input_data` which is your `World_Trace` object that you created before and
the `which_qsr` which is your requested QSR. If you want only one QSR to be computed it is a string, otherwise
if you want to compute multiple QSRs in one call pass their names as a list. The optional argument `req_made_at`
can be safely ignored. The second optional argument dynamic_args_ is in brief a dictionary with your call and QSR
specific parameters.

.. _res_msg:

Response message
^^^^^^^^^^^^^^^^

The response message of QSRlib is an object of the
class :class:`QSRlib_Response_Message <qsrlib.qsrlib.QSRlib_Response_Message>`.
The main field of it is the `qsrs` one that holds your computed QSRs as a World_QSR_Trace_ object.
The remaining ones are simply timestamps of the process and can be safely ignored.


.. _dynamic_args:

`dynamic_args`
~~~~~~~~~~~~~~

.. _qsrs_for:

Requesting QSRs for specific objects only
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Each QSR has a default behavior for which objects to compute QSRs for. Typically, this is for all valid combinations
of the objects in the World_Trace_. One way to compute QSRs for specific objects is to use the slicing utilities
and subsample the World_Trace_. Still, this might not have the desired effect as most QSRs will also create relations
for mirror pairs as well, e.g. the RCC relations computed for two objects o1 and o2 will be for both o1,o2 as well as
o2,o1.

For these reasons QSRlib allows the user to specify valid objects that are passed in the request in the `dynamic_args`
field. It is easier to explain the usage with the examples shown below.

For all cases assument that we have a World_Trace_ of three objects o1, o2 and o3, and we want to compute two dyadic QSRs
:doc:`CARDIR <qsrs/cardir>` and :doc:`RCC8 <qsrs/rcc8>`, and one monadic :doc:`MOS <qsrs/mos>`.

By default the dyadic QSRs will be computed for o1,o2; o1,o3; o2,o3; o3,o1; and o3,o2; and mos relations will be
computed for o1;o2; and o3.

**Example 1:**
Suppose we want to compute QSR relations for o1,o2 for the dyadic QSRs and for o1 and o2 for MOS. All we need to do is
define a `dynamic_args` as follows (and then pass it to our request).

.. code:: python

    dynamic_args = {'for_all_qsrs': {'qsrs_for': [('o1', 'o2'), 'o1', 'o2']}}


**Example 2:**
Now, suppose that we want to compute CARDIR relations for o1,o2 and o2,o3, RCC8 relations for o1,o3 and MOS
relations for o1 only. This is possible by defining the following `dynamic_args`:

.. code:: python

    dynamic_args = {'cardir': {'qsrs_for': [('o1', 'o2'), ('o2', 'o3')]},
                    'rcc8': {'qsrs_for': [('o1', 'o3')]},
                    'mos': {'qsrs_for': ['o1']}}

.. note::
    We can mix the global namespace `'for_all_qsrs'` with the QSR specific namespace, but note that
    parameters in the QSR namespace always take precedence over the global one.


.. _qsr_specific:

QSR specific parameters
^^^^^^^^^^^^^^^^^^^^^^^

Further to the `qsrs_for` option of dynamic args which is common for all QSRs,
some of the QSRs allow some form of customization during the request call via their namespace in `dynamic_args`.
What options are available depends on each QSR and is the options are given and described in their own API pages.

An example is shown below using :doc:`MOS <qsrs/mos>`, which can take a parameter called `'quantisation_factor'`
that determines the minimum distance of an object between two frames in order to be considered that it has moved.

.. code:: python

    dynamic_args = {'mos': {'quantisation_factor': 1.0}}

Of course we can still mix QSR specific parameters with common ones. So we wanted to compute MOS relations with the
above quantisation factor for only object o1 when there are more we could do:

.. code:: python

    dynamic_args = {'mos': {'quantisation_factor': 1.0,
                            'qsrs_for': ['o1']}}

.. _qstag:

Graph representation
~~~~~~~~~~~~~~~~~~~~

QSRlib provides also functionalities to represent time-series QSRs as a graph structure,
called *Qualitative Spatio-Temporal Activity Graphs* (QSTAG).
For details, please refer to its :doc:`documentation <qsrs/qstag>`.

.. _ros:

ROS calls
~~~~~~~~~

The example of a ROS call in the :ref:`MWE <ros-mwe>` provides a good summary of the usage.
For further details have a look in the API of the
:class:`ROS QSRlib client <qsrlib_ros.qsrlib_ros_client.QSRlib_ROS_Client>`.
