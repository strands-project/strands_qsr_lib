Usage
=====

Minimal Working Example
~~~~~~~~~~~~~~~~~~~~~~~

Compute QSRs with the MWE script `mwe.py` in
`strands_qsr_lib/qsr_lib/scripts/`:

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
        options = sorted(qsrlib.get_qsrs_registry().keys())
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

Usage in more detail
~~~~~~~~~~~~~~~~~~~~

Basically the above code consists of the following simple steps: \*
Create a `QSRlib` object \* Convert your data in to QSRlib standard
input format \* Make a request to QSRlib \* Parse the QSRlib response

With the first three being the necessary ones and the parsing step
provided as an example to give you insight on the QSRlib response data
structure.

Create a `QSRlib` object
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code:: python

    qsrlib = QSRlib()

*Note:* This step can be omitted if you want to use QSRlib with ROS.

Convert your data in to QSRlib standard input format
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

Make a request to QSRlib
^^^^^^^^^^^^^^^^^^^^^^^^

.. code:: python

        # make a QSRlib request message
        qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world)
        # request your QSRs
        qsrlib_response_message = qsrlib.request_qsrs(request_message=qsrlib_request_message)

Make a request to QSRlib using ROS
''''''''''''''''''''''''''''''''''

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
