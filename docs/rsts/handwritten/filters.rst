
Filters Available:
~~~~~~~~~~~~~~~~~~

This section provides information about the filters that are available in QSRLib.
Currently available filters include:

- Median Filter


Median Filter:
^^^^^^^^^^^^^^

To use the filter, simply add the "filters" key into your dynamic arguments list
when you call QSRlib_Request_Message(), where the value is a parameter dictionary.
The parameters dictionary should have key = "window" and value = integer equal to half the window for the median
filter.

A small example:

.. code:: python

      dynamic_args = {"qtcbs": {"quantisation_factor": quantisation_factor,
                                "validate": validate,
                                "no_collapse": no_collapse},

                      "filters": {"median_filter" : {"window": 3}}
                      }


      qsrlib_request_message = QSRlib_Request_Message(which_qsr=qtcbs, \
                                input_data=world, dynamic_args=dynamic_args)


How it works:


.. code:: python

    def apply_median_filter(qsr_world, params):

This function takes a qsrlib_io.World_QSR_Trace object as input, and performs the following steps:

- Inspects all timepoints in World_QSR_Trace.
- Generates a list of states for each type of requested QSR in the World_QSR_Trace along with a list of timepoints at which they hold.
- These one dimensional lists are passed to a median_filter function
- Multiple requested QSR types are merged back together
- The qsrs in the World_QSR_Trace are then overwritten with the filtered QSRs.


.. code:: python

    median_filter(data, n):

This function implements the median filter over a list of states in data
using a window of 2*n.


.. note:: Because the QSRs are not always ordinal, this function selects the most frequent state inside the requested window. If the most frequent state is ambiguous, then the previous state is returned.

.. note:: The filter returns a World Trace with the same number of timepoints as the original. So the first 2*n -1 timepoints will not have used the entire length window.
