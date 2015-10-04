Installation
============

Dependencies
------------

* The main dependency is `numpy` which usually gets installed with python.
* Also `matplotlib` is used by some visualization modules, but again it should be already installed in your system.
* `igraph` is required if you want to produce :doc:`QSTAGs <qsrs/qstag>`.

    * In linux it should be in your distro's repositories. For example in modern ubuntu you can install it as ``apt-get install python-igraph`` (don't forget the `sudo` if needed).
    * For other systems please refer to igraph_ installation instructions.

As python package
-----------------

Installation as a python package consists of the following two steps:

1. Clone the `QSRlib git repository`_.
2. Include the QSRlib source folder.

For example, let's say you want to install it in your home directory.
Then the bash commands for the above two steps are:

.. code:: bash

    git clone https://github.com/strands-project/strands_qsr_lib.git
    export PYTHONPATH=$HOME/strands_qsr_lib/qsr_lib/src:$PYTHONPATH

.. note::
    You can also include the export in your `.bashrc` file.

As ROS catkin package
---------------------

You need to firstly follow the `installation instructions for ROS`_. Then clone the `QSRlib git repository`_
in your catkin workspace src folder, and don't forget to make it.

.. note::
    For STRANDS_ developers and users, note that QSRlib can be installed using the usual `software guidelines`_.


.. _igraph: http://igraph.org/python/
.. _`QSRlib git repository`: https://github.com/strands-project/strands_qsr_lib.git
.. _`installation instructions for ROS`: http://www.ros.org/install
.. _STRANDS: http://strands.acin.tuwien.ac.at
.. _`software guidelines`: http://strands.acin.tuwien.ac.at/software.html
