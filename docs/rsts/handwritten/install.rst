Installation
============

Dependencies
------------

* `numpy` which usually gets installed with python.
* `matplotlib` is used by some visualization modules, but again it should be already installed in your system.
* `igraph` (used by :doc:`QSTAGs <qsrs/qstag>`).

    * In linux it should be in your distro's repositories. For example in modern ubuntu you can install it as ``apt-get install python-igraph`` (don't forget the `sudo` if needed).
    * For other systems please refer to igraph_ installation instructions.

Install as
----------

You can install QSRlib as a standalone python package, or if you have ROS installed and you want to use it within ROS you can install as a ROS package. The installation steps for each are explained below.

standalone python package
~~~~~~~~~~~~~~~~~~~~~~~~~

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

ROS catkin package
~~~~~~~~~~~~~~~~~~

You need to firstly follow the `installation instructions for ROS`_. Then clone the `QSRlib git repository`_
in your catkin workspace src folder, and don't forget to make it.

.. note::
    For STRANDS_ developers and users, note that QSRlib can be installed using the usual `software guidelines`_.


.. _igraph: http://igraph.org/python/
.. _`QSRlib git repository`: https://github.com/strands-project/strands_qsr_lib.git
.. _`installation instructions for ROS`: http://www.ros.org/install
.. _STRANDS: http://strands.acin.tuwien.ac.at
.. _`software guidelines`: http://strands.acin.tuwien.ac.at/software.html

Verify installation
-------------------

Depending on whether it has been installed as a standalone python package as a ROS package, you can test that the installation and setup works as follows repsectively:

standalone python package
~~~~~~~~~~~~~~~~~~~~~~~~~

If QSRlib was installed as standalone python package, then firstly from the root directory where you installed QSRlib go to `qsr_lib/scripts/` directory and run the `mwe.py` script as follows:

.. code:: bash

    ./mwe rcc8

If you see an output similar to the following then everything is working fine.

::

    rcc8 request was made at  2016-05-17 10:14:39.482158 and received at 2016-05-17 10:14:39.482166 and finished at 2016-05-17 10:14:39.482805
    ---
    Response is:
    0.0: o2,o1:{'rcc8': 'dc'}; o2,o3:{'rcc8': 'po'}; o1,o3:{'rcc8': 'po'}; o1,o2:{'rcc8': 'dc'}; o3,o2:{'rcc8': 'po'}; o3,o1:{'rcc8': 'po'};
    1.0: o2,o1:{'rcc8': 'dc'}; o2,o3:{'rcc8': 'po'}; o1,o3:{'rcc8': 'po'}; o1,o2:{'rcc8': 'dc'}; o3,o2:{'rcc8': 'po'}; o3,o1:{'rcc8': 'po'};
    2.0: o2,o1:{'rcc8': 'po'}; o2,o3:{'rcc8': 'po'}; o1,o3:{'rcc8': 'po'}; o1,o2:{'rcc8': 'po'}; o3,o2:{'rcc8': 'po'}; o3,o1:{'rcc8': 'po'};
    3.0: o2,o1:{'rcc8': 'ec'}; o2,o3:{'rcc8': 'po'}; o1,o3:{'rcc8': 'po'}; o1,o2:{'rcc8': 'ec'}; o3,o2:{'rcc8': 'po'}; o3,o1:{'rcc8': 'po'};
    4.0: o2,o1:{'rcc8': 'ec'}; o2,o3:{'rcc8': 'po'}; o1,o3:{'rcc8': 'dc'}; o1,o2:{'rcc8': 'ec'}; o3,o2:{'rcc8': 'po'}; o3,o1:{'rcc8': 'dc'};

ROS catkin package
~~~~~~~~~~~~~~~~~~

If QSRlib was installed as a ROS package and given that your ROS installation is setup and works, then firstly bring a `roscore` up if you don't have one already:

.. code:: bash

    roscore

Then bring the QSRlib server node up:

.. code:: bash

    rosrun qsr_lib qsrlib_ros_server.py

You should see a message:

::

    [INFO] [WallTime: 1463477006.669860] QSRlib_ROS_Server up and running, listening to: qsr_lib/request

Then you can request QSRs by running for example:

.. code:: bash

    rosrun qsr_lib example_extended.py rcc8 --ros

And you should see an output similar to the one below::

    rcc8 request was made at  2016-05-17 10:24:49.355499 and received at 2016-05-17 10:24:50.014800 and finished at 2016-05-17 10:24:50.015549
    ---
    Response is:
    0.0: o2,o1:{'rcc8': 'dc'}; o1,o2:{'rcc8': 'dc'}; o1,o3:{'rcc8': 'dc'}; o1,o4:{'rcc8': 'dc'}; o2,o4:{'rcc8': 'dc'}; o2,o3:{'rcc8': 'dc'}; o3,o2:{'rcc8': 'dc'}; o3,o1:{'rcc8': 'dc'}; o3,o4:{'rcc8': 'ntpp'}; o4,o3:{'rcc8': 'ntppi'}; o4,o2:{'rcc8': 'dc'}; o4,o1:{'rcc8': 'dc'};
    1.0: o2,o1:{'rcc8': 'dc'}; o1,o2:{'rcc8': 'dc'}; o1,o3:{'rcc8': 'dc'}; o1,o4:{'rcc8': 'dc'}; o2,o4:{'rcc8': 'dc'}; o2,o3:{'rcc8': 'dc'}; o3,o2:{'rcc8': 'dc'}; o3,o1:{'rcc8': 'dc'}; o3,o4:{'rcc8': 'ntpp'}; o4,o3:{'rcc8': 'ntppi'}; o4,o2:{'rcc8': 'dc'}; o4,o1:{'rcc8': 'dc'};
    2.0: o2,o1:{'rcc8': 'dc'}; o1,o2:{'rcc8': 'dc'}; o1,o3:{'rcc8': 'ec'}; o1,o4:{'rcc8': 'po'}; o2,o4:{'rcc8': 'dc'}; o2,o3:{'rcc8': 'dc'}; o3,o2:{'rcc8': 'dc'}; o3,o1:{'rcc8': 'ec'}; o3,o4:{'rcc8': 'ntpp'}; o4,o3:{'rcc8': 'ntppi'}; o4,o2:{'rcc8': 'dc'}; o4,o1:{'rcc8': 'po'};