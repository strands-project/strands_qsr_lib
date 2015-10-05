Supported QSRs
==============

.. toctree::
    :titlesonly:
    :hidden:

    argd
    argprobd
    cardir
    mos
    mwe
    qtcbs
    qtccs
    qtcbcs
    ra
    rcc2
    rcc3
    rcc4
    rcc5
    rcc8
    tpcc


Currently, the following QSRs are supported:

+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| ID             | Name                                              | Links                                                                                                   | Reference      |
+================+===================================================+=========================================================================================================+================+
| **argd**       | Qualitative Argument Distances                    | :doc:`descr. <argd>` \| :mod:`api <qsrlib_qsrs.qsr_arg_relations_abstractclass>`                        | ?              |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **argprobd**   | Qualitative Argument Probabilistic Distances      | :doc:`descr. <argprobd>` \| :mod:`api <qsrlib_qsrs.qsr_arg_prob_relations_distance>`                    | ?              |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **cardir**     | Cardinal Directions                               | :doc:`descr. <cardir>` \| :mod:`api <qsrlib_qsrs.qsr_cardinal_direction>`                               | [1]_           |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **mos**        | Moving or Stationary                              | :doc:`descr. <mos>` \| :mod:`api <qsrlib_qsrs.qsr_moving_or_stationary>`                                |                |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **mwe**        | Minimal Working Example                           | :doc:`descr. <mwe>` \| :mod:`api <qsrlib_qsrs.qsr_new_mwe>`                                             |                |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **qtcbs**      | Qualitative Trajectory Calculus *b* Simplified    | :doc:`descr. <qtcbs>` \| :mod:`api <qsrlib_qsrs.qsr_qtc_b_simplified>`                                  | ?              |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **qtccs**      | Qualitative Trajectory Calculus *c* Simplified    | :doc:`descr. <qtccs>` \| :mod:`api <qsrlib_qsrs.qsr_qtc_c_simplified>`                                  | ?              |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **qtcbcs**     | Qualitative Trajectory Calculus *bc* Simplified   | :doc:`descr. <qtcbcs>` \| :mod:`api <qsrlib_qsrs.qsr_qtc_bc_simplified>`                                | ?              |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **ra**         | Regional Algebra                                  | :doc:`descr. <ra>` \| :mod:`api <qsrlib_qsrs.qsr_ra>`                                                   | ?              |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **rcc2**       | Region Connection Calculus 2                      | :doc:`descr. <rcc2>` \| :mod:`api <qsrlib_qsrs.qsr_rcc2>`                                               | [2]_ [3]_      |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **rcc3**       | Region Connection Calculus 3                      | :doc:`descr. <rcc3>` \| :mod:`api <qsrlib_qsrs.qsr_rcc3_rectangle_bounding_boxes_2d>`                   | [2]_ [3]_      |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **rcc4**       | Region Connection Calculus 4                      | :doc:`descr. <rcc4>` \| :mod:`api <qsrlib_qsrs.qsr_rcc4>`                                               | [2]_ [3]_      |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **rcc5**       | Region Connection Calculus 5                      | :doc:`descr. <rcc5>` \| :mod:`api <qsrlib_qsrs.qsr_rcc5>`                                               | [2]_ [3]_      |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **rcc8**       | Region Connection Calculus 8                      | :doc:`descr. <rcc8>` \| :mod:`api <qsrlib_qsrs.qsr_rcc8>`                                               | [2]_ [3]_      |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **tpcc**       | Ternary Point Configuration Calculus              | :doc:`descr. <tpcc>` \| :mod:`api <qsrlib_qsrs.qsr_tpcc>`                                               | [4]_           |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+


Special Topics
--------------

.. toctree::
    :titlesonly:
    :hidden:

    allen
    qstag

**Allen's Interval Algebra**

*Allen's Interval Algebra* is a calculus for temporal reasoning. For further details see :doc:`this page <allen>`.

**Qualitative Spatio-Temporal Activity Graphs**

QSRlib provides also functionalities to represent time-series QSRs as a graph structure,
called *Qualitative Spatio-Temporal Activity Graphs* (QSTAG).
For details, please refer to its :doc:`documentation <qstag>`.


References
----------

.. [1] Andrew, U. F. "Qualitative spatial reasoning about cardinal directions." Proc. of the 7th Austrian Conf. on Artificial Intelligence. Baltimore: Morgan Kaufmann. 1991.
.. [2] Randell, D. A., Cui, Z. and Cohn, A. G.: A spatial logic based on regions and connection, Proc. 3rd Int. Conf. on Knowledge Representation and Reasoning, Morgan Kaufmann, San Mateo, pp. 165–176, 1992. `(link) <http://wenxion.net/ac/randell92spatial.pdf>`_
.. [3] Anthony G. Cohn, Brandon Bennett, John Gooday, Micholas Mark Gotts: Qualitative Spatial Representation and Reasoning with the Region Connection Calculus. GeoInformatica, 1, 275–316, 1997.
.. [4] Introduction to the Ternary Point Configuration Calculus (TPCC) http://www.sfbtr8.spatial-cognition.de/project/r3/QualitativeCalculi/TPCC/index.html
