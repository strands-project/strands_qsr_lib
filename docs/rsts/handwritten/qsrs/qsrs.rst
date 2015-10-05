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
| **argd**       | Qualitative Argument Distances                    | :doc:`descr. <argd>` \| :mod:`api <qsrlib_qsrs.qsr_arg_relations_distance>`                             | [6]_           |
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
| **ra**         | Rectangle Algebra                                 | :doc:`descr. <ra>` \| :mod:`api <qsrlib_qsrs.qsr_ra>`                                                   | [5]_           |
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

.. [1] U. F. Andrew: Qualitative spatial reasoning about cardinal directions. In Proc. of the 7th Austrian Conf. on Artificial Intelligence. Morgan Kaufmann, Baltimore, 1991.
.. [2] D. A. Randell, Z. Cui and A. G. Cohn: A spatial logic based on regions and connection. In Proc. 3rd Int. Conf. on Knowledge Representation and Reasoning, Morgan Kaufmann, San Mateo, pp. 165–176, 1992. `(link) <http://wenxion.net/ac/randell92spatial.pdf>`_
.. [3] A. G. Cohn, B. Bennett, J. Gooday and M. M. Gotts: Qualitative Spatial Representation and Reasoning with the Region Connection Calculus. GeoInformatica, 1, pp. 275–316, 1997.
.. [4] Introduction to the Ternary Point Configuration Calculus (TPCC) http://www.sfbtr8.spatial-cognition.de/project/r3/QualitativeCalculi/TPCC/index.html
.. [5] P. Balbiani, J.-F. Condotta and L. F. del Cerro: A model for reasoning about bi-dimensional temporal relations. In Proc. of the Sixth International Conference on Principles of Knowledge Representation and Reasoning (KR'98), A.G. Cohn, L. K. Schubert and S. C. Shapiro (eds). Morgan Kaufmann, pp. 124–130. Trento, Italy, June 2–5 1998.
.. [6] J. Chen, A. G. Cohn, D. Liu, S. Wang, J. Ouyang and Q. Yu: A survey of qualitative spatial representations. The Knowledge Engineering Review, 30 , pp 106-136, 2015.
