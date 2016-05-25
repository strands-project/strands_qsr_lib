QSRs
====

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
    rcc4
    rcc5
    rcc8
    tpcc


Currently, the following QSRs are included in the library:

+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| ID             | Name                                              | Links                                                                                                   | Reference      |
+================+===================================================+=========================================================================================================+================+
| **argd**       | Qualitative Distance Calculus                     | :doc:`descr. <argd>` \| :mod:`api <qsrlib_qsrs.qsr_arg_relations_distance>`                             | [7]_           |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **argprobd**   | Probablistic Qualitative Distance Calculus        | :doc:`descr. <argprobd>` \| :mod:`api <qsrlib_qsrs.qsr_arg_prob_relations_distance>`                    |                |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **cardir**     | Cardinal Directions                               | :doc:`descr. <cardir>` \| :mod:`api <qsrlib_qsrs.qsr_cardinal_direction>`                               | [1]_           |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **mos**        | Moving or Stationary                              | :doc:`descr. <mos>` \| :mod:`api <qsrlib_qsrs.qsr_moving_or_stationary>`                                |                |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **mwe**        | Minimal Working Example                           | :doc:`descr. <mwe>` \| :mod:`api <qsrlib_qsrs.qsr_new_mwe>`                                             |                |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **qtcbs**      | Qualitative Trajectory Calculus *b*               | :doc:`descr. <qtcbs>` \| :mod:`api <qsrlib_qsrs.qsr_qtc_b_simplified>`                                  | [8]_ [9]_      |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **qtccs**      | Qualitative Trajectory Calculus *c*               | :doc:`descr. <qtccs>` \| :mod:`api <qsrlib_qsrs.qsr_qtc_c_simplified>`                                  | [8]_ [9]_      |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **qtcbcs**     | Qualitative Trajectory Calculus *bc*              | :doc:`descr. <qtcbcs>` \| :mod:`api <qsrlib_qsrs.qsr_qtc_bc_simplified>`                                | [8]_ [9]_      |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **ra**         | Rectangle Algebra                                 | :doc:`descr. <ra>` \| :mod:`api <qsrlib_qsrs.qsr_ra>`                                                   | [5]_           |
+----------------+---------------------------------------------------+---------------------------------------------------------------------------------------------------------+----------------+
| **rcc2**       | Region Connection Calculus 2                      | :doc:`descr. <rcc2>` \| :mod:`api <qsrlib_qsrs.qsr_rcc2>`                                               | [2]_ [3]_      |
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

.. [1] Frank, A. U. 1990. Qualitative Spatial Reasoning about Cardinal Directions. In Mark, M., and White, D., eds., Au- tocarto 10. Baltimore: ACSM/ASPRS.
.. [2] D. A. Randell, Z. Cui and A. G. Cohn: A spatial logic based on regions and connection. In Proc. 3rd Int. Conf. on Knowledge Representation and Reasoning, Morgan Kaufmann, San Mateo, pp. 165–176, 1992.
.. [3] A. G. Cohn, B. Bennett, J. Gooday and M. M. Gotts: Qualitative Spatial Representation and Reasoning with the Region Connection Calculus. GeoInformatica, 1, pp. 275–316, 1997.
.. [4] Moratz, R.; Nebel, B.; and Freksa, C. 2003. Qualita- tive spatial reasoning about relative position: The tradeoff between strong formal properties and successful reasoning about route graphs. In Freksa, C.; Brauer, W.; Habel, C.; and Wender, K. F., eds., Lecture Notes in Artificial Intelligence 2685: Spatial Cognition III. Berlin, Heidelberg: Springer Verlag. 385–400.
.. [5] P. Balbiani, J.-F. Condotta and L. F. del Cerro: A model for reasoning about bi-dimensional temporal relations. In Proc. of the Sixth International Conference on Principles of Knowledge Representation and Reasoning (KR'98), A.G. Cohn, L. K. Schubert and S. C. Shapiro (eds). Morgan Kaufmann, pp. 124–130. Trento, Italy, June 2–5 1998.
.. [6] J. Chen, A. G. Cohn, D. Liu, S. Wang, J. Ouyang and Q. Yu: A survey of qualitative spatial representations. The Knowledge Engineering Review, 30 , pp 106-136, 2015.
.. [7] Clementini, E.; Felice, P. D.; and Hernandez, D. 1997. Qualitative representation of positional information. Artificial Intelligence 95(2):317–356.
.. [8] Van de Weghe, N.; Cohn, A.; De Tre ́, B.; and De Maeyer, P. 2005. A Qualitative Trajectory Calculus as a basis for representing moving objects in Geographical Information Systems. Control and Cybernetics 35(1):97–120.
.. [9] Delafontaine, M.; Cohn, A. G.; and Van de Weghe, N. 2011. Implementing a qualitative calculus to analyse moving point objects. Expert Systems with Applications 38(5):5187–5196.
