Region Connection Calculus 2
============================

Description
-----------

*Region Connection Calculus* (RCC) [1]_ [2]_ is intended to serve for qualitative spatial representation and reasoning. RCC abstractly describes regions (in Euclidean space, or in a topological space) by their possible relations to each other.

RCC2 is the simplest form of RCC and consists of 2 basic relations that are possible between two regions. It is a stripped down version
of :doc:`RCC8 <rcc8>`. The mapping from RCC8 to RCC2 can be seen below:


+------------+------------+
| RCC8       | RCC2       +
+============+============+
| dc         | dc         |
+------------+------------+
| ec         | c          |
+------------+            +
| po         |            |
+------------+            +
| tpp        |            |
+------------+            +
| ntpp       |            |
+------------+            +
| eq         |            |
+------------+            +
| tppi       |            |
+------------+            +
| ntppi      |            |
+------------+------------+


Relations
---------

All the possible RCC5 relations between a blue object X and a red object Y are:


+-------------------+------------------------------------------------+-------------------------------------------------+
| Relation          | Illustration                                   | Interpretation                                  +
+===================+================================================+=================================================+
| X **dc** Y        | .. image:: ../images/rcc8_dc.png               | X is disconnected from Y.                       |
+-------------------+------------------------------------------------+-------------------------------------------------+
| X **c** Y         | .. image:: ../images/rcc8_ec.png               | X is connected to Y.                            |
+                   +------------------------------------------------+                                                 +
|                   | .. image:: ../images/rcc8_po.png               |                                                 |
+                   +------------------------------------------------+                                                 +
|                   | .. image:: ../images/rcc8_tpp.png              |                                                 |
+                   +------------------------------------------------+                                                 +
|                   | .. image:: ../images/rcc8_ntpp.png             |                                                 |
+                   +------------------------------------------------+                                                 +
|                   | .. image:: ../images/rcc8_eq.png               |                                                 |
+                   +------------------------------------------------+                                                 +
|                   | .. image:: ../images/rcc8_tppi.png             |                                                 |
+                   +------------------------------------------------+                                                 +
|                   | .. image:: ../images/rcc8_ntppi.png            |                                                 |
+-------------------+------------------------------------------------+-------------------------------------------------+


API
---

The API can be found :mod:`here <qsrlib_qsrs.qsr_rcc2>`.


References
----------

.. [1] Randell, D. A., Cui, Z. and Cohn, A. G.: A spatial logic based on regions and connection, Proc. 3rd Int. Conf. on Knowledge Representation and Reasoning, Morgan Kaufmann, San Mateo, pp. 165–176, 1992. `(link) <http://wenxion.net/ac/randell92spatial.pdf>`_
.. [2] Anthony G. Cohn, Brandon Bennett, John Gooday, Micholas Mark Gotts: Qualitative Spatial Representation and Reasoning with the Region Connection Calculus. GeoInformatica, 1, 275–316, 1997.
