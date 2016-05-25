Region Connection Calculus 8
============================

Description
-----------

*Region Connection Calculus* (RCC) [1]_ [2]_ is intended to serve for qualitative spatial representation and reasoning. RCC abstractly describes regions (in Euclidean space, or in a topological space) by their possible relations to each other.

RCC8 consists of 8 basic relations that are possible between two regions.

Relations
---------

All the possible RCC8 relations between a blue object X and a red object Y are:

+-------------------+------------------------------------------------+-------------------------------------------------+
| Relation          | Illustration                                   | Interpretation                                  +
+===================+================================================+=================================================+
| X **dc** Y        | .. image:: ../images/rcc8_dc.png               | X is disconnected from Y.                       |
+-------------------+------------------------------------------------+-------------------------------------------------+
| X **ec** Y        | .. image:: ../images/rcc8_ec.png               | X is externally connected to Y.                 |
+-------------------+------------------------------------------------+-------------------------------------------------+
| X **po** Y        | .. image:: ../images/rcc8_po.png               | X is partially overlapping Y.                   |
+-------------------+------------------------------------------------+-------------------------------------------------+
| X **tpp** Y       | .. image:: ../images/rcc8_tpp.png              | X is a tangential proper part of Y.             |
+-------------------+------------------------------------------------+-------------------------------------------------+
| X **ntpp** Y      | .. image:: ../images/rcc8_ntpp.png             | X is a non-tangential proper part of Y.         |
+-------------------+------------------------------------------------+-------------------------------------------------+
| X **eq** Y        | .. image:: ../images/rcc8_eq.png               | X is equal to Y.                                |
+-------------------+------------------------------------------------+-------------------------------------------------+
| X **tppi** Y      | .. image:: ../images/rcc8_tppi.png             | X is a tangential proper part inverse of Y.     |
+-------------------+------------------------------------------------+-------------------------------------------------+
| X **ntppi** Y     | .. image:: ../images/rcc8_ntppi.png            | X is a non-tangential proper part inverse of Y. |
+-------------------+------------------------------------------------+-------------------------------------------------+


API
---

The API can be found :mod:`here <qsrlib_qsrs.qsr_rcc8>`.

References
----------

.. [1] Randell, D. A., Cui, Z. and Cohn, A. G.: A spatial logic based on regions and connection, Proc. 3rd Int. Conf. on Knowledge Representation and Reasoning, Morgan Kaufmann, San Mateo, pp. 165–176, 1992. `(link) <http://wenxion.net/ac/randell92spatial.pdf>`_
.. [2] Anthony G. Cohn, Brandon Bennett, John Gooday, Micholas Mark Gotts: Qualitative Spatial Representation and Reasoning with the Region Connection Calculus. GeoInformatica, 1, 275–316, 1997.
