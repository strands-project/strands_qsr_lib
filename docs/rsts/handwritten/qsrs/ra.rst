Rectangle Algebra
=================

Description
-----------

*Rectangle Algebra* (RA) [1]_ [2]_ computes :doc:`Allen's Interval Algebra <allen>` relations on the projected to their xy-axes segments between two 2D-rectangles.

Relations
---------

For example the RA relation between boxes A and B in the case depicted in the figure below is ``A(<, o)B``.

.. figure:: ../images/ra_example.png
    :scale: 50%
    :alt: RA relation: ``A(<, o)B``

The full set of the RA relations is determined by :doc:`Allen's Interval Algebra <allen>` relations.
Therefore, since there are 13 Allen's relations, RA defines 169 possible relations over the xy segments
of two rectangles.


API
---

The API can be found :mod:`here <qsrlib_qsrs.qsr_ra>`.


References
----------

.. [1] P. Balbiani, J.-F. Condotta and L. F. del Cerro: A model for reasoning about bi-dimensional temporal relations. In Proceedings of the Sixth International Conference on Principles of Knowledge Representation and Reasoning (KR'98), A.G. Cohn, L. K. Schubert and S. C. Shapiro (eds). Morgan Kaufmann, pp. 124–130. Trento, Italy, June 2–5 1998.
.. [2] P. Balbiani, J.-F. Condotta and L. F. del Cerro: A new tractable subclass of the rectangle algebra. In Proceedings of the Sixteenth International Joint Conference on Artificial Intelligence (IJCAI'99), T. Dean (ed.). Morgan Kaufmann, pp. 442–447. Stockholm, Sweden, July 31–August 6, 1999.

