# -*- coding: utf-8 -*-
from __future__ import print_function, division
from numpy import isnan
from qsrlib_qsrs.qsr_dyadic_abstractclass import QSR_Dyadic_1t_Abstractclass


class QSR_BA(QSR_Dyadic_1t_Abstractclass):
    """Block Algebra.

    Members:
        * _unique_id: "ba"
        * _all_possible_relations: ("<", ">", "m", "mi", "o", "oi", "s", "si", "d", "di", "f", "fi", "=")
        * _dtype: "points"

    .. seealso:: For further details about BA, refer to its :doc:`description. <../handwritten/qsrs/ba>`
    """

    _unique_id = "ba"
    """str: Unique identifier name of the QSR."""

    _all_possible_relations = ("<", ">", "m", "mi", "o", "oi", "s", "si", "d", "di", "f", "fi", "=")
    """tuple: All possible relations of the QSR."""

    _dtype = "points"
    """str: On what kind of data the QSR works with."""

    _inverse_map = {"<": ">", "m": "mi", "o": "oi", "s": "si", "d": "di", "f": "fi",
                    ">": "<", "mi": "m", "o1": "o", "si": "s", "di": "d", "fi": "f"}
    """dict: Inverse relations"""


    def __init__(self):
        """Constructor.

        :return:
        """
        super(QSR_BA, self).__init__()
    
        
    def _compute_qsr(self, data1, data2, qsr_params, **kwargs):
        """Compute QSR value.

        :param data1: First object data.
        :type data1: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param data2: Second object data.
        :type data2: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param qsr_params: QSR specific parameters passed in `dynamic_args`.
        :type qsr_params: dict
        :param kwargs: kwargs arguments.
        :return: Computed QSR value.
        :rtype: str
        """
        
        bb1 = self.return_bounding_box_3d(data1) 
        bb2 = self.return_bounding_box_3d(data2)
        
        if len(bb1) == 6 and len(bb2) == 6:
            return ",".join([self.__allen((bb1[0], bb1[3]), (bb2[0], bb2[3])),
                             self.__allen((bb1[1], bb1[4]), (bb2[1], bb2[4])),
                             self.__allen((bb1[2], bb1[5]), (bb2[2], bb2[5]))])
        else:
            raise ValueError("bb1 and bb2 must have length of 6 (3D) for block algebra")


    def return_bounding_box_3d(self, data, xsize_minimal=0, ysize_minimal=0, zsize_minimal=0):  
        """Compute the 3D bounding box of the object.

        :param data: Object data.
        :type data: :class:`Object_State <qsrlib_io.world_trace.Object_State>`
        :param xsize_minimal: If object has no x-size (i.e. a point) then compute bounding box based on this minimal x-size.
        :type xsize_minimal: positive int or float
        :param ysize_minimal: If object has no y-size (i.e. a point) then compute bounding box based on this minimal y-size.
        :type ysize_minimal: positive int or float
        :param zsize_minimal: If object has no z-size (i.e. a point) then compute bounding box based on this minimal z-size.
        :type zsize_minimal: positive int or float
        :return: The 3D coordinates of the closest to origin and farthest from origin corners of the bounding box.
        :rtype: list of 6 int or float
        """
        
        xsize = xsize_minimal if isnan(data.xsize) else data.xsize
        ysize = ysize_minimal if isnan(data.ysize) else data.ysize
        zsize = zsize_minimal if isnan(data.zsize) else data.zsize
    
        x1 = data.x - xsize/2 
        y1 = data.y - ysize/2 
        z1 = data.z - zsize/2
        
        x2 = data.x + xsize/2         
        y2 = data.y + ysize/2          
        z2 = data.z + zsize/2 
        
        return [x1, y1, z1, x2, y2, z2]         
            

    def __allen(self, i1, i2):
        if isnan(i1).any() or isnan(i2).any():  
            raise ValueError("illegal 'nan' values found")

        if i1[1] < i2[0]:
            return "<"
        if i1[1] == i2[0]:
            return "m"
        if i1[0] < i2[0] < i1[1] and i2[0] < i1[1] < i2[1]:
            return "o"
        if i1[0] == i2[0] and i1[1] < i2[1]:
            return "s"
        if i2[0] < i1[0] < i2[1] and i2[0] < i1[1] < i2[1]:
            return "d"
        if i2[0] < i1[0] < i2[1] and i1[1] == i2[1]:
            return "f"
        if i1[0] == i2[0] and i1[1] == i2[1]:
            return "="
        return self._inverse_map[self.__allen(i2, i1)]