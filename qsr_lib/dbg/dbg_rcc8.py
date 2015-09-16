#!/usr/bin/python

# import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle

class Dbg(object):
    def __init__(self):
        pass

    def return_bounding_box_2d(self, x, y, xsize, ysize):
        """Return the bounding box

        :param x: x center
        :param y: y center
        :param xsize: x size
        :param ysize: y size
        :return: list(x1, y1, x2, y2) where (x1, y1) and (x2, y2) are the coordinates of the diagonal points of the
                bounding box depending on your coordinates frame
        """
        if xsize <= 0 or ysize <= 0:
            print("ERROR: can't compute bounding box, xsize or height has no positive value")
            return []
        return [x-xsize/2, y-ysize/2, x+xsize/2, y+ysize/2]


    def compute_qsr(self, bb1, bb2):
        """Wrapper for __compute_qsr

        :param bb1: diagonal points coordinates of first bounding box (x1, y1, x2, y2)
        :param bb2: diagonal points coordinates of second bounding box (x1, y1, x2, y2)        :return: an RCC depending on your implementation
        """
        return self.__compute_qsr(bb1, bb2)


    def __compute_qsr(self, bb1, bb2):
        """Return RCC8
            :param bb1: diagonal points coordinates of first bounding box (x1, y1, x2, y2)
            :param bb2: diagonal points coordinates of second bounding box (x1, y1, x2, y2)
            :return: an RCC8 relation from the following:
             {0:'dc'}      x is disconnected from y
             {1:'ec'}      x is externally connected with y
             {2:'po'}      x partially overlaps y
             {3:'eq'}      x equals y
             {4:'tpp'}     x is a tangential proper part of y
             {5:'ntpp'}    y is a non-tangential proper part of x
             {6:'tppi'}    y is a tangential proper part of x
             {7:'ntppi'}   y is a non-tangential proper part of x
             +-------------+         +-------------+
             |a            |         |a            |
             |             |         |             |
             |      A      |         |      B      |
             |             |         |             |
             |            b|         |            b|
             +-------------+         +-------------+
        """

        # Object 1 Top Left X
        ax = bb1[0]
        # Object 1 Top Left Y
        ay = bb1[1]
        # Object 2 Top Left X
        cx = bb2[0]
        # Object 2 Top Left X
        cy = bb2[1]
        # Object 1 Bottom Right X
        bx = bb1[2]
        # Object 1 Bottom Right Y
        by = bb1[3]
        # Object 2 Bottom Right X
        dx = bb2[2]
        # Object 2 Bottom Right Y
        dy = bb2[3]

        # CALCULATE EQ
        # Is object1 equal to object2
        if(bb1 == bb2):
            return "eq"

        # Are objects disconnected?
        # Cond1. If A's left edge is to the right of the B's right edge, - then A is Totally to right Of B
        # Cond2. If A's right edge is to the left of the B's left edge, - then A is Totally to left Of B
        # Cond3. If A's top edge is below B's bottom edge, - then A is Totally below B
        # Cond4. If A's bottom edge is above B's top edge, - then A is Totally above B

        #    Cond1        Cond2        Cond3        Cond4
        if (ax > dx) or (bx < cx) or (ay > dy) or (by < cy):
            return "dc"

        # Is one object inside the other
        BinsideA = (ax <= cx) and (ay <= cy) and (bx >= dx) and (by >= dy)
        AinsideB = (ax >= cx) and (ay >= cy) and (bx <= dx) and (by <= dy)

        # Do objects share an X or Y (but are not necessarily touching)
        sameX = (ax == cx or ax == dx or bx == cx or bx == dx)
        sameY = (ay == cy or ay == dy or by == cy or by == dy)

        if AinsideB and (sameX or sameY):
            return "tpp"

        if BinsideA and (sameX or sameY):
            return "tppi"

        if AinsideB:
            return "ntpp"

        if BinsideA:
            return "ntppi"

        # Are objects touching?
        # Cond1. If A's left edge is equal to B's right edge, - then A is to the right of B and touching
        # Cond2. If A's right edge is qual to B's left edge, - then A is to the left of B and touching
        # Cond3. If A's top edge equal to B's bottom edge, - then A is below B and touching
        # Cond4. If A's bottom edge equal to B's top edge, - then A is above B and touching

        #    Cond1        Cond2        Cond3        Cond4
        if (ax == dx) or (bx == cx) or (ay == dy) or (by == cy):
            return "ec"

        # If none of the other conditions are met, the objects must be parially overlapping
        return "po"



def plot_bbs(bb1, bb2):
    plt.figure()
    ax = plt.gca()
    # ax.invert_yaxis()
    ax.add_patch(Rectangle((bb1[0], bb1[1]), bb1[2]-bb1[0], bb1[3]-bb1[1], alpha=1, facecolor="blue"))
    ax.annotate("o1", (bb1[0], bb1[1]), color='black', weight='bold', fontsize=14)
    ax.add_patch(Rectangle((bb2[0], bb2[1]), bb2[2]-bb2[0], bb2[3]-bb2[1], alpha=1, facecolor="red"))
    ax.annotate("o2", (bb2[0], bb2[1]), color='black', weight='bold', fontsize=14)
    h = 6
    l = 0
    # ax.set_xlim(l, h)
    # ax.set_ylim(l, h)
    ax.set_xlim(l, h)
    ax.set_ylim(h, l)
    plt.show()


if __name__ == '__main__':
    dbg = Dbg()

    """
         {0:'dc'}      x is disconnected from y
         {1:'ec'}      x is externally connected with y
         {2:'po'}      x partially overlaps y
         {3:'eq'}      x equals y
         {4:'tpp'}     x is a tangential proper part of y
         {5:'ntpp'}    y is a non-tangential proper part of x
         {6:'tppi'}    y is a tangential proper part of x
         {7:'ntppi'}   y is a non-tangential proper part of x
         +-------------+         +-------------+
         |a            |         |a            |
         |             |         |             |
         |      A      |         |      B      |
         |             |         |             |
         |            b|         |            b|
         +-------------+         +-------------+
    """

    # Play with these to test (x_center, y_center, xsize(i.e. x-size), ysize(i.e. y-size))
    o1 = (3., 3., 2., 2.)
    o2 = (3., 2., 1., 1.)

    o1 = dbg.return_bounding_box_2d(o1[0], o1[1], o1[2], o1[3])
    o2 = dbg.return_bounding_box_2d(o2[0], o2[1], o2[2], o2[3])

    # Bounding boxes
    # print("o1:", o1)
    # print("o2:", o2)

    # Relations
    print("o1o2:", dbg.compute_qsr(o1, o2))
    print("o2o1:", dbg.compute_qsr(o2, o1))

    # Plot the boxes
    plot_bbs(o1, o2)
