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
        :param bb2: diagonal points coordinates of second bounding box (x1, y1, x2, y2)
        :return: an RCC depending on your implementation
        """
        return self.__compute_qsr(bb1, bb2)

    def __compute_qsr(self, bb1, bb2):
        """Replace with your own

        :param bb1: diagonal points coordinates of first bounding box (x1, y1, x2, y2)
        :param bb2: diagonal points coordinates of second bounding box (x1, y1, x2, y2)        :return: an RCC depending on your implementation
        :return: an RCC depending on your implementation
        """
        raise NotImplementedError("Replace with your code")



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

    # Play with these to test (x_center, y_center, xsize(i.e. x-size), ysize(i.e. y-size))
    o1 = (2.0, 2.0, 2., 2.)
    o2 = (4.0, 3.0, 1., 1.)

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
