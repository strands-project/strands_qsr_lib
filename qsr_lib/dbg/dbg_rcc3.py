#!/usr/bin/python

# import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle


def return_bounding_box_2d(x, y, xsize, ysize):
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


def compute_qsr(bb1, bb2):
    """Return symmetrical RCC3 relation

    :param bb1: first bounding box (x_bottom_left, y_bottom_left, x_top_right, y_top_right)
    :param bb2: second bounding box (x_bottom_left, y_bottom_left, x_top_right, y_top_right)
    :return: an RCC3 relation from the following: 'dc':disconnected, 'po':partial overlap, 'o': occluded/part of
    """
    bboxes_intercept_v, rabx, raxPrbx, raby, rayPrby = bboxes_intercept(bb1, bb2)
    if bboxes_intercept_v:
        if rabx > 0.0 or raby > 0:
            return "po", -1
        else:
            occluded_points = count_occluded_points(bb1, bb2)
            if occluded_points >= 4:
                return "o", occluded_points
            else:
                return "po", occluded_points
    else:
        return "dc", 0


def count_occluded_points(bb1, bb2):
    occluded_points = 0
    bb1_4corners = ((bb1[0], bb1[1]),
                    (bb1[2], bb1[1]),
                    (bb1[2], bb1[3]),
                    (bb1[0], bb1[3]))
    bb2_4corners = ((bb2[0], bb2[1]),
                    (bb2[2], bb2[1]),
                    (bb2[2], bb2[3]),
                    (bb2[0], bb2[3]))

    for p in bb1_4corners:
        if is_point_in_rectangle(p, bb2):
            occluded_points += 1
    for p in bb2_4corners:
        if is_point_in_rectangle(p, bb1):
            occluded_points += 1

    return occluded_points

# function does not return correctly if the bounding boxes in count_occluded points are the ones below, still doesn't matter
# o1 = (.0, .0, 1., 1.)
# o2 = (.0, .0, 1.5, .5)
def is_point_in_rectangle(p, r, d=0.):
    return p[0] >= r[0]-d and p[0] <= r[2]+d and p[1] >= r[0]-d and p[1] <= r[3]+d


def bboxes_intercept(bb1, bb2):
    """
    https://rbrundritt.wordpress.com/2009/10/03/determining-if-two-bounding-boxes-overlap/

    :param bb1: first bounding box (x_bottom_left, y_bottom_left, x_top_right, y_top_right)
    :param bb2: second bounding box (x_bottom_left, y_bottom_left, x_top_right, y_top_right)
    :return:
    """

    # First bounding box, top left corner, bottom right corner
    ATLx = bb1[0]
    ATLy = bb1[3]
    ABRx = bb1[2]
    ABRy = bb1[1]

    # Second bounding box, top left corner, bottom right corner
    BTLx = bb2[0]
    BTLy = bb2[3]
    BBRx = bb2[2]
    BBRy = bb2[1]

    rabx = abs(ATLx + ABRx - BTLx - BBRx)
    raby = abs(ATLy + ABRy - BTLy - BBRy)

    # rAx + rBx
    raxPrbx = ABRx - ATLx + BBRx - BTLx

    # rAy + rBy
    rayPrby = ATLy - ABRy + BTLy - BBRy

    if(rabx <= raxPrbx) and (raby <= rayPrby):
        return True, rabx, raxPrbx, raby, rayPrby
    else:
        return False, rabx, raxPrbx, raby, rayPrby


def plot_bbs(bb1, bb2):
    plt.figure()
    ax = plt.gca()
    # ax.invert_yaxis()
    ax.add_patch(Rectangle((bb1[0], bb1[1]), bb1[2]-bb1[0], bb1[3]-bb1[1], alpha=1, facecolor="blue"))
    ax.add_patch(Rectangle((bb2[0], bb2[1]), bb2[2]-bb2[0], bb2[3]-bb2[1], alpha=1, facecolor="red"))
    h = 6
    l = 0
    # ax.set_xlim(l, h)
    # ax.set_ylim(l, h)
    ax.set_xlim(l, h)
    ax.set_ylim(h, l)
    plt.show()


    #     o1 = [Object_State(name="o1", timestamp=0, x=1., y=1., xsize=5., ysize=8.),
    #           Object_State(name="o1", timestamp=1, x=1., y=2., xsize=5., ysize=8.),
    #           Object_State(name="o1", timestamp=2, x=1., y=3., xsize=5., ysize=8.)]
    #
    #     o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., xsize=5., ysize=8.),
    #           Object_State(name="o2", timestamp=1, x=11., y=2., xsize=5., ysize=8.),
    #           Object_State(name="o2", timestamp=2, x=11., y=3., xsize=5., ysize=8.),
    #           Object_State(name="o2", timestamp=3, x=11., y=4., xsize=5., ysize=8.)]
    #
    #     o3 = [Object_State(name="o3", timestamp=0, x=1., y=11., xsize=5., ysize=8.),
    #           Object_State(name="o3", timestamp=1, x=2., y=11., xsize=5., ysize=8.),
    #           Object_State(name="o3", timestamp=2, x=3., y=11., xsize=5., ysize=8.)]

o1 = (2.0, 2.0, 2., 2.)
o2 = (4.0, 3.0, 1., 1.)
o1 = return_bounding_box_2d(o1[0], o1[1], o1[2], o1[3])
o2 = return_bounding_box_2d(o2[0], o2[1], o2[2], o2[3])

print("o1o2:", bboxes_intercept(o1, o2))
print("o2o1:", bboxes_intercept(o2, o1))
# print("o1:", o1)
# print("o2:", o2)
print("o1o2:", compute_qsr(o1, o2))
print("o2o1:", compute_qsr(o2, o1))
plot_bbs(o1, o2)
