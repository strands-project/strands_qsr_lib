#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Graphical Visualisation for QSR Relations

:Author: Peter Lightbody <plightbody@lincoln.ac.uk>
:Organization: University of Lincoln
:Date: 12 September 2015
:Version: 0.1
:Status: Development
:Copyright: STRANDS default

"""
import argparse
import matplotlib
import matplotlib.pyplot as plt
import matplotlib as mpl
from random import randint
from pylab import *
import textwrap
import math
from math import sin, cos, radians
from matplotlib.widgets import CheckButtons
import time
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace

class qsr_gui():
    bb1 = None # (x1, y1, x1+w1, y1+h1)
    bb2 = None # (x2, y2, x2+w2, y2+h2)
    qsr = list()
    qsr_type = ("rcc2", "rcc3", "rcc8", "cardir", "argd")

    def __compute_qsr(self, bb1, bb2):
        if not self.qsr:
            return ""
        ax, ay, bx, by = self.bb1
        cx, cy, dx, dy = self.bb2
        
        qsrlib = QSRlib()
        world = World_Trace()
        world.add_object_state_series([Object_State(name="red", timestamp=0, x=((ax+bx)/2.0), y=((ay+by)/2.0), xsize=abs(bx-ax), ysize=abs(by-ay)),
            Object_State(name="yellow", timestamp=0, x=((cx+dx)/2.0), y=((cy+dy)/2.0), xsize=abs(dx-cx), ysize=abs(dy-cy))])
        dynamic_args = {"argd": {"qsr_relations_and_values": self.distance}}
        qsrlib_request_message = QSRlib_Request_Message(which_qsr=self.qsr, input_data=world, dynamic_args=dynamic_args)
        qsrlib_response_message = qsrlib.request_qsrs(req_msg=qsrlib_request_message)

        for t in qsrlib_response_message.qsrs.get_sorted_timestamps():
            foo = ""
            for k, v in zip(qsrlib_response_message.qsrs.trace[t].qsrs.keys(),
                            qsrlib_response_message.qsrs.trace[t].qsrs.values()):
                foo += str(k) + ":" + str(v.qsr) + "; \n"
        return foo

    def randomBB(self):
            x1 = randint(1,10);
            y1 = randint(1,10);
            w1 = randint(1,10);
            h1 = randint(1,10);
            x2 = randint(1,10);
            y2 = randint(1,10);
            w2 = randint(1,10);
            h2 = randint(1,10);
            self.bb1 = (x1, y1, x1+w1, y1+h1)
            self.bb2 = (x2, y2, x2+w2, y2+h2)

    def EventClick(self,label):
        if label in self.qsr:
            self.qsr.remove(label)
        else:
            self.qsr.append(label)
        if not (self.args.placeOne and self.args.placeTwo):
            self.randomBB()
        self.updateWindow()

    def updateWindow(self):
        plt.subplot(2, 2, (1, 2)).clear()
        plt.subplot(2, 2, 3)
        plt.subplot(2, 2, 3).clear()
        plt.axis('off')
        plt.text(1, 1, (self.__compute_qsr(self.bb1, self.bb2)), family='serif', style='italic', ha='center')
        rect1 = matplotlib.patches.Rectangle((self.bb1[0],self.bb1[1]), abs(self.bb1[2]-self.bb1[0]),  abs(self.bb1[1]-self.bb1[3]), color='yellow', alpha=0.5)
        rect2 = matplotlib.patches.Rectangle((self.bb2[0],self.bb2[1]), abs(self.bb2[2]-self.bb2[0]),  abs(self.bb2[1]-self.bb2[3]), color='red', alpha=0.5)
        ax, ay, bx, by = self.bb1
        cx, cy, dx, dy = self.bb2
        plt.subplot(2, 2, (1, 2)).add_patch(rect1)
        plt.subplot(2, 2, (1, 2)).add_patch(rect2)
        self.qsr_specific_reference_gui()
        xlim([min(ax,bx,cx,dx)-15,max(ay,by,cy,dy)+15])
        ylim([min(ax,bx,cx,dx)-15,max(ay,by,cy,dy)+15])
        draw()

    def qsr_specific_reference_gui(self):

        ax, ay, bx, by = self.bb1
        cx, cy, dx, dy = self.bb2
        # Centre of BB1 on the X angle
        AcentreX = ((ax+bx)/2.0)
        # Centre of BB1 on the Y angle
        AcentreY = ((ay+by)/2.0)
        # Centre of BB2 on the X angle
        BcentreX = ((cx+dx)/2.0)
        # Centre of BB2 on the Y angle
        BcentreY = ((cy+dy)/2.0)

        plt.subplot(2, 2, (1, 2))

        if "cardir" in self.qsr:
            # Draws a Line between the centre of bb1 and bb2
            verts = [(AcentreX, AcentreY), (BcentreX, BcentreY)]
            xs, ys = zip(*verts)
            plot(xs, ys, 'x--', lw=1, color='black')

            # Draws the compass shaped guide around bb1 to help identify regions
            for index in range(1,8):
                angle = math.pi/8+(index*math.pi/4)
                distanceBetweenObjects = math.sqrt(math.pow(dx,2) + math.pow(dy,2))
                distance = (16)
                verts = [(((distance * math.cos(angle)) + AcentreX), 
                    ((distance * math.sin(angle)) + AcentreY)), 
                (((distance * math.cos(angle+math.pi)) + AcentreX), 
                    ((distance * math.sin(angle+math.pi))+ AcentreY))]
                xs, ys = zip(*verts)
                plot(xs, ys, 'x--', lw=1, color='green')
                # Add circles around bb1 to identify distance regions
        if "argd" in self.qsr:
            for k in self.distance.keys():
                plt.subplot(2, 2, (1, 2)).add_patch(plt.Circle((AcentreX,AcentreY),self.distance[k], fill=False, color='green'))

    def initWindow(self):
        if not (self.args.placeOne and self.args.placeTwo):
            self.randomBB()
        axes().set_aspect('equal', 'datalim')
        plt.subplot(2, 2, (1, 2))
        plt.subplot(2, 2, (1, 2)).set_aspect('equal')
        subplots_adjust(left=0.31)
        subplots_adjust(bottom=-0.7)
        plt.title('QSR Visualisation')
        axcolor = 'lightgoldenrodyellow'
        rax = plt.axes([0.03, 0.4, 0.22, 0.45], axisbg=axcolor)
        checkBox = CheckButtons(rax, self.qsr_type,(False,False,False,False,False))
        checkBox.on_clicked(self.EventClick)
        plt.subplot(2, 2, 3)
        plt.axis('off')
        plt.text(1, 1, (self.__compute_qsr(self.bb1, self.bb2)), family='serif', style='italic', ha='center')
        if self.qsr:
            self.updateWindow()
        plt.show()

    def processArgs(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("-pOne","--placeOne", help="specify the location of object one", nargs='+', type=int)
        parser.add_argument("-pTwo","--placeTwo", help="specify the location of object two", nargs='+', type=int)
        parser.add_argument("-argd","--distance", help="specify the distances for argd", nargs='+', type=float)
        self.args = parser.parse_args()

        self.bb1 = self.args.placeOne
        self.bb2 = self.args.placeTwo
        self.distance = dict()
        if self.args.distance:
            for x, d in enumerate(self.args.distance):
                self.distance[str(x)] = d
        else:
            self.distance = {"0": 4., "1": 8., "2": 12., "3":16.}

if __name__ == "__main__":
    vis = qsr_gui()
    vis.processArgs()
    vis.initWindow()
