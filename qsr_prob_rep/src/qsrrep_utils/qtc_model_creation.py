# -*- coding: utf-8 -*-

from qsrrep_utils.model_creation_abstractclass import ModelCreationAbstractclass, return_numpy_array
import numpy as np


class QTCModelCreation(ModelCreationAbstractclass):

    class qtc_types():
        qtcbs = 'qtcbs'
        qtccs = 'qtccs'
        qtcbcs = 'qtcbcs'
        qtch = 'qtch'
        robot = 'robot'


    __confusion_matrix = np.array([
        [.9, .1, .0, .0],
        [.1, .8, .1, .0],
        [.0, .1, .9, .0],
        [.0, .0, .0, 1.]
    ])
    __states = [-1, 0, 1, np.nan]

    def __create_states(self, qtc_type, start_end=False):
        if qtc_type == self.qtc_types.qtcbs:
            if start_end:
                yield [np.NaN, np.NaN]
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    yield [i-2, j-2]
            if start_end:
                yield [np.NaN, np.NaN]
        elif qtc_type == self.qtc_types.qtccs:
            if start_end:
                yield [np.NaN, np.NaN, np.NaN, np.NaN]
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    for k in xrange(1, 4):
                        for l in xrange(1, 4):
                            yield [i-2, j-2, k-2, l-2]
            if start_end:
                yield [np.NaN, np.NaN, np.NaN, np.NaN]
        elif qtc_type.startswith(self.qtc_types.qtcbcs):
            if start_end:
                yield [np.NaN, np.NaN, np.NaN, np.NaN]
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    yield [i-2, j-2, np.NaN, np.NaN]
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    for k in xrange(1, 4):
                        for l in xrange(1, 4):
                            yield [i-2, j-2, k-2, l-2]
            if start_end:
                yield [np.NaN, np.NaN, np.NaN, np.NaN]
        elif qtc_type == self.qtc_types.qtch:
            if start_end:
                yield [np.NaN, np.NaN, np.NaN, np.NaN]
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    for k in xrange(1, 4):
                        yield [i-2, j-2, k-2, np.NaN]
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    for k in xrange(1, 4):
                        for l in xrange(1, 4):
                            yield [i-2, j-2, k-2, l-2]
            if start_end:
                yield [np.NaN, np.NaN, np.NaN, np.NaN]
        elif qtc_type == self.qtc_types.robot:
            if start_end:
                yield [np.NaN, np.NaN]
            for i in xrange(1, 4):
                yield [i-2, np.NaN]
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    yield [i-2, j-2]
            if start_end:
                yield [np.NaN, np.NaN]
        else:
            print "QTC type: '%s' not found" % qtc_type

    def create_states(self, **kwargs):
        res = [self._create_qtc_string(x) for x in self.__create_states(kwargs["qtc_type"])]
        if kwargs["start_end"]:
            res.reverse()
            res.append("start")
            res.reverse()
            res.append("end")
        return res

    def _create_qtc_string(self, qtc):
        qtc = np.array(qtc)
        qtc = qtc[~np.isnan(qtc)]
        return ','.join(map(str, qtc.astype(int))).replace('-1','-').replace('1','+')

    def create_prediction_model(self, **kwargs):
        raise NotImplementedError("Static prediction model creation for QTC is not available. Please use the trained Markov Model from the HMM.")

    @return_numpy_array
    def create_observation_model(self, **kwargs):
        states = [x for x in self.__create_states(kwargs["qtc_type"], kwargs["start_end"])]
        res = []
        for p in states:
            res.append([])
            for o in states:
                res[-1].append(
                    np.prod([
                        self.__confusion_matrix[self.__states.index(p[y])][self.__states.index(o[y])] \
                            for y in np.arange(0, len(p))
                    ])
                )
            res[-1] = res[-1]/np.sum(res[-1])
        return res
