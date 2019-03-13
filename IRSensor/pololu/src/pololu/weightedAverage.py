#!/usr/bin/env python
from __future__ import division

class WeightedAverage():

    def __init__(self, numTerms2AverageOver):
        self.beta = 1 - 1/numTerms2AverageOver
        self.avg = None

    def getNewAvg(self, newInput):
        if self.avg == None:
            self.avg = newInput
        else:
            self.avg = self.beta * self.avg + (1-self.beta) * newInput
        return self.avg

if __name__ == "__main__":
    w = WeightedAverage(2)
    for i in range(20):
        print(w.getNewAvg(i))
