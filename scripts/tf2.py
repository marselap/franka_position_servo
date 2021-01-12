#! /usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np
from math import exp, sqrt

SINGLE_REAL = 1
DOUBLE_REAL = 2

class Tf2():

    def __init__(self):

        self.poletype_ = None
        self.T_ = 1.0
        self.nc0_ = 0.0
        self.nc1_ = 0.0
        self.nc2_ = 0.0
        self.dc0_ = 0.0
        self.dc1_ = 0.0
        self.dc2_ = 1.0

        self.n0_ = 0.0
        self.n1_ = 0.0
        self.n2_ = 0.0
        self.d0_ = 0.0
        self.d1_ = 0.0
        self.d2_ = 1.0

        self.x_ = np.zeros((3,1))
        self.y_ = np.zeros((3,1))

        self.numeratorInit_ = False
        self.denominatorInit_ = False

        self.a_ = 0.0
        self.b_ = 0.0

    def setNumerator(self, n0, n1, n2) : #n0 bez s-a
        self.nc0_ = n0
        self.nc1_ = n1
        self.nc2_ = n2

        self.numeratorInit_ = True

        return self.numeratorInit_


    def setDenominator(self, d0, d1, d2):

        self.dc0_ = d0
        self.dc1_ = d1
        self.dc2_ = d2

        if (self.dc2_ == 0.0):
            self.denominatorInit_ = False
            return self.denominatorInit_

        discriminant = self.dc1_*self.dc1_ - 4.*self.dc0_*self.dc2_

        if ((abs(discriminant)*100000) > 1.0):
            discriminant = 0.0 

        if (discriminant < 0):
            self.denominatorInit_ = False
        else:
            self.a_ = (self.dc1_ + sqrt(discriminant))/(2*self.dc2_)
            self.b_ = (self.dc1_ - sqrt(discriminant))/(2*self.dc2_)

            if (discriminant == 0.0): 
                self.poletype_ = DOUBLE_REAL
            else:
                self.poletype_ = SINGLE_REAL

            self.denominatorInit_ = True
        
        return self.denominatorInit_


    def c2d(self, samplingTime, method):
        if (samplingTime > 0.0): 
            self.T_ = samplingTime
        else:
            return False

        if (method in "zoh"):
            return self.zohTransform()
        elif (method in "tustin"):
            return self.tustinTransform()
        else:
            return False
        

    def zohTransform(self):

        if (self.numeratorInit_ and self.denominatorInit_):
            if (self.poletype_ == SINGLE_REAL):
                self.n0_ = self.nc0_ * (self.b_ * exp(-self.b_ * self.T_) - self.a_ * exp(-self.a_ * self.T_) + (self.a_ - self.b_) * exp(-self.T_ * self.dc1_ / self.dc2_)) / (self.dc0_ * (self.a_ - self.b_)) + self.nc1_ * (- (exp(-self.a_ * self.T_) - exp(-self.b_ * self.T_))) / (self.dc2_ * (self.b_ - self.a_)) + self.nc2_ * ((self.a_ / (self.a_ - self.b_)) * exp(-self.b_ * self.T_) + (self.b_ / (self.b_ - self.a_)) * exp(-self.a_ * self.T_)) / (self.dc2_)
                self.n1_ = self.nc0_ * (self.a_ * (1.0 + exp(-self.a_ * self.T_)) - self.b_ * (1.0 + exp(-self.b_ * self.T_)) - (self.a_ - self.b_) * (exp(-self.b_ * self.T_) + exp(-self.a_ * self.T_))) / (self.dc0_ * (self.a_ - self.b_)) + self.nc1_ * (exp(-self.a_ * self.T_) - exp(-self.b_ * self.T_)) / (self.dc2_ * (self.b_ - self.a_)) + self.nc2_ * (- (1.0 + (self.a_ / (self.a_ - self.b_)) * exp(-self.b_ * self.T_) + (self.b_ / (self.b_ - self.a_)) * exp(-self.a_ * self.T_))) / (self.dc2_)
                self.n2_ = self.nc2_/self.dc2_
                self.d0_ = exp(-self.T_ * self.dc1_ / self.dc2_)
                self.d1_ = -(exp(-self.b_ * self.T_) + exp(-self.a_ * self.T_))
                self.d2_ = 1.0
                return True
            elif (self.poletype_ == DOUBLE_REAL):
                self.n0_ = self.nc0_ * (exp(-self.T_ * self.a_) * (self.T_ * self.a_ - 1.0 + exp(-self.T_*self.a_))) / (self.dc2_ * self.a_ * self.a_) + self.nc1_ * ( - self.T_ * exp(-self.a_ * self.T_)) / (self.dc2_) + self.nc2_ * ((self.T_ * self.a_) * exp(-self.T_ * self.a_)) / (self.dc2_)
                self.n1_ = self.nc0_ * (1.0 - exp(-self.T_ * self.a_) * (1.0 + self.T_ * self.a_)) / (self.dc2_ * self.a_ * self.a_) + self.nc1_ * (self.T_ * exp(-self.a_ * self.T_)) / (self.dc2_) + self.nc2_ * (- (1.0 + (self.T_ * self.a_ + 1) * exp(-self.T_ * self.a_))) / (self.dc2_)
                self.n2_ = self.nc2_/self.dc2_
                self.d0_ = exp(-2.0 * self.T_ * self.a_)
                self.d1_ = -2.0*exp(-self.a_ * self.T_)
                self.d2_ = 1.0
                return True
        return False

    def tustinTransform(self):
        if (self.numeratorInit_ and self.denominatorInit_):
            self.n0_ = self.nc0_ * self.T_ * self.T_ - 2.0 * self.nc1_ * self.T_ + 4.0 * self.nc2_
            self.n1_ = (2.0 * self.nc0_ * self.T_ * self.T_ - 8.0 * self.nc2_)
            self.n2_ = (self.nc0_ * self.T_ * self.T_ + 2.0 * self.nc1_ * self.T_ + 4.0 * self.nc2_)
            self.d0_ = self.dc0_ * self.T_ * self.T_ - 2.0 * self.dc1_ * self.T_ + 4.0 * self.dc2_
            self.d1_ = (2.0 * self.dc0_ * self.T_ * self.T_ - 8.0 * self.dc2_)
            self.d2_ = (self.dc0_ * self.T_ * self.T_ + 2.0 * self.dc1_ * self.T_ + 4.0 * self.dc2_)

            # print self.n0_
            # print self.n1_
            # print self.n2_

            # print("d")
            # print self.d0_
            # print self.d1_
            # print self.d2_

            return True
        return False

    def setInitialValues(self, y0, x0):
        for i in range(3):
            self.x_[i] = x0[i]
            self.y_[i] = y0[i]

    def getDiscreteDenominator(self):
        return self.d0_, self.d1_, self.d2_


    def getDiscreteNumerator(self):
        return self.n0_, self.n1_, self.n2_


    def reset(self):
        for i in range(3):
            self.x_[i] = 0
            self.y_[i] = 0

    def getDiscreteOutput(self, input):

        # for i in range(2,0,-1):
        #     self.x_[i] = self.x_[i-1]
        #     self.y_[i] = self.y_[i-1]

        self.x_[2] = self.x_[1]
        self.x_[1] = self.x_[0]
        self.y_[2] = self.y_[1]
        self.y_[1] = self.y_[0]

        self.x_[0] = input

        # print(self.y_[0])

        self.y_[0] = (self.n2_/self.d2_)*self.x_[0] + (self.n1_/self.d2_)*self.x_[1] + (self.n0_/self.d2_)*self.x_[2] - (self.d1_/self.d2_)*self.y_[1] - (self.d0_/self.d2_)*self.y_[2]

        return self.y_[0]

import matplotlib.pyplot as plt

if __name__ == "__main__":

    a = Tf2()

    a.setNumerator(1.,0.,0.)
    a.setDenominator(2.5, 10., 10.)
    init = [0.,0.,0.]
    a.c2d(0.1, "tustin")
    a.setInitialValues(init, init)

    i = 0
    outputs = []
    while i < 100:
        y = a.getDiscreteOutput(1.)
        outputs.append(y[0])
        i+=1

    fig = plt.figure()
    plt.plot(outputs)
    plt.show()

