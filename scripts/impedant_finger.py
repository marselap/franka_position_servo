#! /usr/bin/env python
import rospy
import numpy as np

from tf2 import Tf2

class ImpedantController():

    def __init__(self, rate):

        self.m_ = 0.0
        self.b_ = 0.0
        self.k_ = 0.0

        self.dead_zone_ = 0.0
        self.rate_ = rate
        self.Xc_ = np.zeros((3,1))

        self.Ge_ = [Tf2(), Tf2(), Tf2()]
        self.Gxr_ = [Tf2(), Tf2(), Tf2()]
        self.Gvr_ = [Tf2(), Tf2(), Tf2()]
        self.Gar_ = [Tf2(), Tf2(), Tf2()]

    def setImpedanceFilterMass(self, mass):
        self.m_ = mass

    def setImpedanceFilterDamping(self, damping):
        self.b_ = damping

    def setImpedanceFilterStiffness(self, stiffness):
        self.k_ = stiffness


    def setImpedanceFilterInitialValue(self, initial_values):

        y0 = [initial_values, initial_values, initial_values]
        x0 = [initial_values, initial_values, initial_values]

        self.Gxr_[0].setInitialValues(y0, x0)

        y0 = [0.0, 0.0, 0.0]
        x0 = [0.0, 0.0, 0.0]

        self.Gxr_[1].setInitialValues(y0, x0)
        self.Gxr_[2].setInitialValues(y0, x0)

        for i in range(3):
            self.Gvr_[i].setInitialValues(y0, x0)
            self.Gar_[i].setInitialValues(y0, x0)


    def initializeImpedanceFilterTransferFunction(self):

        samplingTime = 1.0/self.rate_

        #error transfer function
        self.Ge_[0].reset()
        self.Ge_[0].setNumerator(1.0, 0.0, 0.0)
        self.Ge_[0].setDenominator(self.k_, self.b_, self.m_)
        self.Ge_[0].c2d(samplingTime, "tustin")

        self.Ge_[1].reset()
        self.Ge_[1].setNumerator(0.0, 1.0, 0.0)
        self.Ge_[1].setDenominator(self.k_, self.b_, self.m_)
        self.Ge_[1].c2d(samplingTime, "tustin")

        self.Ge_[2].reset()
        self.Ge_[2].setNumerator(0.0, 0.0, 1.0)
        self.Ge_[2].setDenominator(self.k_, self.b_, self.m_)
        self.Ge_[2].c2d(samplingTime, "tustin")

        #Position transfer function
        self.Gxr_[0].reset()
        self.Gxr_[0].setNumerator(self.k_, 0.0, 0.0)
        self.Gxr_[0].setDenominator(self.k_, self.b_, self.m_)
        self.Gxr_[0].c2d(samplingTime, "tustin")

        self.Gxr_[1].reset()
        self.Gxr_[1].setNumerator(0.0, self.k_, 0.0)
        self.Gxr_[1].setDenominator(self.k_, self.b_, self.m_)
        self.Gxr_[1].c2d(samplingTime, "tustin")

        self.Gxr_[2].reset()
        self.Gxr_[2].setNumerator(0.0, 0.0, self.k_)
        self.Gxr_[2].setDenominator(self.k_, self.b_, self.m_)
        self.Gxr_[2].c2d(samplingTime, "tustin")

        #velocity transfer function
        self.Gvr_[0].reset()
        self.Gvr_[0].setNumerator(self.b_, 0.0, 0.0)
        self.Gvr_[0].setDenominator(self.k_, self.b_, self.m_)
        self.Gvr_[0].c2d(samplingTime, "tustin")

        self.Gvr_[1].reset()
        self.Gvr_[1].setNumerator(0.0, self.b_, 0.0)
        self.Gvr_[1].setDenominator(self.k_, self.b_, self.m_)
        self.Gvr_[1].c2d(samplingTime, "tustin")

        self.Gvr_[2].reset()
        self.Gvr_[2].setNumerator(0.0, 0.0, self.b_)
        self.Gvr_[2].setDenominator(self.k_, self.b_, self.m_)
        self.Gvr_[2].c2d(samplingTime, "tustin")

        #acceleration transfer function
        self.Gar_[0].reset()
        self.Gar_[0].setNumerator(self.m_, 0.0, 0.0)
        self.Gar_[0].setDenominator(self.k_, self.b_, self.m_)
        self.Gar_[0].c2d(samplingTime, "tustin")

        self.Gar_[1].reset()
        self.Gar_[1].setNumerator(0.0, self.m_, 0.0)
        self.Gar_[1].setDenominator(self.k_, self.b_, self.m_)
        self.Gar_[1].c2d(samplingTime, "tustin")

        self.Gar_[2].reset()
        self.Gar_[2].setNumerator(0.0, 0.0, self.m_)
        self.Gar_[2].setDenominator(self.k_, self.b_, self.m_)
        self.Gar_[2].c2d(samplingTime, "tustin")


    def impedanceFilter(self, f, fd, Xr) :
        e = fd - f

        self.Xc_[0] = self.Ge_[0].getDiscreteOutput(deadZone(e, self.dead_zone_)) + self.Gxr_[0].getDiscreteOutput(Xr[0]) \
                + self.Gvr_[0].getDiscreteOutput(Xr[1]) + self.Gar_[0].getDiscreteOutput(Xr[2])

        self.Xc_[1] = self.Ge_[1].getDiscreteOutput(deadZone(e, self.dead_zone_)) + self.Gxr_[1].getDiscreteOutput(Xr[0]) \
                + self.Gvr_[1].getDiscreteOutput(Xr[1]) + self.Gar_[1].getDiscreteOutput(Xr[2])

        self.Xc_[2] = self.Ge_[2].getDiscreteOutput(deadZone(e, self.dead_zone_)) + self.Gxr_[2].getDiscreteOutput(Xr[0]) \
                + self.Gvr_[2].getDiscreteOutput(Xr[1]) + self.Gar_[2].getDiscreteOutput(Xr[2])

        return self.Xc_


    def setDeadZone(self, dead_zone) :
        self.dead_zone_ = dead_zone


def deadZone(data, limit):

    if (abs(data) < limit) :
        temp = 0.0
    elif (data > 0.0):
        temp = data - limit
    else:
        temp = data + limit

    return temp


if __name__ == "__main__":

    # a = ImpedantController(10.)

    # a.setImpedanceFilterMass(2.5)
    # a.setImpedanceFilterDamping(10.)
    # a.setImpedanceFilterStiffness(10.)
    
    # a.initializeImpedanceFilterTransferFunction()

    # a.setImpedanceFilterInitialValue([0., 0., 0.])


    # xc = a.impedanceFilter(force_meas, force_ref, xc)



    rospy.init_node("impedance")

    rate_hz = 1.
    rate = rospy.Rate(rate_hz)

    a = ImpedantController(rate_hz)
    m = 10.
    b = 10.
    k = 2.5
    a.setImpedanceFilterMass(m)
    a.setImpedanceFilterDamping(b)
    a.setImpedanceFilterStiffness(k)
    a.initializeImpedanceFilterTransferFunction()
    a.setImpedanceFilterInitialValue(0.0)

    while not rospy.is_shutdown():

        rate.sleep()

        xc = a.impedanceFilter(10., 0., [0., 0., 0.])
        print(np.transpose(xc))

