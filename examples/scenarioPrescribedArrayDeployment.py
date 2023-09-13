#
#  ISC License

#  Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.

#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

#
# Prescribed Solar Array Deployment Scenario
#
# Purpose:  Simulate the deployment of an n-element solar array
# Author:   Leah Kiner
# Creation Date:  September 13, 2023
#

import os
import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from Basilisk.utilities import RigidBodyKinematics as rbk
import pylab as pl
from matplotlib import collections as mc
import matplotlib
matplotlib.rc('xtick', labelsize=16)
matplotlib.rc('ytick', labelsize=16)
from matplotlib import rcParams


def run():
    deg2Rad = np.pi / 180
    
    numElements = 10
    separationAngle = 360 / numElements

    # Initial scalar state info
    thetaInit = 0.0
    thetaDotInit = 0.0
    thetaDDotInit = 0.0

    # Reference scalar state info
    thetaDotRef = 0.0

    # Create storage arrays
    # r_FM_MData =
    # rPrimeFM_MData =
    # rPrimePrimeFM_MData =
    # sigma_FMData =
    # omega_FM_FData =
    # omegaPrime_FM_FData =
    # omegaPrimePrime_FM_FData =
    thetaDataElements = np.array(thetaInit)
    thetaDotDataElements = np.array(thetaDotInit)
    thetaDDotDataElements = np.array(thetaDDotInit)

    for idx in range(numElements-1):
        thetaDataElements = np.hstack((thetaDataElements, thetaInit))
        thetaDotDataElements = np.hstack((thetaDotDataElements, thetaDotInit))
        thetaDDotDataElements = np.hstack((thetaDDotDataElements, thetaDDotInit))

    # Create local data
    # r_FM_M =
    # rPrime_FM_M =
    # rPrimePrime_FM_M =
    # sigma_FMD =
    # omega_FM_F =
    # omegaPrime_FM_F =
    # omegaPrimePrime_FM_F =
    thetaData = np.array(thetaInit)
    thetaDotData = np.array(thetaDotInit)
    thetaDDotData = np.array(thetaDDotInit)

    # Define temporal parameters
    dt = 0.01  # [s]
    t = 0.0
    maxAngAccel = 0.05  # [deg/s^2]
    stepSimTime = np.sqrt( ((0.5 * np.abs(separationAngle)) * 8) / maxAngAccel )  # [s]
    n_s = int(stepSimTime / dt)
    timespan = np.array(t)

    intermediateInitialAngle = thetaInit
    thetaRef = thetaInit
    counter = 0
    for idx1 in range(numElements):
        thetaRef = thetaInit + ((idx1 + 1) * separationAngle)
        tInit = t

        # Find switch time and final time
        ts = tInit + (0.5 * stepSimTime)
        tf = tInit + stepSimTime

        # Find parabolic constants
        a = (0.5 * separationAngle) / ((ts - tInit) * (ts - tInit))
        b = (-0.5 * separationAngle) / ((ts - tf) * (ts - tf))

        for idx2 in range(n_s):
            # Update current time
            t = t + dt

            if ((t < ts or t == ts) and tf - tInit != 0):
                thetaDDot = maxAngAccel
                thetaDot = thetaDDot * (t - tInit) + thetaDotInit
                theta = a * (t - tInit) * (t - tInit) + intermediateInitialAngle

            elif ( t > ts and t <= tf and tf - tInit != 0):
                thetaDDot = -1 * maxAngAccel
                thetaDot = thetaDDot * (t - tInit) + thetaDotInit - thetaDDot * (tf - tInit)
                theta = b * (t - tf) * (t - tf) + thetaRef
            else:
                thetaDDot = 0.0
                thetaDot = thetaDotRef
                theta = thetaRef

            # Store data
            tempThetaArray = np.array(0.0)
            tempThetaDotArray = np.array(0.0)
            tempThetaDDotArray = np.array(0.0)
            for j in range(numElements):
                if j != 0:
                    if (j < idx1):
                        tempThetaArray = np.hstack((tempThetaArray, thetaDataElements[counter, j]))
                        tempThetaDotArray = np.hstack((tempThetaDotArray, thetaDotDataElements[counter, j]))
                        tempThetaDDotArray = np.hstack((tempThetaDDotArray, thetaDDotDataElements[counter, j]))
                    else:
                        tempThetaArray = np.hstack((tempThetaArray, theta))
                        tempThetaDotArray = np.hstack((tempThetaDotArray, thetaDot))
                        tempThetaDDotArray = np.hstack((tempThetaDDotArray, thetaDDot))
                else:
                    if (j < idx1):
                        tempThetaArray = np.array(thetaDataElements[counter, j])
                        tempThetaDotArray = np.array(thetaDotDataElements[counter, j])
                        tempThetaDDotArray = np.array(thetaDDotDataElements[counter, j])
                    else:
                        tempThetaArray = np.array(theta)
                        tempThetaDotArray = np.array(thetaDot)
                        tempThetaDDotArray = np.array(thetaDDot)

            timespan = np.append(timespan, t)
            thetaData = np.append(thetaData, theta)
            thetaDotData = np.append(thetaDotData, thetaDot)
            thetaDDotData = np.append(thetaDDotData, thetaDDot)
            thetaDataElements = np.vstack((thetaDataElements, tempThetaArray))
            thetaDotDataElements = np.vstack((thetaDotDataElements, tempThetaDotArray))
            thetaDDotDataElements = np.vstack((thetaDDotDataElements, tempThetaDDotArray))

            counter = counter + 1

        # Update reference angle
        intermediateInitialAngle = thetaRef

    # Plot element scalar angles
    plt.figure()
    plt.clf()
    for idx in range(numElements):
        plt.plot(timespan * (1/60), thetaDataElements[:, idx], label=r'Element %i' %(idx + 1))
    plt.title('Array Element Angles', fontsize=14)
    plt.ylabel('(deg)', fontsize=16)
    plt.xlabel('Time (min)', fontsize=16)
    plt.legend(loc='upper left', prop={'size': 12})
    plt.grid(True)

    # Plot element scalar angle rates
    plt.figure()
    plt.clf()
    for idx in range(numElements):
        plt.plot(timespan * (1/60), thetaDotDataElements[:, idx], label=r'Element %i' %(idx + 1))
    plt.title('Array Element Angle Rates', fontsize=14)
    plt.ylabel('(deg/s)', fontsize=16)
    plt.xlabel('Time (min)', fontsize=16)
    plt.legend(loc='upper left', prop={'size': 12})
    plt.grid(True)


if __name__ == "__main__":
    run()
    plt.show()
    plt.close("all")
