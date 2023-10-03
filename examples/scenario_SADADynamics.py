#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

#
#   SADA Dynamics Scenario
#   Author:             Leah Kiner
#   Creation Date:      Oct 3, 2023
#

import inspect
import os
import matplotlib.pyplot as plt
import numpy as np
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import stepperMotorProfiler
from Basilisk.simulation import spacecraft, spinningBodyTwoDOFStateEffector
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

def run(show_plots, initialMotorAngle, stepsCommanded, stepAngle, stepTime):
    simTaskName = "simTask"
    simProcessName = "simProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testProcessRate = macros.sec2nano(0.1)     # Set process rate update time
    testProc = scSim.CreateNewProcess(simProcessName)
    testProc.addTask(scSim.CreateNewTask(simTaskName, testProcessRate))

    # Create the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # Create an instance of the stepperMotorProfiler module to be tested
    StepperMotorProfiler = stepperMotorProfiler.stepperMotorProfiler()
    StepperMotorProfiler.ModelTag = "StepperMotorProfiler"
    rotAxis_M = np.array([1.0, 0.0, 0.0])
    StepperMotorProfiler.rotAxis_M = rotAxis_M
    StepperMotorProfiler.thetaInit = initialMotorAngle
    StepperMotorProfiler.stepAngle = stepAngle
    StepperMotorProfiler.stepTime = stepTime
    StepperMotorProfiler.thetaDDotMax = stepAngle / (0.25 * stepTime * stepTime)
    StepperMotorProfiler.r_FM_M = np.array([0.0, 0.0, 0.0])
    StepperMotorProfiler.rPrime_FM_M = np.array([0.0, 0.0, 0.0])
    StepperMotorProfiler.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])

    # Add the test module to the runtime call list
    scSim.AddModelToTask(simTaskName, StepperMotorProfiler)

    # Create the StepperMotorProfiler input message
    MotorStepCommandMessageData = messaging.MotorStepCommandMsgPayload()
    MotorStepCommandMessageData.stepsCommanded = stepsCommanded
    MotorStepCommandMessage = messaging.MotorStepCommandMsg().write(MotorStepCommandMessageData)
    StepperMotorProfiler.motorStepCommandInMsg.subscribeTo(MotorStepCommandMessage)

    # Create an instance of the spinningBodyTwoDOF module
    spinningBody = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    spinningBody.ModelTag = "SpinningBody"
    spinningBody.mass1 = 100.0
    spinningBody.mass2 = 50.0
    spinningBody.IS1PntSc1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    spinningBody.IS2PntSc2_S2 = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]
    spinningBody.dcm_S10B = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody.dcm_S20S1 = [[0.0, -1.0, 0.0], [0.0, .0, -1.0], [1.0, 0.0, 0.0]]
    spinningBody.r_Sc1S1_S1 = [[2.0], [-0.5], [0.0]]
    spinningBody.r_Sc2S2_S2 = [[1.0], [0.0], [-1.0]]
    spinningBody.r_S1B_B = [[-2.0], [0.5], [-1.0]]
    spinningBody.r_S2S1_S1 = [[0.5], [-1.5], [-0.5]]
    spinningBody.s1Hat_S1 = [[0], [0], [1]]
    spinningBody.s2Hat_S2 = [[0], [-1], [0]]
    spinningBody.theta1Init = 0 * macros.D2R
    spinningBody.theta2Init = 5 * macros.D2R
    spinningBody.k1 = 1.0
    spinningBody.k2 = 2.0
    if lock1:
        spinningBody.theta1DotInit = 0 * macros.D2R
    else:
        spinningBody.theta1DotInit = 2.0 * macros.D2R
    if lock2:
        spinningBody.theta2DotInit = 0 * macros.D2R
    else:
        spinningBody.theta2DotInit = -1.0 * macros.D2R

    # Connect stepperMotorProfiler output message to the spinningBodyTwoDOF input message
    spinningBody.spinningBodyInMsg.subscribeTo(stepperMotorProfiler.hingedRigidBodyOutMsg)

    # Add spinning body to spacecraft
    scObject.addStateEffector(spinningBody)

    # Log the test module output message for data comparison
    stepperMotorDataLog = StepperMotorProfiler.stepperMotorOutMsg.recorder()
    prescribedDataLog = StepperMotorProfiler.prescribedMotionOutMsg.recorder()
    theta1Data = spinningBody.spinningBodyOutMsgs[0].recorder()
    theta2Data = spinningBody.spinningBodyOutMsgs[1].recorder()
    scSim.AddModelToTask(simTaskName, stepperMotorDataLog)
    scSim.AddModelToTask(simTaskName, prescribedDataLog)
    scSim.AddModelToTask(simTaskName, theta1Data)
    scSim.AddModelToTask(simTaskName, theta2Data)

    # Initialize the simulation, set the sim run time, and execute the simulation
    scSim.InitializeSimulation()
    actuateTime = stepTime * np.abs(stepsCommanded)  # [sec] Time for the motor to actuate to the desired angle
    holdTime = 5  # [sec] Time the simulation will continue while holding the final angle
    scSim.ConfigureStopTime(macros.sec2nano(actuateTime + holdTime))
    scSim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = stepperMotorDataLog.times()
    theta = (180 / np.pi) * stepperMotorDataLog.theta
    thetaDot = (180 / np.pi) * stepperMotorDataLog.thetaDot
    thetaDDot = (180 / np.pi) * stepperMotorDataLog.thetaDDot
    motorStepCount = stepperMotorDataLog.stepCount
    motorCommandedSteps = stepperMotorDataLog.stepsCommanded
    sigma_FM = prescribedDataLog.sigma_FM
    omega_FM_F = prescribedDataLog.omega_FM_F
    omegaPrime_FM_F = prescribedDataLog.omegaPrime_FM_F
    theta1 = (180 / np.pi) * theta1Data.theta
    theta1Dot = (180 / np.pi) * theta1Data.thetaDot
    theta2 = (180 / np.pi) * theta2Data.theta
    theta2Dot = (180 / np.pi) * theta2Data.thetaDot

    # Plot motor angle
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, theta, label=r"$\theta$")
    plt.title(r'Stepper Motor Angle $\theta_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot steps commanded and motor steps taken
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, motorStepCount)
    plt.plot(timespan * macros.NANO2SEC, motorCommandedSteps, '--', label='Commanded')
    plt.title(r'Motor Step History', fontsize=14)
    plt.ylabel('Steps', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot spinning body angles
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, theta1, label=r"Torsional: $\theta_1$")
    plt.plot(timespan * macros.NANO2SEC, theta2, label=r"Bending: $\theta_2$")
    plt.title(r'Solar Array Angles', fontsize=14)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")

#
# This statement below ensures that the unitTestScript can be run as a stand-along python script
#
if __name__ == "__main__":
    run(
        True,
        0.0,                       # initialMotorAngle
        10,                        # stepsCommanded
        1.0 * (np.pi / 180),       # stepAngle
        1.0,                       # stepTime
    )
