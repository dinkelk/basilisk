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
    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.01], [-0.01], [0.01]]

    # Add the sc to the runtime call list
    scSim.AddModelToTask(simTaskName, scObject)

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
    spinningBody.mass1 = 0.0  # lower body
    spinningBody.mass2 = 100.0
    spinningBody.IS1PntSc1_S1 = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    spinningBody.IS2PntSc2_S2 = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]
    spinningBody.dcm_S10B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody.r_Sc1S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody.r_Sc2S2_S2 = [[1.0], [0.0], [0.0]]
    spinningBody.r_S1B_B = [[-2.0], [0.5], [-1.0]]
    spinningBody.r_S2S1_S1 = [[0.0], [0.0], [0.0]]
    spinningBody.s1Hat_S1 = [[1.0], [0], [0.0]]  # torsional axis
    spinningBody.s2Hat_S2 = [[0.0], [1.0], [0.0]]  # bending axis
    spinningBody.theta1Init = initialMotorAngle  # initial torsional angle
    spinningBody.theta2Init = 0.01  # should be very small, bending

    # Define spring and damper terms
    omega_n_1 = 2 * np.pi * 0.5  # natural freq
    omega_n_2 = 2 * np.pi * 0.6  # natural freq
    Q = 30.0
    spinningBody.k1 = (omega_n_1**2) * spinningBody.IS2PntSc2_S2[0][0]
    spinningBody.k2 = (omega_n_2**2) * spinningBody.IS2PntSc2_S2[1][1]
    spinningBody.c1 = omega_n_1 / Q
    spinningBody.c2 = omega_n_2 / Q

    # Connect stepperMotorProfiler output message to the spinningBodyTwoDOF input message
    spinningBody.spinningBodyRefInMsgs[0].subscribeTo(StepperMotorProfiler.hingedRigidBodyOutMsg)

    # Add spinning body to spacecraft
    scObject.addStateEffector(spinningBody)

    # Log the test module output message for data comparison
    stepperMotorDataLog = StepperMotorProfiler.stepperMotorOutMsg.recorder()
    theta1Data = spinningBody.spinningBodyOutMsgs[0].recorder()
    theta2Data = spinningBody.spinningBodyOutMsgs[1].recorder()
    scSim.AddModelToTask(simTaskName, stepperMotorDataLog)
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
