#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import math

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (hillPoint, inertial3D, attTrackingError, mrpFeedback,
                                    rwMotorTorque,
                                    velocityPoint, mrpSteering, rateServoFullNonlinear,
                                    sunSafePoint, cssWlsEst, lambertPlanner, lambertSolver, lambertValidator,
                                    lambertSurfaceRelativeVelocity, lambertSecondDV,
                                    dvGuidance, thrForceMapping, thrFiringRemainder,
                                    attRefCorrection, dvExecuteGuidance)
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import (fswSetupRW, fswSetupThrusters)
from Basilisk.utilities import deprecated
from Basilisk.utilities import macros as mc


class BSKFswModels:
    """Defines the bskSim FSW class"""
    def __init__(self, SimBase, fswRate):
        # define empty class variables
        self.vcMsg = None
        self.fswRwConfigMsg = None
        self.cmdTorqueMsg = None
        self.cmdTorqueDirectMsg = None
        self.attRefMsg = None
        self.attGuidMsg = None
        self.cmdRwMotorMsg = None
        self.dvBurnCmdMsg = None
        self.acsOnTimeCmdMsg = None
        self.dvOnTimeCmdMsg = None

        # by default, use ACS thrusters if attitude is controlled with thrusters
        self.useDvThrusters = False

        # Define process name and default time-step for all FSW tasks defined later on
        self.processName = SimBase.FSWProcessName
        self.processTasksTimeStep = mc.sec2nano(fswRate)

        # Create module data and module wraps
        self.inertial3D = inertial3D.inertial3D()
        self.inertial3D.ModelTag = "inertial3D"

        self.hillPoint = hillPoint.hillPoint()
        self.hillPoint.ModelTag = "hillPoint"

        self.sunSafePoint = sunSafePoint.sunSafePoint()
        self.sunSafePoint.ModelTag = "sunSafePoint"

        self.velocityPoint = velocityPoint.velocityPoint()
        self.velocityPoint.ModelTag  = "velocityPoint"

        self.dvPoint = dvGuidance.dvGuidance()
        self.dvPoint.ModelTag = "dvPoint"

        self.cssWlsEst = cssWlsEst.cssWlsEst()
        self.cssWlsEst.ModelTag = "cssWlsEst"

        self.attRefCorrection = attRefCorrection.attRefCorrection()
        self.attRefCorrection.ModelTag = "attRefCorrection"

        self.trackingError = attTrackingError.attTrackingError()
        self.trackingError.ModelTag = "trackingError"

        self.mrpFeedbackControl = mrpFeedback.mrpFeedback()
        self.mrpFeedbackControl.ModelTag = "mrpFeedbackControl"

        self.mrpFeedbackRWs = mrpFeedback.mrpFeedback()
        self.mrpFeedbackRWs.ModelTag = "mrpFeedbackRWs"

        self.mrpFeedbackTHs = mrpFeedback.mrpFeedback()
        self.mrpFeedbackTHs.ModelTag = "mrpFeedbackTHs"

        self.mrpSteering = mrpSteering.mrpSteering()
        self.mrpSteering.ModelTag = "MRP_Steering"

        self.rateServo = rateServoFullNonlinear.rateServoFullNonlinear()
        self.rateServo.ModelTag = "rate_servo"

        self.rwMotorTorque = rwMotorTorque.rwMotorTorque()
        self.rwMotorTorque.ModelTag = "rwMotorTorque"

        self.thrForceMapping = thrForceMapping.thrForceMapping()
        self.thrForceMapping.ModelTag = "thrForceMapping"

        self.thrFiringRemainder = thrFiringRemainder.thrFiringRemainder()
        self.thrFiringRemainder.ModelTag = "thrFiringRemainder"

        self.dvManeuver = dvExecuteGuidance.dvExecuteGuidance()
        self.dvManeuver.ModelTag = "dvManeuver"

        self.lambertPlannerObject = lambertPlanner.LambertPlanner()
        self.lambertPlannerObject.ModelTag = "LambertPlanner"

        self.lambertSolverObject = lambertSolver.LambertSolver()
        self.lambertSolverObject.ModelTag = "LambertSolver"

        self.lambertValidatorObject = lambertValidator.LambertValidator()
        self.lambertValidatorObject.ModelTag = "LambertValidator"

        self.lambertSurfaceRelativeVelocityObject = lambertSurfaceRelativeVelocity.LambertSurfaceRelativeVelocity()
        self.lambertSurfaceRelativeVelocityObject.ModelTag = "LambertSurfaceRelativeVelocity"

        self.lambertSecondDvObject = lambertSecondDV.LambertSecondDV()
        self.lambertSecondDvObject.ModelTag = "LambertSecondDV"

        # create the FSW module gateway messages
        self.setupGatewayMsgs(SimBase)

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Create tasks
        SimBase.fswProc.addTask(SimBase.CreateNewTask("inertial3DPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("hillPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("sunSafePointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("velocityPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("dvPointTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackTask", self.processTasksTimeStep), 10)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpSteeringRWsTask", self.processTasksTimeStep), 10)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask", self.processTasksTimeStep), 10)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("mrpFeedbackTHsTask", self.processTasksTimeStep), 10)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("dvBurnTask", int(self.processTasksTimeStep/2)), 9)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("lambertGuidanceFirstDV", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("lambertGuidanceSecondDV", self.processTasksTimeStep), 20)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask("inertial3DPointTask", self.inertial3D, 10)
        SimBase.AddModelToTask("inertial3DPointTask", self.trackingError, 9)

        SimBase.AddModelToTask("hillPointTask", self.hillPoint, 10)
        SimBase.AddModelToTask("hillPointTask", self.trackingError, 9)

        SimBase.AddModelToTask("sunSafePointTask", self.cssWlsEst, 10)
        SimBase.AddModelToTask("sunSafePointTask", self.sunSafePoint, 9)

        SimBase.AddModelToTask("velocityPointTask", self.velocityPoint, 10)
        SimBase.AddModelToTask("velocityPointTask", self.trackingError, 9)

        SimBase.AddModelToTask("dvPointTask", self.dvPoint, 10)
        SimBase.AddModelToTask("dvPointTask", self.attRefCorrection, 9)
        SimBase.AddModelToTask("dvPointTask", self.trackingError, 8)

        SimBase.AddModelToTask("mrpFeedbackTask", self.mrpFeedbackControl, 10)

        SimBase.AddModelToTask("mrpSteeringRWsTask", self.mrpSteering, 10)
        SimBase.AddModelToTask("mrpSteeringRWsTask", self.rateServo, 9)
        SimBase.AddModelToTask("mrpSteeringRWsTask", self.rwMotorTorque, 8)

        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.mrpFeedbackRWs, 9)
        SimBase.AddModelToTask("mrpFeedbackRWsTask", self.rwMotorTorque, 8)

        SimBase.AddModelToTask("mrpFeedbackTHsTask", self.mrpFeedbackTHs, 9)
        SimBase.AddModelToTask("mrpFeedbackTHsTask", self.thrForceMapping, 8)
        SimBase.AddModelToTask("mrpFeedbackTHsTask", self.thrFiringRemainder, 7)

        SimBase.AddModelToTask("dvBurnTask", self.dvManeuver, 10)

        SimBase.AddModelToTask("lambertGuidanceFirstDV", self.lambertPlannerObject, None, 10)
        SimBase.AddModelToTask("lambertGuidanceFirstDV", self.lambertSolverObject, None, 9)
        SimBase.AddModelToTask("lambertGuidanceFirstDV", self.lambertValidatorObject, None, 8)

        SimBase.AddModelToTask("lambertGuidanceSecondDV", self.lambertSurfaceRelativeVelocityObject, None, 10)
        SimBase.AddModelToTask("lambertGuidanceSecondDV", self.lambertSecondDvObject, None, 9)

        # Create events to be called for triggering GN&C maneuvers
        SimBase.fswProc.disableAllTasks()

        SimBase.createNewEvent("initiateStandby", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'standby'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.setAllButCurrentEventActivity('initiateStandby', True)"
                                ])

        SimBase.createNewEvent("initiateAttitudeGuidance", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'inertial3D'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('inertial3DPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateAttitudeGuidance', True)"
                                ])

        SimBase.createNewEvent("initiateAttitudeGuidanceDirect", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'directInertial3D'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('inertial3DPointTask')",
                                "self.enableTask('mrpFeedbackTask')",
                                "self.setAllButCurrentEventActivity('initiateAttitudeGuidanceDirect', True)"
                                ])

        SimBase.createNewEvent("initiateHillPoint", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'hillPoint'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('hillPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateHillPoint', True)"
                                ])

        SimBase.createNewEvent("initiateSunSafePoint", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'sunSafePoint'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('sunSafePointTask')",
                                "self.enableTask('mrpSteeringRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateSunSafePoint', True)"
                                ])

        SimBase.createNewEvent("initiateVelocityPoint", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'velocityPoint'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('velocityPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateVelocityPoint', True)"])

        SimBase.createNewEvent("initiateSteeringRW", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'steeringRW'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('hillPointTask')",
                                "self.enableTask('mrpSteeringRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateSteeringRW', True)"])

        SimBase.createNewEvent("initiateLambertGuidanceFirstDV", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'lambertFirstDV'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('hillPointTask')",
                                "self.enableTask('mrpSteeringRWsTask')",
                                "self.enableTask('lambertGuidanceFirstDV')",
                                "self.setAllButCurrentEventActivity('initiateLambertGuidanceFirstDV', True)"])

        SimBase.createNewEvent("initiateLambertGuidanceSecondDV", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'lambertSecondDV'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.FSWModels.zeroGateWayMsgs()",
                                "self.enableTask('hillPointTask')",
                                "self.enableTask('mrpSteeringRWsTask')",
                                "self.enableTask('lambertGuidanceSecondDV')",
                                "self.setAllButCurrentEventActivity('initiateLambertGuidanceSecondDV', True)"])

        SimBase.createNewEvent("initiateDvPoint", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'dvPoint'"],
                               ["self.fswProc.disableAllTasks()",
                                "self.enableTask('dvPointTask')",
                                "self.enableTask('mrpFeedbackRWsTask')",
                                "self.setAllButCurrentEventActivity('initiateDvPoint', True)"
                                ])

        SimBase.createNewEvent("initiateDvBurn", self.processTasksTimeStep, True,
                               ["self.modeRequest == 'dvBurn'"],
                               ["self.fswProc.disableAllTasks()",
                                "from Basilisk.architecture import messaging",
                                "self.FSWModels.cmdRwMotorMsg.write(messaging.ArrayMotorTorqueMsgPayload())",
                                "self.enableTask('dvPointTask')",
                                "self.enableTask('mrpFeedbackTHsTask')",
                                "self.enableTask('dvBurnTask')",
                                "self.setAllButCurrentEventActivity('initiateDvBurn', True)"
                                ])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetInertial3DPointGuidance(self):
        """Define the inertial 3D guidance module"""
        self.inertial3D.sigma_R0N = [0.2, 0.4, 0.6]
        messaging.AttRefMsg_C_addAuthor(self.inertial3D.attRefOutMsg, self.attRefMsg)

    def SetHillPointGuidance(self, SimBase):
        """Define the Hill pointing guidance module"""
        self.hillPoint.transNavInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.transOutMsg)
        self.hillPoint.celBodyInMsg.subscribeTo(SimBase.DynModels.EarthEphemObject.ephemOutMsgs[0])  # earth
        messaging.AttRefMsg_C_addAuthor(self.hillPoint.attRefOutMsg, self.attRefMsg)

    def SetSunSafePointGuidance(self, SimBase):
        """Define the sun safe pointing guidance module"""
        self.sunSafePoint.imuInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)
        self.sunSafePoint.sunDirectionInMsg.subscribeTo(self.cssWlsEst.navStateOutMsg)
        self.sunSafePoint.sHatBdyCmd = [0.0, 0.0, 1.0]
        messaging.AttGuidMsg_C_addAuthor(self.sunSafePoint.attGuidanceOutMsg, self.attGuidMsg)

    def SetVelocityPointGuidance(self, SimBase):
        """Define the velocity pointing guidance module"""
        self.velocityPoint.transNavInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.transOutMsg)
        self.velocityPoint.celBodyInMsg.subscribeTo(SimBase.DynModels.EarthEphemObject.ephemOutMsgs[0])
        self.velocityPoint.mu = SimBase.DynModels.gravFactory.gravBodies['earth'].mu
        messaging.AttRefMsg_C_addAuthor(self.velocityPoint.attRefOutMsg, self.attRefMsg)

    def SetDvPointGuidance(self):
        """Define the Delta-V pointing guidance module"""
        self.dvPoint.burnDataInMsg.subscribeTo(self.dvBurnCmdMsg)
        messaging.AttRefMsg_C_addAuthor(self.dvPoint.attRefOutMsg, self.attRefMsg)

    def SetAttRefCorrection(self):
        """Define the attitude reference correction module"""
        self.attRefCorrection.sigma_BcB = [0., 0., 0.]
        self.attRefCorrection.attRefInMsg.subscribeTo(self.attRefMsg)
        messaging.AttRefMsg_C_addAuthor(self.attRefCorrection.attRefOutMsg, self.attRefMsg)

    def SetAttitudeTrackingError(self, SimBase):
        """Define the attitude tracking error module"""
        self.trackingError.attNavInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)
        self.trackingError.attRefInMsg.subscribeTo(self.attRefMsg)
        messaging.AttGuidMsg_C_addAuthor(self.trackingError.attGuidOutMsg, self.attGuidMsg)

    def SetCSSWlsEst(self, SimBase):
        """Set the FSW CSS configuration information """
        cssConfig = messaging.CSSConfigMsgPayload()
        totalCSSList = []
        nHat_B_vec = [
            [0.0, 0.707107, 0.707107],
            [0.707107, 0., 0.707107],
            [0.0, -0.707107, 0.707107],
            [-0.707107, 0., 0.707107],
            [0.0, -0.965926, -0.258819],
            [-0.707107, -0.353553, -0.612372],
            [0., 0.258819, -0.965926],
            [0.707107, -0.353553, -0.612372]
        ]
        for CSSHat in nHat_B_vec:
            CSSConfigElement = messaging.CSSUnitConfigMsgPayload()
            CSSConfigElement.CBias = 1.0
            CSSConfigElement.nHat_B = CSSHat
            totalCSSList.append(CSSConfigElement)
        cssConfig.cssVals = totalCSSList

        cssConfig.nCSS = len(nHat_B_vec)
        self.cssConfigMsg = messaging.CSSConfigMsg().write(cssConfig)

        self.cssWlsEst.cssDataInMsg.subscribeTo(SimBase.DynModels.CSSConstellationObject.constellationOutMsg)
        self.cssWlsEst.cssConfigInMsg.subscribeTo(self.cssConfigMsg)

    def SetMRPFeedbackControl(self, SimBase):
        """Set the MRP feedback module configuration"""
        self.mrpFeedbackControl.guidInMsg.subscribeTo(self.attGuidMsg)
        self.mrpFeedbackControl.vehConfigInMsg.subscribeTo(self.vcMsg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.mrpFeedbackControl.cmdTorqueOutMsg, self.cmdTorqueDirectMsg)

        self.mrpFeedbackControl.K = 3.5
        self.mrpFeedbackControl.Ki = -1.0  # Note: make value negative to turn off integral feedback
        self.mrpFeedbackControl.P = 30.0
        self.mrpFeedbackControl.integralLimit = 2. / self.mrpFeedbackControl.Ki * 0.1

    def SetMRPFeedbackRWA(self, SimBase):
        """Set the MRP feedback information if RWs are considered"""
        self.mrpFeedbackRWs.K = 3.5
        self.mrpFeedbackRWs.Ki = -1  # Note: make value negative to turn off integral feedback
        self.mrpFeedbackRWs.P = 30.0
        self.mrpFeedbackRWs.integralLimit = 2. / self.mrpFeedbackRWs.Ki * 0.1

        self.mrpFeedbackRWs.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.mrpFeedbackRWs.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector.rwSpeedOutMsg)
        self.mrpFeedbackRWs.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mrpFeedbackRWs.guidInMsg.subscribeTo(self.attGuidMsg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.mrpFeedbackRWs.cmdTorqueOutMsg, self.cmdTorqueMsg)

    def SetMRPFeedbackTH(self, SimBase):
        """Set the MRP feedback information if Thrusters are considered"""
        self.mrpFeedbackTHs.K = 3.5*10
        self.mrpFeedbackTHs.Ki = 0.0002  # Note: make value negative to turn off integral feedback
        self.mrpFeedbackTHs.P = 30.0*10
        self.mrpFeedbackTHs.integralLimit = 2. / self.mrpFeedbackTHs.Ki * 0.1

        self.mrpFeedbackTHs.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.mrpFeedbackTHs.guidInMsg.subscribeTo(self.attGuidMsg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.mrpFeedbackTHs.cmdTorqueOutMsg, self.cmdTorqueMsg)

    def SetMRPSteering(self):
        """Set the MRP Steering module"""
        self.mrpSteering.K1 = 0.05
        self.mrpSteering.ignoreOuterLoopFeedforward = False
        self.mrpSteering.K3 = 0.75
        self.mrpSteering.omega_max = 1.0 * mc.D2R
        self.mrpSteering.guidInMsg.subscribeTo(self.attGuidMsg)

    def SetRateServo(self, SimBase):
        """Set the rate servo module"""
        self.rateServo.guidInMsg.subscribeTo(self.attGuidMsg)
        self.rateServo.vehConfigInMsg.subscribeTo(self.vcMsg)
        self.rateServo.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.rateServo.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector.rwSpeedOutMsg)
        self.rateServo.rateSteeringInMsg.subscribeTo(self.mrpSteering.rateCmdOutMsg)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.rateServo.cmdTorqueOutMsg, self.cmdTorqueMsg)

        self.rateServo.Ki = 5.0
        self.rateServo.P = 150.0
        self.rateServo.integralLimit = 2. / self.rateServo.Ki * 0.1
        self.rateServo.knownTorquePntB_B = [0., 0., 0.]

    def SetVehicleConfiguration(self):
        """Set the spacecraft configuration information"""
        vehicleConfigOut = messaging.VehicleConfigMsgPayload()
        # use the same inertia in the FSW algorithm as in the simulation
        vehicleConfigOut.ISCPntB_B = [900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]
        self.vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    def SetRWConfigMsg(self):
        """Set the RW device information"""
        # Configure RW pyramid exactly as it is in the Dynamics (i.e. FSW with perfect knowledge)
        rwElAngle = np.array([40.0, 40.0, 40.0, 40.0]) * mc.D2R
        rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0]) * mc.D2R
        wheelJs = 50.0 / (6000.0 * math.pi * 2.0 / 60)

        fswSetupRW.clearSetup()
        for elAngle, azAngle in zip(rwElAngle, rwAzimuthAngle):
            gsHat = (rbk.Mi(-azAngle, 3).dot(rbk.Mi(elAngle, 2))).dot(np.array([1, 0, 0]))
            fswSetupRW.create(gsHat,  # spin axis
                              wheelJs,  # kg*m^2
                              0.2)  # Nm        uMax

        self.fswRwConfigMsg = fswSetupRW.writeConfigMessage()

    def SetRWMotorTorque(self):
        """Set the RW motor torque information"""
        controlAxes_B = [
            1.0, 0.0, 0.0
            , 0.0, 1.0, 0.0
            , 0.0, 0.0, 1.0
        ]
        self.rwMotorTorque.controlAxes_B = controlAxes_B
        self.rwMotorTorque.vehControlInMsg.subscribeTo(self.cmdTorqueMsg)
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.rwMotorTorque.rwMotorTorqueOutMsg, self.cmdRwMotorMsg)
        self.rwMotorTorque.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)

    def SetThrConfigMsg(self):
        """Set the Thruster information"""

        if self.useDvThrusters:
            # 6 DV thrusters
            thPos = [[0, 0.95, -1.1],
                     [0.8227241335952166, 0.4750000000000003, -1.1],
                     [0.8227241335952168, -0.47499999999999976, -1.1],
                     [0, -0.95, -1.1],
                     [-0.8227241335952165, -0.4750000000000004, -1.1],
                     [-0.822724133595217, 0.4749999999999993, -1.1]]
            thDir = [[0.0, 0.0, 1.0],
                     [0.0, 0.0, 1.0],
                     [0.0, 0.0, 1.0],
                     [0.0, 0.0, 1.0],
                     [0.0, 0.0, 1.0],
                     [0.0, 0.0, 1.0]]
            maxThrust = 22
        else:
            # 8 thrusters are modeled that act in pairs to provide the desired torque
            thPos = [[825.5/1000.0, 880.3/1000.0, 1765.3/1000.0],
                     [825.5/1000.0, 880.3/1000.0, 260.4/1000.0],
                     [880.3/1000.0, 825.5/1000.0, 1765.3/1000.0],
                     [880.3/1000.0, 825.5/1000.0, 260.4/1000.0],
                     [-825.5/1000.0, -880.3/1000.0, 1765.3/1000.0],
                     [-825.5/1000.0, -880.3/1000.0, 260.4/1000.0],
                     [-880.3/1000.0, -825.5/1000.0, 1765.3/1000.0],
                     [-880.3/1000.0, -825.5/1000.0, 260.4/1000.0]]
            thDir = [[0.0, -1.0, 0.0],
                     [0.0, -1.0, 0.0],
                     [-1.0, 0.0, 0.0],
                     [-1.0, 0.0, 0.0],
                     [0.0, 1.0, 0.0],
                     [0.0, 1.0, 0.0],
                     [1.0, 0.0, 0.0],
                     [1.0, 0.0, 0.0]]
            maxThrust = 1

        fswSetupThrusters.clearSetup()
        for pos_B, dir_B in zip(thPos, thDir):
            fswSetupThrusters.create(pos_B, dir_B, maxThrust)

        self.fswThrConfigMsg = fswSetupThrusters.writeConfigMessage()

    def SetThrForceMapping(self):
        """Set the Thrust Force Mapping information"""
        if self.useDvThrusters:
            controlAxes_B = [1, 0, 0,
                             0, 1, 0]
            thrForceSign = -1
        else:
            controlAxes_B = [1, 0, 0,
                             0, 1, 0,
                             0, 0, 1]
            thrForceSign = +1

        self.thrForceMapping.thrForceSign = thrForceSign
        self.thrForceMapping.controlAxes_B = controlAxes_B

        self.thrForceMapping.cmdTorqueInMsg.subscribeTo(self.cmdTorqueMsg)
        self.thrForceMapping.thrConfigInMsg.subscribeTo(self.fswThrConfigMsg)
        self.thrForceMapping.vehConfigInMsg.subscribeTo(self.vcMsg)

    def SetThrFiringRemainder(self):
        """Set the Thrust Firing Remainder information"""
        self.thrFiringRemainder.thrMinFireTime = 0.002
        if self.useDvThrusters:
            self.thrFiringRemainder.baseThrustState = 1

        self.thrFiringRemainder.thrConfInMsg.subscribeTo(self.fswThrConfigMsg)
        self.thrFiringRemainder.thrForceInMsg.subscribeTo(self.thrForceMapping.thrForceCmdOutMsg)
        if self.useDvThrusters:
            messaging.THRArrayOnTimeCmdMsg_C_addAuthor(self.thrFiringRemainder.onTimeOutMsg, self.dvOnTimeCmdMsg)
        else:
            messaging.THRArrayOnTimeCmdMsg_C_addAuthor(self.thrFiringRemainder.onTimeOutMsg, self.acsOnTimeCmdMsg)

    def SetDvManeuver(self, SimBase):
        """Set the Dv Burn Maneuver information"""
        self.dvManeuver.navDataInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.transOutMsg)
        self.dvManeuver.burnDataInMsg.subscribeTo(self.dvBurnCmdMsg)
        messaging.THRArrayOnTimeCmdMsg_C_addAuthor(self.dvManeuver.thrCmdOutMsg, self.dvOnTimeCmdMsg)

    def SetLambertPlannerObject(self, SimBase):
        """Set the lambert planner object."""
        self.lambertPlannerObject.navTransInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.transOutMsg)


    def SetLambertSolverObject(self):
        """Set the lambert solver object."""
        self.lambertSolverObject.lambertProblemInMsg.subscribeTo(self.lambertPlannerObject.lambertProblemOutMsg)


    def SetLambertValidatorObject(self, SimBase):
        """Set the lambert validator object."""
        self.lambertValidatorObject.navTransInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.transOutMsg)
        self.lambertValidatorObject.lambertProblemInMsg.subscribeTo(self.lambertPlannerObject.lambertProblemOutMsg)
        self.lambertValidatorObject.lambertPerformanceInMsg.subscribeTo(
            self.lambertSolverObject.lambertPerformanceOutMsg)
        self.lambertValidatorObject.lambertSolutionInMsg.subscribeTo(self.lambertSolverObject.lambertSolutionOutMsg)
        self.lambertValidatorObject.dvBurnCmdOutMsg = self.dvBurnCmdMsg


    def SetLambertSurfaceRelativeVelocityObject(self, SimBase):
        """Set the lambert surface relative velocity object."""
        self.lambertSurfaceRelativeVelocityObject.lambertProblemInMsg.subscribeTo(
            self.lambertPlannerObject.lambertProblemOutMsg)
        self.lambertSurfaceRelativeVelocityObject.ephemerisInMsg.subscribeTo(
            SimBase.DynModels.EarthEphemObject.ephemOutMsgs[0])


    def SetLambertSecondDvObject(self):
        """Set the lambert second DV object."""
        self.lambertSecondDvObject.lambertSolutionInMsg.subscribeTo(self.lambertSolverObject.lambertSolutionOutMsg)
        self.lambertSecondDvObject.desiredVelocityInMsg.subscribeTo(
            self.lambertSurfaceRelativeVelocityObject.desiredVelocityOutMsg)
        self.lambertSecondDvObject.dvBurnCmdOutMsg = self.dvBurnCmdMsg

    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        """Initialize all the FSW objects"""

        # note that the order in which these routines are called is important.
        # To subscribe to a message that message must already exit.
        self.SetVehicleConfiguration()
        self.SetRWConfigMsg()
        self.SetInertial3DPointGuidance()
        self.SetHillPointGuidance(SimBase)
        self.SetCSSWlsEst(SimBase)
        self.SetSunSafePointGuidance(SimBase)
        self.SetVelocityPointGuidance(SimBase)
        self.SetDvPointGuidance()
        self.SetAttRefCorrection()
        self.SetAttitudeTrackingError(SimBase)
        self.SetMRPFeedbackControl(SimBase)
        self.SetMRPFeedbackRWA(SimBase)
        self.SetMRPFeedbackTH(SimBase)
        self.SetMRPSteering()
        self.SetRateServo(SimBase)
        self.SetRWMotorTorque()
        self.SetThrConfigMsg()
        self.SetThrForceMapping()
        self.SetThrFiringRemainder()
        self.SetDvManeuver(SimBase)
        self.SetLambertPlannerObject(SimBase)
        self.SetLambertSolverObject()
        self.SetLambertValidatorObject(SimBase)
        self.SetLambertSurfaceRelativeVelocityObject(SimBase)
        self.SetLambertSecondDvObject()

    def setupGatewayMsgs(self, SimBase):
        """create C-wrapped gateway messages such that different modules can write to this message
        and provide a common input msg for down-stream modules"""
        self.cmdTorqueMsg = messaging.CmdTorqueBodyMsg_C()
        self.cmdTorqueDirectMsg = messaging.CmdTorqueBodyMsg_C()
        self.attRefMsg = messaging.AttRefMsg_C()
        self.attGuidMsg = messaging.AttGuidMsg_C()
        self.cmdRwMotorMsg = messaging.ArrayMotorTorqueMsg_C()
        self.acsOnTimeCmdMsg = messaging.THRArrayOnTimeCmdMsg_C()
        self.dvOnTimeCmdMsg = messaging.THRArrayOnTimeCmdMsg_C()

        # C++ wrapped gateway messages
        self.dvBurnCmdMsg = messaging.DvBurnCmdMsg()

        self.zeroGateWayMsgs()

        # connect gateway FSW effector command msgs with the dynamics
        SimBase.DynModels.extForceTorqueObject.cmdTorqueInMsg.subscribeTo(self.cmdTorqueDirectMsg)
        SimBase.DynModels.rwStateEffector.rwMotorCmdInMsg.subscribeTo(self.cmdRwMotorMsg)
        SimBase.DynModels.thrustersDynamicEffectorACS.cmdsInMsg.subscribeTo(self.acsOnTimeCmdMsg)
        SimBase.DynModels.thrustersDynamicEffectorDV.cmdsInMsg.subscribeTo(self.dvOnTimeCmdMsg)

    def zeroGateWayMsgs(self):
        """Zero all the FSW gateway message payloads"""
        self.cmdTorqueMsg.write(messaging.CmdTorqueBodyMsgPayload())
        self.cmdTorqueDirectMsg.write(messaging.CmdTorqueBodyMsgPayload())
        self.attRefMsg.write(messaging.AttRefMsgPayload())
        self.attGuidMsg.write(messaging.AttGuidMsgPayload())
        self.cmdRwMotorMsg.write(messaging.ArrayMotorTorqueMsgPayload())
        self.acsOnTimeCmdMsg.write(messaging.THRArrayOnTimeCmdMsgPayload())
        self.dvOnTimeCmdMsg.write(messaging.THRArrayOnTimeCmdMsgPayload())
        self.dvBurnCmdMsg.write(messaging.DvBurnCmdMsgPayload())

    def SetAttThrusters(self, useDvThrusters):
        """Change thruster type used for attitude control (ACS or DV)"""
        self.useDvThrusters = useDvThrusters
        self.SetThrConfigMsg()
        self.SetThrForceMapping()
        self.SetThrFiringRemainder()

    @property
    def inertial3DData(self):
        return self.inertial3D

    inertial3DData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to inertial3DData as inertial3D",
        inertial3DData)

    @property
    def inertial3DWrap(self):
        return self.inertial3D

    inertial3DWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to inertial3DWrap as inertial3D",
        inertial3DWrap)


    @property
    def hillPointData(self):
        return self.hillPoint

    hillPointData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to hillPointData as hillPoint",
        hillPointData)

    @property
    def hillPointWrap(self):
        return self.hillPoint

    hillPointWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to hillPointWrap as hillPoint",
        hillPointWrap)


    @property
    def sunSafePointData(self):
        return self.sunSafePoint

    sunSafePointData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to sunSafePointData as sunSafePoint",
        sunSafePointData)

    @property
    def sunSafePointWrap(self):
        return self.sunSafePoint

    sunSafePointWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to sunSafePointWrap as sunSafePoint",
        sunSafePointWrap)


    @property
    def velocityPointData(self):
        return self.velocityPoint

    velocityPointData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to velocityPointData as velocityPoint",
        velocityPointData)

    @property
    def velocityPointWrap(self):
        return self.velocityPoint

    velocityPointWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to velocityPointWrap as velocityPoint",
        velocityPointWrap)


    @property
    def cssWlsEstData(self):
        return self.cssWlsEst

    cssWlsEstData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to cssWlsEstData as cssWlsEst",
        cssWlsEstData)

    @property
    def cssWlsEstWrap(self):
        return self.cssWlsEst

    cssWlsEstWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to cssWlsEstWrap as cssWlsEst",
        cssWlsEstWrap)


    @property
    def trackingErrorData(self):
        return self.trackingError

    trackingErrorData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to trackingErrorData as trackingError",
        trackingErrorData)

    @property
    def trackingErrorWrap(self):
        return self.trackingError

    trackingErrorWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to trackingErrorWrap as trackingError",
        trackingErrorWrap)


    @property
    def mrpFeedbackControlData(self):
        return self.mrpFeedbackControl

    mrpFeedbackControlData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to mrpFeedbackControlData as mrpFeedbackControl",
        mrpFeedbackControlData)

    @property
    def mrpFeedbackControlWrap(self):
        return self.mrpFeedbackControl

    mrpFeedbackControlWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to mrpFeedbackControlWrap as mrpFeedbackControl",
        mrpFeedbackControlWrap)


    @property
    def mrpFeedbackRWsData(self):
        return self.mrpFeedbackRWs

    mrpFeedbackRWsData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to mrpFeedbackRWsData as mrpFeedbackRWs",
        mrpFeedbackRWsData)

    @property
    def mrpFeedbackRWsWrap(self):
        return self.mrpFeedbackRWs

    mrpFeedbackRWsWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to mrpFeedbackRWsWrap as mrpFeedbackRWs",
        mrpFeedbackRWsWrap)


    @property
    def mrpSteeringData(self):
        return self.mrpSteering

    mrpSteeringData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to mrpSteeringData as mrpSteering",
        mrpSteeringData)

    @property
    def mrpSteeringWrap(self):
        return self.mrpSteering

    mrpSteeringWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to mrpSteeringWrap as mrpSteering",
        mrpSteeringWrap)


    @property
    def rateServoData(self):
        return self.rateServo

    rateServoData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to rateServoData as rateServo",
        rateServoData)

    @property
    def rateServoWrap(self):
        return self.rateServo

    rateServoWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to rateServoWrap as rateServo",
        rateServoWrap)


    @property
    def rwMotorTorqueData(self):
        return self.rwMotorTorque

    rwMotorTorqueData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to rwMotorTorqueData as rwMotorTorque",
        rwMotorTorqueData)

    @property
    def rwMotorTorqueWrap(self):
        return self.rwMotorTorque

    rwMotorTorqueWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to rwMotorTorqueWrap as rwMotorTorque",
        rwMotorTorqueWrap)
