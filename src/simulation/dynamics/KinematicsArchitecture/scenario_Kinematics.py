import os
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

import numpy as np
from Basilisk.simulation import KinematicsEngine
from Basilisk.simulation import Frame
from Basilisk.simulation import Vector
from Basilisk.simulation import Tensor
from Basilisk.simulation import AttitudeParameterization
from Basilisk.simulation import Assembly
from Basilisk.simulation import Part
from Basilisk.simulation import Point
from Basilisk.simulation import Joint
from Basilisk.simulation import Hinge
from Basilisk.utilities import RigidBodyKinematics as rbk

def runLeah():
    myKinematicsEngine = KinematicsEngine.KinematicsEngine()

    # Create frames
    frameN = myKinematicsEngine.createFrame()
    frameB = myKinematicsEngine.createFrame(frameN)
    frameM = myKinematicsEngine.createFrame(frameB)

    # Create parts
    part1Inertia = [[10, 0, 0], [0, 10, 0], [0, 0, 10]]
    part2Inertia = [[10, 0, 0], [0, 8, 0], [0, 0, 5]]
    part1 = myKinematicsEngine.createPart(frameM)
    part2 = myKinematicsEngine.createPart(frameB)

    # Create part data
    frameF = part1.frame
    frameP = part2.frame
    part1.IPntSc_S.matrix = part1Inertia
    part2.IPntSc_S.matrix = part2Inertia
    part1.IPntSc_S.writtenFrame = frameF
    part2.IPntSc_S.writtenFrame = frameP

    # Create assembly
    myAssembly = myKinematicsEngine.createAssembly()
    myAssembly.addPart(part1)
    myAssembly.addPart(part2)

    # Create position vector data
    r_PcP_P = [0, 2, 1]  # [m]
    r_FcF_F = [1, 2, 0]  # [m]
    r_BN_N = [1.0, 2.0, 3.0]  # [m]
    r_PB_B = [2.0, 3.0, 4.0]  # [m]
    r_MB_B = [2.0, 1.0, 3.0]  # [m]
    r_FM_M = [4.0, 3.0, 1.0]  # [m]

    # Create frame attitude data
    theta_BN = 30 * np.pi / 180  # [rad]
    theta_PB = 10 * np.pi / 180  # [rad]
    theta_MB = -5 * np.pi / 180  # [rad]
    theta_FM = -10 * np.pi / 180  # [rad]

    dcm_BN = np.array([[1, 0, 0],
                       [0, np.cos(theta_BN),  np.sin(theta_BN)],
                       [0, -np.sin(theta_BN),  np.cos(theta_BN)]])
    dcm_PB = np.array([[np.cos(theta_PB),  np.sin(theta_PB),  0],
                       [-np.sin(theta_PB),  np.cos(theta_PB), 0],
                       [0,  0,  1]])
    dcm_MB = np.array([[np.cos(theta_MB),  np.sin(theta_MB),  0],
                       [-np.sin(theta_MB),  np.cos(theta_MB), 0],
                       [0,  0,  1]])
    dcm_FM = np.array([[np.cos(theta_FM),  0, -np.sin(theta_FM)],
                       [0, 1, 0],
                       [np.sin(theta_FM),  0, np.cos(theta_FM)]])

    sigma_BN = rbk.C2MRP(dcm_BN)
    sigma_PB = rbk.C2MRP(dcm_PB)
    sigma_MB = rbk.C2MRP(dcm_MB)
    sigma_FM = rbk.C2MRP(dcm_FM)

    # Create angular velocity data
    omega_PB_P = [0.5, 1, 0.5]  # [rad/s]
    omega_FM_F = [-1.0, -2.0, 3.0]  # [rad/s]
    omega_MB_M = [-2.0, -3.0, -4.0]  # [rad/s]
    omega_BN_B = [0.0, 1, 0.0]  # [rad/s]

    # Create frame data
    frameB.tag = "frameB"
    frameM.tag = "frameM"
    frameF.tag = "frameF"
    frameP.tag = "frameP"
    frameN.tag = "frameN"
    frameB.r_SP.matrix = r_BN_N
    frameM.r_SP.matrix = r_MB_B
    frameF.r_SP.matrix = r_FM_M
    frameP.r_SP.matrix = r_PB_B
    frameB.sigma_SP = sigma_BN
    frameP.sigma_SP = sigma_PB
    frameM.sigma_SP = sigma_MB
    frameF.sigma_SP = sigma_FM
    frameB.omega_SP.matrix = omega_BN_B
    frameP.omega_SP.matrix = omega_PB_P
    frameM.omega_SP.matrix = omega_MB_M
    frameF.omega_SP.matrix = omega_FM_F
    frameB.originPoint.tag = "pointB"
    frameP.originPoint.tag = "pointP"
    frameM.originPoint.tag = "pointM"
    frameF.originPoint.tag = "pointF"
    frameN.originPoint.tag = "pointN"

    # Create part data
    m1 = 10  # [kg]
    m2 = 20  # [kg]
    part1.mass = m1
    part2.mass = m2
    part1.r_ScS.matrix = r_FcF_F
    part2.r_ScS.matrix = r_PcP_P
    # part1.r_ScS.writtenFrame = frameF
    # part2.r_ScS.writtenFrame = frameP
    part1.r_ScS.headPoint.tag = "pointFc"
    part2.r_ScS.headPoint.tag = "pointPc"

    # Create velocity data
    rBPrime_PcP_M = [0.1, 0, 0.2]
    part2.r_ScS.setFirstOrder(rBPrime_PcP_M, frameM, frameB)

    # Call kinematic engine functions
    # sigma_FPKin = myKinematicsEngine.findRelativeAttitude(frameF, frameP)
    # r_FBKin = frameF.r_SP.add(frameM.r_SP)
    # r_FB_BKin = r_FBKin.getZerothOrder(frameB)
    # omega_FBKin = frameF.omega_SP.add(frameM.omega_SP)
    # omega_FB_MKin = omega_FBKin.getZerothOrder(frameM)
    # r_PcF_Kin = myKinematicsEngine.callFindRelativePosition(part2.r_ScS.headPoint, frameF.originPoint)
    # r_PcF_MKin = r_PcF_Kin.getZerothOrder(frameM)
    # omega_PFKin = myKinematicsEngine.callFindRelativeAngularVelocity(frameP, frameF)
    # omega_PF_MKin = omega_PFKin.getZerothOrder(frameM)
    # assemblyMassKin = myKinematicsEngine.getAssemblyMass(myAssembly)
    # r_AssemCMPntBKin = myKinematicsEngine.getAssemblyCOM(myAssembly, frameB.originPoint)
    # r_AssemCMPntB_MKin = r_AssemCMPntBKin.getZerothOrder(frameM)
    # IPart1PntBKin = myKinematicsEngine.parallelAxisTheorem(part1, frameB.originPoint)
    # IPart1PntB_MKin = IPart1PntBKin.getZerothOrder(frameM)
    # IAssemPntBKin = myKinematicsEngine.getAssemblyInertia(myAssembly, frameB.originPoint)
    # IAssemPntB_MKin = IAssemPntBKin.getZerothOrder(frameM)
    rFPrime_PcP_NKin = myKinematicsEngine.getFirstOrder(part2.r_ScS, frameF, frameN)

    # Testing getter functions
    r_PcP_FKin = part2.r_ScS.getZerothOrder(frameF)
    IPntPc_MKin = part2.IPntSc_S.getZerothOrder(frameM)

    # Calculate the truth data
    sigma_FP = [0.0, 0.0, 0.0]
    r_FB_B = [0.0, 0.0, 0.0]
    omega_FB_M = [0.0, 0.0, 0.0]
    r_PcF_M = [0.0, 0.0, 0.0]
    IPart1PntB_M = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

    # 1. Calculate sigma_FP
    dcm_FP = np.matmul(np.matmul(dcm_FM, dcm_MB), np.transpose(dcm_PB))
    sigma_FP = rbk.C2MRP(dcm_FP)

    # 2. Calculate r_FB_B
    r_FM_B = np.matmul(np.transpose(dcm_MB), r_FM_M)
    r_FB_B = np.add(r_FM_B, r_MB_B)

    # 3. Calculate omega_FB_M
    omega_FM_M = np.matmul(np.transpose(dcm_FM), omega_FM_F)
    omega_FB_M = np.add(omega_FM_M, omega_MB_M)

    # 4. Calculate r_PcF_M
    r_PcP_M = np.matmul(np.matmul(dcm_MB, np.transpose(dcm_PB)), r_PcP_P)
    r_PB_M = np.matmul(dcm_MB, r_PB_B)
    r_PcB_M = np.add(r_PcP_M, r_PB_M)
    r_MB_M = np.matmul(dcm_MB, r_MB_B)
    r_PcM_M = np.subtract(r_PcB_M, r_MB_M)
    r_PcF_M = np.subtract(r_PcM_M, r_FM_M)

    # 5. Calculate omega_PF_M
    dcm_PM = np.matmul(dcm_PB, dcm_MB.transpose())
    omega_PF_M = np.subtract(np.subtract(np.matmul(dcm_PM.transpose(), omega_PB_P), omega_MB_M), np.matmul(dcm_FM.transpose(), omega_FM_F))

    # 6. Calculate IPart1PntB_M
    rFcB_M = np.add(np.add(np.matmul(np.transpose(dcm_FM), r_FcF_F), r_FM_M), np.matmul(dcm_MB, r_MB_B))
    rTildeFcB_M = [[0, -rFcB_M[2], rFcB_M[1]],
                   [rFcB_M[2], 0, -rFcB_M[0]],
                   [-rFcB_M[1], rFcB_M[0], 0]]
    IPart1PntB_M = np.matmul(np.matmul(np.transpose(dcm_FM), part1Inertia), dcm_FM) + m1 * np.matmul(rTildeFcB_M, np.transpose(rTildeFcB_M))

    # 7. Calculate assembly mass
    assemblyMass = m1 + m2

    # 8. Calculate assembly CoM
    r_FcB_M = np.add(np.add(np.matmul(np.transpose(dcm_FM), r_FcF_F), r_FM_M), r_MB_M)
    r_AssemCMPntB_M = np.add(m2 * r_PcB_M, m1 * r_FcB_M) / assemblyMass

    # 9. Calculate assembly inertia IAssemPntB_M
    rTildePcB_M = [[0, -r_PcB_M[2], r_PcB_M[1]],
                   [r_PcB_M[2], 0, -r_PcB_M[0]],
                   [-r_PcB_M[1], r_PcB_M[0], 0]]
    IPart2PntB_M = np.matmul(np.matmul(np.transpose(dcm_PM), part2Inertia), dcm_PM) + m2 * np.matmul(rTildePcB_M, np.transpose(rTildePcB_M))
    IAssemPntB_M = np.add(IPart1PntB_M, IPart2PntB_M)

    # Calculating zeroth order getter truth data
    # 1. Vector getter
    r_PcP_F = np.matmul(dcm_FP, r_PcP_P)

    # 2. Tensor getter
    IPntPc_M = np.matmul(np.matmul(np.transpose(dcm_PM), part2Inertia), dcm_PM)

    # Testing tensor/vector math functions
    # r_MB_BVecDotr_FM_MKin = myKinematicsEngine.dot(frameM.r_SP, frameF.r_SP, frameB)
    # r_MB_BCrossDotr_FM_MKin = myKinematicsEngine.cross(frameM.r_SP, frameF.r_SP, frameB)
    # IAssemPntB_MTensor = myKinematicsEngine.createInertiaTensor(frameB.originPoint)
    # IAssemPntB_MTensor.setZerothOrder(IAssemPntB_M, frameM)
    # IAssemPntB_MInvKin = myKinematicsEngine.tensorInverse(IAssemPntB_MTensor)
    # IAssemPntB_MTimesr_MB_BKin = myKinematicsEngine.tensorTimesVector(IAssemPntB_MTensor, frameM.r_SP, frameB)
    # IAssemPntB_MTimesIAssemPntB_MKin = myKinematicsEngine.tensorTimesTensor(IAssemPntB_MTensor, IAssemPntB_MTensor, frameB)

    # Calculating tensor/vector math truth data
    # 1. Dot product
    r_MB_BVecDotr_FM_M = np.dot(r_MB_B, r_FM_B)

    # 2. Cross product
    r_MB_BCrossr_FM_M = np.cross(r_MB_B, r_FM_B)

    # 3. Tensor inverse
    IAssemPntB_MInv = np.linalg.inv(IAssemPntB_M)

    # 4. Tensor times vector
    IAssemPntB_MTimesr_MB_B = np.matmul(np.matmul(dcm_MB.transpose(), np.matmul(IAssemPntB_M, dcm_MB)), r_MB_B)

    # Tensor times tensor
    IAssemPntB_MTimesIAssemPntB_M = np.matmul(np.matmul(dcm_MB.transpose(), np.matmul(IAssemPntB_M, dcm_MB)), np.matmul(dcm_MB.transpose(), np.matmul(IAssemPntB_M, dcm_MB)))

    # Calculate transport theorem result
    dcm_NM = np.matmul(np.transpose(dcm_BN), np.transpose(dcm_MB))
    dcm_NF = np.matmul(dcm_NM, np.transpose(dcm_FM))
    omega_BM_M = [-omega_MB_M[0], -omega_MB_M[1], -omega_MB_M[2]]
    omega_BF_N = np.subtract(np.matmul(dcm_NM, omega_BM_M), np.matmul(dcm_NF, omega_FM_F))
    dcm_NP = np.matmul(dcm_NF, dcm_FP)
    r_PcP_N = np.matmul(dcm_NP, r_PcP_P)
    crossTerm = np.cross(omega_BF_N, r_PcP_N)
    derivTerm = np.matmul(dcm_NM, rBPrime_PcP_M)
    rFPrime_PcP_N = np.add(derivTerm, crossTerm)

    # print("\n** ** ** ** ** TESTING 'ZEROTH' ORDER RELATIVE KINEMATICS ** ** ** ** **")
    #
    # print("\n\n1. RELATIVE ATTITUDE:")
    # print("\nTRUTH: sigma_FP: ")
    # print(sigma_FP)
    # print("\nKINEMATICS ENGINE: sigma_FP: ")
    # print(sigma_FPKin)
    #
    # print("\n\n2. ADD POSITION VECTORS:")
    # print("\nTRUTH: r_FB_B: ")
    # print(r_FB_B)
    # print("\nKINEMATICS ENGINE: r_FB_B: ")
    # print(r_FB_BKin)
    #
    # print("\n\n3. ADD ANGULAR VELOCITY VECTORS:")
    # print("\nTRUTH: omega_FB_M: ")
    # print(omega_FB_M)
    # print("\nKINEMATICS ENGINE: omega_FB_M: ")
    # print(omega_FB_MKin)
    #
    # print("\n\n4. RELATIVE POSITION:")
    # print("\nTRUTH: r_PcF_M: ")
    # print(r_PcF_M)
    # print("\nKINEMATICS ENGINE: r_PcF_M: ")
    # print(r_PcF_MKin)
    #
    # print("\n\n5. RELATIVE ANGULAR VELOCITY:")
    # print("\nTRUTH: omega_PF_M: ")
    # print(omega_PF_M)
    # print("\nKINEMATICS ENGINE: omega_PF_M: ")
    # print(omega_PF_MKin)
    #
    # print("\n\n6. PARALLEL AXIS THEOREM:")
    # print("\nTRUTH: Inertia: ")
    # print(IPart1PntB_M)
    # print("\nKINEMATICS ENGINE: Inertia: ")
    # print(IPart1PntB_MKin)
    #
    # print("\n\n7. ASSEMBLY MASS:")
    # print("\nTRUTH: Assembly Mass: ")
    # print(assemblyMass)
    # print("\nKINEMATICS ENGINE: Assembly Mass: ")
    # print(assemblyMassKin)
    #
    # print("\n\n8. ASSEMBLY COM:")
    # print("\nTRUTH: r_AssemCMPntB_M: ")
    # print(r_AssemCMPntB_M)
    # print("\nKINEMATICS ENGINE: r_AssemCMPntB_M: ")
    # print(r_AssemCMPntB_MKin)
    #
    # print("\n\n9. ASSEMBLY INERTIA:")
    # print("\nTRUTH: IAssemPntB_M: ")
    # print(IAssemPntB_M)
    # print("\nKINEMATICS ENGINE: IAssemPntB_M: ")
    # print(IAssemPntB_MKin)

    # print("\n** ** ** ** ** TESTING ZEROTH ORDER GETTER FUNCTIONS ** ** ** ** **")
    #
    # print("\n\n1. VECTOR GETTER:")
    # print("\nTRUTH: r_PcP_F: ")
    # print(r_PcP_F)
    # print("\nGETTER: r_PcP_F: ")
    # print(r_PcP_FKin)
    #
    # print("\n\n2. TENSOR GETTER:")
    # print("\nTRUTH: IPntPc_M: ")
    # print(IPntPc_M)
    # print("\nGETTER: IPntPc_M: ")
    # print(IPntPc_MKin)

    print("\n** ** ** ** ** TESTING FIRST ORDER GETTER FUNCTIONS ** ** ** ** **")

    print("\n\n1. VECTOR TRANSPORT THEOREM:")
    print("\nTRUTH: rFPrime_PcP_N: ")
    print(rFPrime_PcP_N)
    print("\nKINEMATICS ENGINE: rFPrime_PcP_N: ")
    print(rFPrime_PcP_NKin)

    # print("\n** ** ** ** ** TESTING TENSOR/VECTOR MATH FUNCTIONS ** ** ** ** **")

    # print("\n\n1. VECTOR DOT PRODUCT:")
    # print("\nTRUTH: r_MB_BVecDotr_FM_M: ")
    # print(r_MB_BVecDotr_FM_MKin)
    # print("\nKINEMATICS ENGINE: r_MB_BVecDotr_FM_M: ")
    # print(r_MB_BVecDotr_FM_M)
    #
    # print("\n\n2. VECTOR CROSS PRODUCT:")
    # print("\nTRUTH: r_MB_BVecCrossr_FM_M: ")
    # print(r_MB_BCrossr_FM_M)
    # print("\nKINEMATICS ENGINE: r_MB_BCrossDotr_FM_M: ")
    # print(r_MB_BCrossDotr_FM_MKin.matrix)
    #
    # print("\n\n3. INERTIA INVERSE:")
    # print("\nTRUTH: IAssemPntB_MInv: ")
    # print(IAssemPntB_MInv)
    # print("\nKINEMATICS ENGINE: IAssemPntB_MInv: ")
    # print(IAssemPntB_MInvKin.matrix)
    #
    # print("\n\n4. TENSOR TIMES VECTOR:")
    # print("\nTRUTH: IAssemPntB_MTimesr_MB_BKin: ")
    # print(IAssemPntB_MTimesr_MB_B)
    # print("\nKINEMATICS ENGINE: IAssemPntB_MTimesr_MB_BKin: ")
    # print(IAssemPntB_MTimesr_MB_BKin.matrix)
    #
    # print("\n\n5. TENSOR TIMES TENSOR:")
    # print("\nTRUTH: IAssemPntB_MTimesIAssemPntB_M: ")
    # print(IAssemPntB_MTimesIAssemPntB_M)
    # print("\nKINEMATICS ENGINE: IAssemPntB_MTimesIAssemPntB_M: ")
    # print(IAssemPntB_MTimesIAssemPntB_MKin.matrix)

def runJoao():
    myKinematicsEngine = KinematicsEngine.KinematicsEngine()

    myInertialFrame = myKinematicsEngine.createFrame()
    myInertialFrame.tag = "inertial"
    myBodyFrame = myKinematicsEngine.createFrame(myInertialFrame)
    myBodyFrame.tag = "body"

    myPart1 = myKinematicsEngine.createPart(myBodyFrame)
    myPart1.frame.tag = "part1"
    myPart2 = myKinematicsEngine.createPart(myPart1.frame)
    myPart2.frame.tag = "part2"

    myJoint = myKinematicsEngine.createRotaryOneDOFJoint()
    # myJoint.hingeVector[0].spinAxis = [1, 0, 0]
    myJoint.lowerFrame.tag = "lower"
    myJoint.upperFrame.tag = "upper"

    myKinematicsEngine.connect(myPart1, myJoint, myPart2)

if __name__ == "__main__":
    runLeah()
    #runJoao()