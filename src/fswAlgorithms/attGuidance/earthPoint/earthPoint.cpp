/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */


#include "earthPoint.h"
#include <math.h>
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "fswAlgorithms/attGuidance/_GeneralModuleFiles/attitudePointingLibrary.h"

const double epsilon = 1e-12;                           // module tolerance for zero

/*! Module constructor */
EarthPoint::EarthPoint() = default;


/*! Module destructor */
EarthPoint::~EarthPoint() = default;


/*! Initialize C-wrapped output messages */
void EarthPoint::SelfInit(){
    AttRefMsg_C_init(&this->attRefOutMsgC);
}

/*! This method is used to reset the module.
 @return void
 */
void EarthPoint::Reset(uint64_t CurrentSimNanos)
{
    if (!this->attNavInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, ".attNavInMsg wasn't connected.");
    }
    if (!this->transNavInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, ".transNavInMsg wasn't connected.");
    }
    if (!this->ephemerisInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, ".ephemerisInMsg wasn't connected.");
    }
    this->callCount = 0;
    this->antennaCount = 1;
}


/*! This method is the main carrier for the boresight calculation routine.  If it detects
 that it needs to re-init (direction change maybe) it will re-init itself.
 Then it will compute the angles away that the boresight is from the celestial target.
 @return void
 @param CurrentSimNanos The current simulation time for system
 */
void EarthPoint::UpdateState(uint64_t CurrentSimNanos)
{
    
}


void EarthPoint::ComputeReferenceFrame(double hRefHat_B[3], double hReqHat_B[3], double rHat_SB_B[3], double BN[3][3], double RN[3][3])
{
    /*! compute intermediate rotation DB to align the boresight */
    double DB[3][3];
    boresightAlignment(hRefHat_B, hReqHat_B, epsilon, DB);

    /*! map Sun direction vector to intermediate frame */
    double rHat_SB_D[3];
    m33MultV3(DB, rHat_SB_B, rHat_SB_D);

    /*! define the coefficients of the quadratic equation A, B, and C for the solar array drive axis */
    double e_psi[3];
    v3Copy(hRefHat_B, e_psi);
    double b3[3];
    v3Cross(rHat_SB_D, e_psi, b3);
    double A = 2 * v3Dot(rHat_SB_D, e_psi) * v3Dot(e_psi, this->a1Hat_B) - v3Dot(this->a1Hat_B, rHat_SB_D);
    double B = 2 * v3Dot(this->a1Hat_B, b3);
    double C = v3Dot(this->a1Hat_B, rHat_SB_D);

    /*! define the coefficients of the quadratic equation D, E, and F for the Sun-constrained axis */
    double D = 2 * v3Dot(rHat_SB_D, e_psi) * v3Dot(e_psi, this->a2Hat_B) - v3Dot(this->a2Hat_B, rHat_SB_D);
    double E = 2 * v3Dot(this->a2Hat_B, b3);
    double F = v3Dot(this->a2Hat_B, rHat_SB_D);

    /*! compute the solution(s) to the optimized solar array alignment problem */
    SolutionSpace solarArraySolutions(A, B, C, epsilon);

    double PRV_psi[3];
    switch (this->alignmentPriority) {

        case solarArrayAlign :
            if (solarArraySolutions.numberOfZeros() == 2) {
                double psi1 = solarArraySolutions.returnAbsMin(1);
                double psi2 = solarArraySolutions.returnAbsMin(2);
                double PRV_psi1[3];
                v3Scale(psi1, e_psi, PRV_psi1);
                double PRV_psi2[3];
                v3Scale(psi2, e_psi, PRV_psi2);
                double P1D[3][3];
                PRV2C(PRV_psi1, P1D);
                double P2D[3][3];
                PRV2C(PRV_psi2, P2D);
                double rHat_SB_P1[3];
                m33MultV3(P1D, rHat_SB_D, rHat_SB_P1);
                double rHat_SB_P2[3];
                m33MultV3(P2D, rHat_SB_D, rHat_SB_P2);
                if (fabs(v3Dot(a2Hat_B, rHat_SB_P2) - v3Dot(a2Hat_B, rHat_SB_P1)) > 0) {
                    v3Scale(psi2, e_psi, PRV_psi);
                }
                else {
                    v3Scale(psi1, e_psi, PRV_psi);
                }
            }
            else {
                double psi = solarArraySolutions.returnAbsMin(1);
                v3Scale(psi, e_psi, PRV_psi);
            }
            break;

        case sunConstrAxisAlign :
            double k = cos(this->beta);
            SolutionSpace sunConstAxisSolutions(D-k, E, F-k, epsilon);
            if (sunConstAxisSolutions.isEmpty) {
                double psi = sunConstAxisSolutions.returnAbsMin(1);
                v3Scale(psi, e_psi, PRV_psi);
            }
            else {
                if (solarArraySolutions.numberOfZeros() == 2) {
                    double psi1 = solarArraySolutions.returnAbsMin(1);
                    double psi2 = solarArraySolutions.returnAbsMin(2);
                    double deltaPsi1 = psi1 - sunConstAxisSolutions.passThrough(psi1);
                    double deltaPsi2 = psi2 - sunConstAxisSolutions.passThrough(psi2);
                    if (fabs(deltaPsi2 - deltaPsi1) > 0) {
                        v3Scale(psi1, e_psi, PRV_psi);
                    }
                    else {
                        v3Scale(psi2, e_psi, PRV_psi);
                    }
                }
                else {
                    double psi = solarArraySolutions.returnAbsMin(1);
                    psi = sunConstAxisSolutions.passThrough(psi);
                    v3Scale(psi, e_psi, PRV_psi);
                }
            }
            break;
    }

    /*! map PRV to RD direction cosine matrix */
    double RD[3][3];
    PRV2C(PRV_psi, RD);
    double RB[3][3];
    m33MultM33(RD, DB, RB);

    m33MultM33(RB, BN, RN);
}

void EarthPoint::ChooseReferenceFrame(double R1N[3][3], double R2N[3][3], double BN[3][3], double rHat_SB_B[3], double RN[3][3])
{
    // map Sun direction vector to R1 frame
    double R1B[3][3];
    m33MultM33t(R1N, BN, R1B);
    double rHat_SB_R1[3];
    m33MultV3(R1B, rHat_SB_B, rHat_SB_R1);
    // map Sun direction vector to R2 frame
    double R2B[3][3];
    m33MultM33t(R2N, BN, R2B);
    double rHat_SB_R2[3];
    m33MultV3(R2B, rHat_SB_B, rHat_SB_R2);

    double a1dotR1 = v3Dot(this->a1Hat_B, rHat_SB_R1);
    double a1dotR2 = v3Dot(this->a1Hat_B, rHat_SB_R2);
    double a2dotR1 = v3Dot(this->a2Hat_B, rHat_SB_R1);
    double a2dotR2 = v3Dot(this->a2Hat_B, rHat_SB_R2);

    if (alignmentPriority == solarArrayAlign) {
        if (fabs(a1dotR2) - fabs(a1dotR1) > epsilon) {
            // R1 has the solar array drive axis more perpendicular to sunlight
            m33Copy(R1N, RN);
        }
        else if (fabs(a1dotR2) - fabs(a1dotR1) < -epsilon) {
            // R2 has the solar array drive axis more perpendicular to sunlight
            m33Copy(R2N, RN);
        }
        else {
            // if the two have same sunlight incidence, choose based on Sun-constrained vector
            if (a2dotR1 - a2dotR2 > epsilon) {
                m33Copy(R1N, RN);
            }
            else {
                m33Copy(R2N, RN);
            }
        }
    }
    else {
        double cBeta = cos(this->beta);
        if ((a2dotR1 >= cBeta) && (a2dotR2 < cBeta)) {
            m33Copy(R1N, RN);
        }
        else if ((a2dotR1 < cBeta) && (a2dotR2 >= cBeta)) {
            m33Copy(R2N, RN);
        }
        else if ((a2dotR1 < cBeta) && (a2dotR2 < cBeta)) {
            if (a2dotR2 < a2dotR1) {
                m33Copy(R1N, RN);
            }
            else {
                m33Copy(R2N, RN);
            }
        }
        else {
            if (fabs(a1dotR2) - fabs(a1dotR1) > epsilon) {
                // R1 has the solar array drive axis more perpendicular to sunlight
                m33Copy(R1N, RN);
            }
            else if (fabs(a1dotR2) - fabs(a1dotR1) < -epsilon) {
                // R2 has the solar array drive axis more perpendicular to sunlight
                m33Copy(R2N, RN);
            }
        }
    }
}
