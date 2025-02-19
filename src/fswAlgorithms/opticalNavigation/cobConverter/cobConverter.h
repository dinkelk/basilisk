/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _COB_CONVERT_H_
#define _COB_CONVERT_H_

#include <stdint.h>
#include <Eigen/Dense>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/messaging/messaging.h"

#include "architecture/msgPayloadDefC/CameraConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefCpp/OpNavCOBMsgPayload.h"
#include "architecture/msgPayloadDefCpp/OpNavUnitVecMsgPayload.h"

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief visual limb finding module */
class CobConverter: public SysModel {
public:
    CobConverter();
    ~CobConverter();
    
    void UpdateState(uint64_t CurrentSimNanos);
    void Reset(uint64_t CurrentSimNanos);
    
public:
    Message<OpNavUnitVecMsgPayload> opnavUnitVecOutMsg;
    ReadFunctor<OpNavCOBMsgPayload> opnavCOBInMsg;
    ReadFunctor<CameraConfigMsgPayload> cameraConfigInMsg;
    ReadFunctor<NavAttMsgPayload> navAttInMsg;

    uint64_t sensorTimeTag;
    BSKLogger bskLogger;
};

#endif
