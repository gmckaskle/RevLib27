/*
 * Copyright (c) 2018-2026 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <string>

#include "rev/REVLibDevices.h"

extern "C" {

typedef enum _c_REVLib_ErrorCode {
    c_REVLibError_None = 0,
    c_REVLibError_General = 1,
    c_REVLibError_CANTimeout = 2,
    c_REVLibError_NotImplemented = 3,
    c_REVLibError_HAL = 4,
    c_REVLibError_CantFindFirmware = 5,
    c_REVLibError_FirmwareTooOld = 6,
    c_REVLibError_FirmwareTooNew = 7,
    c_REVLibError_ParamInvalidID = 8,
    c_REVLibError_ParamMismatchType = 9,
    c_REVLibError_ParamAccessMode = 10,
    c_REVLibError_ParamInvalid = 11,
    c_REVLibError_ParamNotImplementedDeprecated = 12,
    c_REVLibError_FollowConfigMismatch = 13,
    c_REVLibError_Invalid = 14,
    c_REVLibError_SetpointOutOfRange = 15,
    c_REVLibError_Unknown = 16,
    c_REVLibError_CANDisconnected = 17,
    c_REVLibError_DuplicateCANId = 18,
    c_REVLibError_InvalidCANId = 19,
    c_REVLibError_SparkMaxDataPortAlreadyConfiguredDifferently = 20,
    c_REVLibError_SparkFlexBrushedWithoutDock = 21,
    c_REVLibError_InvalidBrushlessEncoderConfiguration = 22,
    c_REVLibError_FeedbackSensorIncompatibleWithDataPortConfig = 23,
    c_REVLibError_ParamInvalidChannel = 24,
    c_REVLibError_ParamInvalidValue = 25,
    c_REVLibError_CannotPersistParametersWhileEnabled = 26,
    c_REVLibError_NotReadyToReceiveCommand = 27,
    c_REVLibError_NumCodes = 28
} c_REVLib_ErrorCode;

void c_REVLib_SendError(c_REVLib_ErrorCode code, REVDevice deviceType,
                        int busId, int deviceId);
void c_REVLib_SendErrorText(c_REVLib_ErrorCode code, REVDevice deviceType,
                            int busId, int deviceId, std::string context);

void c_REVLib_FlushErrors(void);
void c_REVLib_SuppressErrors(bool suppress);
int c_REVLib_ErrorSize(void);
const char* c_REVLib_ErrorFromCode(c_REVLib_ErrorCode code);

}  // extern "C"
