/*
 * Copyright (c) 2025-2026 REV Robotics
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

#include "rev/DetachedEncoderLowLevel.h"

#include <format>
#include <stdexcept>

#include "rev/CANDetachedEncoderDriver.h"
#include "rev/REVLibError.h"
#include "rev/util/Signal.h"

using namespace rev::detached;
using namespace rev::util;

DetachedEncoderLowLevel::DetachedEncoderLowLevel(wpi::CANPort canPort,
                                                 int deviceID,
                                                 EncoderModel model)
    : m_busID{static_cast<int>(canPort)}, m_deviceID{deviceID} {
    if (c_Detached_RegisterId(m_busID, deviceID) ==
        c_REVLibError_DuplicateCANId) {
        throw std::runtime_error(
            std::format("A Detached Encoder instance has already been created "
                        "on Bus {} with this device ID: {}",
                        m_busID, deviceID));
    }

    c_REVLib_ErrorCode status;
    m_detachedEncoderHandle = static_cast<void*>(c_Detached_Create(
        m_busID, deviceID, static_cast<c_Detached_EncoderModel>(model),
        &status));

    if (status == c_REVLibError_CantFindFirmware) {
        // Don't throw exception when no firmware is found. It's possible the
        // device is disconnected and we don't want to stop the program if that
        // is the case.
    } else if (status == c_REVLibError_FirmwareTooOld) {
        throw std::runtime_error(std::format(
            "The firmware version of Bus {} Detached Encoder #{} is too"
            "old and needs to be updated.",
            m_busID, deviceID));
    } else if (status == c_REVLibError_FirmwareTooNew) {
        throw std::runtime_error(std::format(
            "The firmware version of Bus {} Detached Encoder #{} is too "
            "new for this version of REVLib",
            m_busID, deviceID));
    } else if (status != c_REVLibError_None) {
        throw std::runtime_error(
            std::format("Error ({}) creating Bus {} Detached Encoder #{}.",
                        static_cast<int>(status), m_busID, deviceID));
    }
}

DetachedEncoderLowLevel::~DetachedEncoderLowLevel() {
    c_Detached_Close(static_cast<c_Detached_handle>(m_detachedEncoderHandle));
    c_Detached_Destroy(static_cast<c_Detached_handle>(m_detachedEncoderHandle));
}

wpi::CANPort DetachedEncoderLowLevel::GetCanPort() const {
    return static_cast<wpi::CANPort>(m_busID);
}

int DetachedEncoderLowLevel::GetDeviceId() const { return m_deviceID; }

DetachedEncoderLowLevel::EncoderModel DetachedEncoderLowLevel::GetEncoderModel()
    const {
    c_Detached_EncoderModel model;
    c_Detached_GetEncoderModel(
        static_cast<c_Detached_handle>(m_detachedEncoderHandle), &model);
    return static_cast<EncoderModel>(model);
}

DetachedEncoderLowLevel::FirmwareVersion
DetachedEncoderLowLevel::GetFirmwareVersion() const {
    c_Detached_FirmwareVersion cVersion;
    c_Detached_GetFirmwareVersion(
        static_cast<c_Detached_handle>(m_detachedEncoderHandle), &cVersion);

    FirmwareVersion version;
    version.year = cVersion.fwYear;
    version.minor = cVersion.fwMinor;
    version.fix = cVersion.fwFix;
    version.prerelease = cVersion.fwPrerelease;
    version.hardwareMajor = cVersion.hwMajor;
    version.hardwareMinor = cVersion.hwMinor;

    return version;
}

DetachedEncoderLowLevel::PeriodicStatus0
DetachedEncoderLowLevel::GetPeriodicStatus0() const {
    c_Detached_PeriodicStatus0 cStatus0;
    c_Detached_GetPeriodicStatus0(
        static_cast<c_Detached_handle>(m_detachedEncoderHandle), &cStatus0);

    PeriodicStatus0 status0;
    status0.model = static_cast<EncoderModel>(cStatus0.encoderModel);

    return status0;
}

Signal<DetachedEncoderLowLevel::PeriodicStatus1>
DetachedEncoderLowLevel::GetPeriodicStatus1() const {
    c_Detached_PeriodicStatus1 cStatus1;
    auto revLibError = static_cast<REVLibError>(c_Detached_GetPeriodicStatus1(
        static_cast<c_Detached_handle>(m_detachedEncoderHandle), &cStatus1));

    const PeriodicStatus1 status1 = {
        .unexpectedFault = cStatus1.unexpectedFault != 0,
        .hasResetFault = cStatus1.hasResetFault != 0,
        .canTxFault = cStatus1.canTxFault != 0,
        .canRxFault = cStatus1.canRxFault != 0,
        .eepromFault = cStatus1.eepromFault != 0,
        .stickyUnexpectedFault = cStatus1.unexpectedStickyFault != 0,
        .stickyHasResetFault = cStatus1.hasResetStickyFault != 0,
        .stickyCanTxFault = cStatus1.canTxStickyFault != 0,
        .stickyCanRxFault = cStatus1.canRxStickyFault != 0,
        .stickyEepromFault = cStatus1.eepromStickyFault != 0,
        .timestamp = cStatus1.timestamp,
    };
    return Signal(status1, revLibError, status1.timestamp);
}

Signal<DetachedEncoderLowLevel::PeriodicStatus2>
DetachedEncoderLowLevel::GetPeriodicStatus2() const {
    c_Detached_PeriodicStatus2 cstatus2;
    auto revLibError = static_cast<REVLibError>(c_Detached_GetPeriodicStatus2(
        static_cast<c_Detached_handle>(m_detachedEncoderHandle), &cstatus2));

    const PeriodicStatus2 status2 = {
        .rawAngle = cstatus2.rawAbsoluteAngle,
        .angle = cstatus2.absoluteAngle,
        .timestamp = cstatus2.timestamp,
    };
    return Signal(status2, revLibError, status2.timestamp);
}

Signal<DetachedEncoderLowLevel::PeriodicStatus3>
DetachedEncoderLowLevel::GetPeriodicStatus3() const {
    c_Detached_PeriodicStatus3 cStatus3;
    auto revLibError = static_cast<REVLibError>(c_Detached_GetPeriodicStatus3(
        static_cast<c_Detached_handle>(m_detachedEncoderHandle), &cStatus3));

    const PeriodicStatus3 status3 = {
        .position = cStatus3.relativePosition,
        .timestamp = cStatus3.timestamp,
    };
    return Signal(status3, revLibError, status3.timestamp);
}

Signal<DetachedEncoderLowLevel::PeriodicStatus4>
DetachedEncoderLowLevel::GetPeriodicStatus4() const {
    c_Detached_PeriodicStatus4 cStatus4;
    auto revLibError = static_cast<REVLibError>(c_Detached_GetPeriodicStatus4(
        static_cast<c_Detached_handle>(m_detachedEncoderHandle), &cStatus4));

    const PeriodicStatus4 status4 = {
        .velocity = cStatus4.encoderVelocity,
        .timestamp = cStatus4.timestamp,
    };
    return Signal(status4, revLibError, status4.timestamp);
}

void DetachedEncoderLowLevel::CreateSimFaultManager() {
    c_SIM_Detached_CreateSimFaultManager(
        static_cast<c_Detached_handle>(m_detachedEncoderHandle));
}
