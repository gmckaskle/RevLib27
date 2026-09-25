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

#include "rev/DetachedEncoder.h"

#include <stdexcept>
#include <string>

#include "rev/CANDetachedEncoderDriver.h"

using namespace rev::detached;
using namespace rev::util;

DetachedEncoder::DetachedEncoder(int busID, int deviceID, EncoderModel model)
    : DetachedEncoderLowLevel{busID, deviceID, model},
      configAccessor{m_detachedEncoderHandle} {}

Signal<double> DetachedEncoder::GetPosition() const {
    return GetPeriodicStatus3().Map(
        [](const auto& s) { return static_cast<double>(s.position); });
}

Signal<double> DetachedEncoder::GetVelocity() const {
    return GetPeriodicStatus4().Map(
        [](const auto& s) { return static_cast<double>(s.velocity); });
}

Signal<double> DetachedEncoder::GetAngle() const {
    return GetPeriodicStatus2().Map(
        [](const auto& s) { return static_cast<double>(s.angle); });
}

Signal<double> DetachedEncoder::GetRawAngle() const {
    return GetPeriodicStatus2().Map(
        [](const auto& s) { return static_cast<double>(s.rawAngle); });
}

rev::REVLibError DetachedEncoder::SetPosition(double position) {
    c_REVLib_ErrorCode status;
    status = c_Detached_SetEncoderPosition(
        static_cast<c_Detached_handle>(m_detachedEncoderHandle), position);
    return static_cast<rev::REVLibError>(status);
}

rev::REVLibError DetachedEncoder::Configure(DetachedEncoderConfig& config,
                                            rev::ResetMode resetMode) {
    std::string flattenedString = config.Flatten();
    rev::REVLibError status =
        static_cast<rev::REVLibError>(c_Detached_Configure(
            static_cast<c_Detached_handle>(m_detachedEncoderHandle),
            flattenedString.c_str(),
            resetMode == rev::ResetMode::kResetSafeParameters));

    if (status != REVLibError::kOk) {
        // Check if fatal error
        if (status == REVLibError::kTimeout ||
            status == REVLibError::kCannotPersistParametersWhileEnabled) {
            return status;
        }

        throw std::runtime_error(
            c_REVLib_ErrorFromCode(static_cast<c_REVLib_ErrorCode>(status)));
    }
    return status;
}

Signal<DetachedEncoder::Faults> DetachedEncoder::GetFaults() const {
    return GetPeriodicStatus1().Map([](const auto& s) {
        const int rawBits =
            (s.unexpectedFault ? c_Detached_FaultMask_kUnexpected : 0) |
            (s.hasResetFault ? c_Detached_FaultMask_kHasReset : 0) |
            (s.canTxFault ? c_Detached_FaultMask_kCanTx : 0) |
            (s.canRxFault ? c_Detached_FaultMask_kCanRx : 0) |
            (s.eepromFault ? c_Detached_FaultMask_kEeprom : 0);
        return Faults{.unexpected = s.unexpectedFault,
                      .hasReset = s.hasResetFault,
                      .canTx = s.canTxFault,
                      .canRx = s.canRxFault,
                      .eeprom = s.eepromFault,
                      .rawBits = rawBits};
    });
}

Signal<DetachedEncoder::Faults> DetachedEncoder::GetStickyFaults() const {
    return GetPeriodicStatus1().Map([](const auto& s) {
        const int rawBits =
            (s.stickyUnexpectedFault ? c_Detached_FaultMask_kUnexpected : 0) |
            (s.stickyHasResetFault ? c_Detached_FaultMask_kHasReset : 0) |
            (s.stickyCanTxFault ? c_Detached_FaultMask_kCanTx : 0) |
            (s.stickyCanRxFault ? c_Detached_FaultMask_kCanRx : 0) |
            (s.stickyEepromFault ? c_Detached_FaultMask_kEeprom : 0);
        return Faults{.unexpected = s.stickyUnexpectedFault,
                      .hasReset = s.stickyHasResetFault,
                      .canTx = s.stickyCanTxFault,
                      .canRx = s.stickyCanRxFault,
                      .eeprom = s.stickyEepromFault,
                      .rawBits = rawBits};
    });
}

rev::REVLibError DetachedEncoder::ClearFaults() {
    auto status = c_Detached_ClearFaults(
        static_cast<c_Detached_handle>(m_detachedEncoderHandle));
    return static_cast<rev::REVLibError>(status);
}
