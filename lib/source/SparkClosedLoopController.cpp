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

#include "rev/SparkClosedLoopController.h"

#include "rev/CANSparkDriver.h"
#include "rev/SparkBase.h"

using namespace rev::spark;
using namespace rev::util;

SparkClosedLoopController::SparkClosedLoopController(SparkBase& device)
    : m_device(&device) {}

rev::REVLibError SparkClosedLoopController::SetSetpoint(
    double setpoint, SparkLowLevel::ControlType ctrl, ClosedLoopSlot slot,
    double arbFF, SparkClosedLoopController::ArbFFUnits arbFFUnits) {
    m_controlType = ctrl;

    auto status = c_Spark_SetpointCommand(
        static_cast<c_Spark_handle>(m_device->m_sparkMaxHandle), setpoint,
        static_cast<c_Spark_ControlType>(ctrl), static_cast<uint8_t>(slot),
        arbFF, static_cast<int>(arbFFUnits));

    return static_cast<rev::REVLibError>(status);
}

rev::REVLibError SparkClosedLoopController::SetIAccum(double iAccum) {
    auto status = c_Spark_SetIAccum(
        static_cast<c_Spark_handle>(m_device->m_sparkMaxHandle), iAccum);
    return static_cast<rev::REVLibError>(status);
}

Signal<double> SparkClosedLoopController::GetIAccum() const {
    return m_device->GetPeriodicStatus7().Map(
        [](const auto& s7) { return s7.iAccumulation; });
}

Signal<double> SparkClosedLoopController::GetSetpoint() const {
    return m_device->GetPeriodicStatus8().Map(
        [](const auto& s8) { return s8.setpoint; });
}

Signal<bool> SparkClosedLoopController::IsAtSetpoint() const {
    return m_device->GetPeriodicStatus8().Map(
        [](const auto& s8) { return s8.isAtSetpoint; });
}

Signal<ClosedLoopSlot> SparkClosedLoopController::GetSelectedSlot() const {
    return m_device->GetPeriodicStatus8().Map(
        [](const auto& s8) { return s8.selectedPidSlot; });
}

Signal<double> SparkClosedLoopController::GetMAXMotionSetpointPosition() const {
    return m_device->GetPeriodicStatus9().Map(
        [](const auto& s9) { return s9.maxMotionSetpointPosition; });
}

Signal<double> SparkClosedLoopController::GetMAXMotionSetpointVelocity() const {
    return m_device->GetPeriodicStatus9().Map(
        [](const auto& s9) { return s9.maxMotionSetpointVelocity; });
}
