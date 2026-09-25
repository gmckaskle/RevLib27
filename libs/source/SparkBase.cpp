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

#include "rev/SparkBase.h"

#include <string>

#include <wpi/system/Errors.hpp>

#include "rev/CANSparkDriver.h"
#include "rev/SparkAbsoluteEncoder.h"
#include "rev/SparkAnalogSensor.h"
#include "rev/SparkClosedLoopController.h"
#include "rev/SparkLimitSwitch.h"
#include "rev/SparkRelativeEncoder.h"
#include "rev/SparkSoftLimit.h"
#include "rev/config/SparkBaseConfig.h"

using namespace rev::spark;
using namespace rev::util;

SparkBase::SparkBase(wpi::CANPort canPort, int deviceID, MotorType type,
                     SparkModel model)
    : SparkLowLevel{canPort, deviceID, type, model},
      m_RelativeEncoder{*this},
      m_AnalogSensor{*this},
      m_AbsoluteEncoder{*this},
      m_ClosedLoopController{*this},
      m_ForwardLimitSwitch{*this, SparkLimitSwitch::Direction::kForward},
      m_ReverseLimitSwitch{*this, SparkLimitSwitch::Direction::kReverse},
      m_ForwardSoftLimit{*this, SparkSoftLimit::Direction::kForward},
      m_ReverseSoftLimit{*this, SparkSoftLimit::Direction::kReverse} {}

void SparkBase::SetThrottle(double throttle) {
    // Only for 'get' API
    m_setpoint = throttle;
    SetpointCommand(throttle, ControlType::kDutyCycle);
}

void SparkBase::SetVoltage(wpi::units::volt_t output) {
    // simple conversion too keep Get() trivial
    // Use GetAppliedOutput() instead of Get() for
    // actual applied duty cycle
    double dOutput = wpi::units::unit_cast<double>(output);
    m_setpoint = dOutput / 12.0;
    SetpointCommand(dOutput, ControlType::kVoltage);
}

double SparkBase::GetThrottle() const { return m_setpoint; }

void SparkBase::SetInverted(bool isInverted) {
    WPILIB_ReportError(
        wpi::warn::Warning,
        "The inversion setting should be set via a SparkMaxConfig "
        "or a SparkFlexConfig object");
    c_Spark_SetInverted(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                        isInverted);
}

bool SparkBase::GetInverted() const {
    WPILIB_ReportError(
        wpi::warn::Warning,
        "The inversion setting should be retrieved via the "
        "configAccessor field of a SparkMax or SparkFlex object");
    uint8_t inverted;
    c_Spark_GetInverted(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                        &inverted);
    return inverted ? true : false;
}

void SparkBase::Disable() { SetThrottle(0.0); }

void SparkBase::StopMotor() { SetThrottle(0.0); }

rev::REVLibError SparkBase::Configure(SparkBaseConfig& config,
                                      rev::ResetMode resetMode,
                                      rev::PersistMode persistMode) {
    std::string flattenedString = config.Flatten();
    rev::REVLibError status = static_cast<rev::REVLibError>(c_Spark_Configure(
        static_cast<c_Spark_handle>(m_sparkMaxHandle), flattenedString.c_str(),
        resetMode == rev::ResetMode::kResetSafeParameters,
        persistMode == rev::PersistMode::kPersistParameters));

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

rev::REVLibError SparkBase::ConfigureAsync(SparkBaseConfig& config,
                                           rev::ResetMode resetMode,
                                           rev::PersistMode persistMode) {
    std::string flattenedString = config.Flatten();
    rev::REVLibError status =
        static_cast<rev::REVLibError>(c_Spark_ConfigureAsync(
            static_cast<c_Spark_handle>(m_sparkMaxHandle),
            flattenedString.c_str(),
            resetMode == rev::ResetMode::kResetSafeParameters,
            persistMode == rev::PersistMode::kPersistParameters));

    return status;
}

SparkRelativeEncoder& SparkBase::GetEncoder() {
    m_relativeEncoderCreated.exchange(true);
    return m_RelativeEncoder;
}

SparkAnalogSensor& SparkBase::GetAnalog() {
    m_analogSensorCreated.exchange(true);
    return m_AnalogSensor;
}

SparkAbsoluteEncoder& SparkBase::GetAbsoluteEncoder() {
    m_absoluteEncoderCreated.exchange(true);
    return m_AbsoluteEncoder;
}

SparkClosedLoopController& SparkBase::GetClosedLoopController() {
    m_closedLoopControllerCreated.exchange(true);
    return m_ClosedLoopController;
}

SparkLimitSwitch& SparkBase::GetForwardLimitSwitch() {
    m_forwardLimitSwitchCreated.exchange(true);
    return m_ForwardLimitSwitch;
}

SparkLimitSwitch& SparkBase::GetReverseLimitSwitch() {
    m_reverseLimitSwitchCreated.exchange(true);
    return m_ReverseLimitSwitch;
}

SparkSoftLimit& SparkBase::GetForwardSoftLimit() {
    m_forwardSoftLimitCreated.exchange(true);
    return m_ForwardSoftLimit;
}

SparkSoftLimit& SparkBase::GetReverseSoftLimit() {
    m_reverseSoftLimitCreated.exchange(true);
    return m_ReverseSoftLimit;
}

uint8_t SparkBase::GetMotorInterface() {
    uint8_t motorInterface;
    c_Spark_GetMotorInterface(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                              &motorInterface);

    return motorInterface;
}

SparkBase::SparkModel SparkBase::GetSparkModel() {
    c_Spark_SparkModel model;
    c_Spark_GetSparkModel(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                          &model);
    return static_cast<SparkBase::SparkModel>(model);
}

rev::REVLibError SparkBase::ResumeFollowerMode() {
    auto status = c_Spark_StartFollowerMode(
        static_cast<c_Spark_handle>(m_sparkMaxHandle));
    return static_cast<rev::REVLibError>(status);
}

rev::REVLibError SparkBase::ResumeFollowerModeAsync() {
    auto status = c_Spark_StartFollowerModeAsync(
        static_cast<c_Spark_handle>(m_sparkMaxHandle));
    return static_cast<rev::REVLibError>(status);
}

rev::REVLibError SparkBase::PauseFollowerMode() {
    auto status =
        c_Spark_StopFollowerMode(static_cast<c_Spark_handle>(m_sparkMaxHandle));
    return static_cast<rev::REVLibError>(status);
}

rev::REVLibError SparkBase::PauseFollowerModeAsync() {
    auto status = c_Spark_StopFollowerModeAsync(
        static_cast<c_Spark_handle>(m_sparkMaxHandle));
    return static_cast<rev::REVLibError>(status);
}

Signal<bool> SparkBase::IsFollower() const {
    return GetPeriodicStatus1().Map(
        [](const auto& s1) { return s1.isFollower; });
}

Signal<bool> SparkBase::HasActiveFault() const {
    return GetFaults().Map(
        [](const auto& faults) { return faults.rawBits != 0; });
}

Signal<bool> SparkBase::HasStickyFault() const {
    return GetStickyFaults().Map(
        [](const auto& faults) { return faults.rawBits != 0; });
}

Signal<bool> SparkBase::HasActiveWarning() const {
    return GetWarnings().Map(
        [](const auto& warnings) { return warnings.rawBits != 0; });
}

Signal<bool> SparkBase::HasStickyWarning() const {
    return GetStickyWarnings().Map(
        [](const auto& warnings) { return warnings.rawBits != 0; });
}

Signal<SparkBase::Faults> SparkBase::GetFaults() const {
    return GetPeriodicStatus1().Map([](const auto& s1) {
        const uint16_t faults =
            0 | s1.otherFault << static_cast<uint8_t>(c_Spark_Fault_kOther) |
            s1.motorTypeFault
                << static_cast<uint8_t>(c_Spark_Fault_kMotorType) |
            s1.sensorFault << static_cast<uint8_t>(c_Spark_Fault_kSensor) |
            s1.canFault << static_cast<uint8_t>(c_Spark_Fault_kCan) |
            s1.temperatureFault
                << static_cast<uint8_t>(c_Spark_Fault_kTemperature) |
            s1.drvFault << static_cast<uint8_t>(c_Spark_Fault_kDrv) |
            s1.escEepromFault
                << static_cast<uint8_t>(c_Spark_Fault_kEscEeprom) |
            s1.firmwareFault << static_cast<uint8_t>(c_Spark_Fault_kFirmware);

        return Faults(faults);
    });
}

Signal<SparkBase::Faults> SparkBase::GetStickyFaults() const {
    return GetPeriodicStatus1().Map([](const auto& s1) {
        const uint16_t faults =
            0 |
            s1.otherStickyFault << static_cast<uint8_t>(c_Spark_Fault_kOther) |
            s1.motorTypeStickyFault
                << static_cast<uint8_t>(c_Spark_Fault_kMotorType) |
            s1.sensorStickyFault
                << static_cast<uint8_t>(c_Spark_Fault_kSensor) |
            s1.canStickyFault << static_cast<uint8_t>(c_Spark_Fault_kCan) |
            s1.temperatureStickyFault
                << static_cast<uint8_t>(c_Spark_Fault_kTemperature) |
            s1.drvStickyFault << static_cast<uint8_t>(c_Spark_Fault_kDrv) |
            s1.escEepromStickyFault
                << static_cast<uint8_t>(c_Spark_Fault_kEscEeprom) |
            s1.firmwareStickyFault
                << static_cast<uint8_t>(c_Spark_Fault_kFirmware);

        return Faults(faults);
    });
}

Signal<SparkBase::Warnings> SparkBase::GetWarnings() const {
    return GetPeriodicStatus1().Map([](const auto& s1) {
        const uint16_t warnings =
            0 |
            s1.brownoutWarning
                << static_cast<uint8_t>(c_Spark_Warning_kBrownout) |
            s1.overcurrentWarning
                << static_cast<uint8_t>(c_Spark_Warning_kOvercurrent) |
            s1.escEepromWarning
                << static_cast<uint8_t>(c_Spark_Warning_kEscEeprom) |
            s1.extEepromWarning
                << static_cast<uint8_t>(c_Spark_Warning_kExtEeprom) |
            s1.sensorWarning << static_cast<uint8_t>(c_Spark_Warning_kSensor) |
            s1.stallWarning << static_cast<uint8_t>(c_Spark_Warning_kStall) |
            s1.hasResetWarning
                << static_cast<uint8_t>(c_Spark_Warning_kHasReset) |
            s1.otherWarning << static_cast<uint8_t>(c_Spark_Warning_kOther);

        return Warnings(warnings);
    });
}

Signal<SparkBase::Warnings> SparkBase::GetStickyWarnings() const {
    return GetPeriodicStatus1().Map([](const auto& s1) {
        const uint16_t warnings =
            0 |
            s1.brownoutStickyWarning
                << static_cast<uint8_t>(c_Spark_Warning_kBrownout) |
            s1.overcurrentStickyWarning
                << static_cast<uint8_t>(c_Spark_Warning_kOvercurrent) |
            s1.escEepromStickyWarning
                << static_cast<uint8_t>(c_Spark_Warning_kEscEeprom) |
            s1.extEepromStickyWarning
                << static_cast<uint8_t>(c_Spark_Warning_kExtEeprom) |
            s1.sensorStickyWarning
                << static_cast<uint8_t>(c_Spark_Warning_kSensor) |
            s1.stallStickyWarning
                << static_cast<uint8_t>(c_Spark_Warning_kStall) |
            s1.hasResetStickyWarning
                << static_cast<uint8_t>(c_Spark_Warning_kHasReset) |
            s1.otherStickyWarning
                << static_cast<uint8_t>(c_Spark_Warning_kOther);

        return Warnings(warnings);
    });
}

Signal<double> SparkBase::GetBusVoltage() const {
    return GetPeriodicStatus0().Map(
        [](const auto& s0) { return static_cast<double>(s0.voltage); });
}

Signal<double> SparkBase::GetAppliedOutput() const {
    return GetPeriodicStatus0().Map(
        [](const auto& s0) { return static_cast<double>(s0.appliedOutput); });
}

Signal<double> SparkBase::GetOutputCurrent() const {
    return GetPeriodicStatus0().Map(
        [](const auto& s0) { return static_cast<double>(s0.current); });
}

Signal<double> SparkBase::GetMotorTemperature() const {
    return GetPeriodicStatus0().Map([](const auto& s0) {
        return static_cast<double>(s0.motorTemperature);
    });
}

rev::REVLibError SparkBase::ClearFaults() {
    auto status =
        c_Spark_ClearFaults(static_cast<c_Spark_handle>(m_sparkMaxHandle));
    return static_cast<rev::REVLibError>(status);
}

rev::REVLibError SparkBase::SetCANTimeout(int milliseconds) {
    auto status = c_Spark_SetCANTimeout(
        static_cast<c_Spark_handle>(m_sparkMaxHandle), milliseconds);
    return static_cast<rev::REVLibError>(status);
}

rev::REVLibError SparkBase::GetLastError() {
    return static_cast<rev::REVLibError>(
        c_Spark_GetLastError(static_cast<c_Spark_handle>(m_sparkMaxHandle)));
}

// Used by the HIL tester
SparkRelativeEncoder SparkBase::GetEncoderEvenIfAlreadyCreated() {
    return GetEncoder();
}
