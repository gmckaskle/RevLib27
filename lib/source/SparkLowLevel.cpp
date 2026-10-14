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

#include "rev/SparkLowLevel.h"

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "rev/CANSparkDriver.h"

using namespace rev::spark;
using namespace rev::util;

const uint16_t SparkLowLevel::kAPIMajorVersion = c_Spark_kAPIMajorVersion;
const uint8_t SparkLowLevel::kAPIMinorVersion = c_Spark_kAPIMinorVersion;
const uint8_t SparkLowLevel::kAPIBuildVersion = c_Spark_kAPIBuildVersion;
const uint32_t SparkLowLevel::kAPIVersion = c_Spark_kAPIVersion;

SparkLowLevel::SparkLowLevel(wpi::CANPort canPort, int deviceID, MotorType type,
                             SparkModel model)
    : m_motorType{type},
      m_expectedSparkModel{model},
      m_busID{static_cast<int>(canPort)},
      m_deviceID{deviceID} {
    if (c_Spark_RegisterId(m_busID, deviceID) == c_REVLibError_DuplicateCANId) {
        throw std::runtime_error(std::format(
            "A SparkMax instance has already been created with this device "
            "ID: {} on Bus: {}",
            deviceID, m_busID));
    }

    c_REVLib_ErrorCode status;
    m_sparkMaxHandle = static_cast<void*>(
        c_Spark_Create(m_busID, deviceID, static_cast<c_Spark_MotorType>(type),
                       static_cast<c_Spark_SparkModel>(model), &status));

    if (status == c_REVLibError_CantFindFirmware) {
        // Don't throw exception when no firmware is found. It's possible the
        // device is disconnected and we don't want to stop the program if that
        // is the case.
    } else if (status == c_REVLibError_FirmwareTooOld) {
        throw std::runtime_error(std::format(
            "The firmware version of Bus {} SPARK #{} is too old and "
            "needs to be updated.",
            m_busID, deviceID));
    } else if (status == c_REVLibError_FirmwareTooNew) {
        throw std::runtime_error(std::format(
            "The firmware version of Bus {} SPARK #{} is too new for this "
            "version of REVLib",
            m_busID, deviceID));
    } else if (status == c_REVLibError_SparkFlexBrushedWithoutDock) {
        throw std::runtime_error(std::format(
            "Cannot set motor type to kBrushed for Bus {} SPARK #{} "
            "without a dock connected.",
            m_busID, deviceID));
    }
}

SparkLowLevel::~SparkLowLevel() {
    c_Spark_Close(static_cast<c_Spark_handle>(m_sparkMaxHandle));
    c_Spark_Destroy(static_cast<c_Spark_handle>(m_sparkMaxHandle));
}

uint32_t SparkLowLevel::GetFirmwareVersion() {
    bool isDebugBuild;
    return GetFirmwareVersion(isDebugBuild);
}

uint32_t SparkLowLevel::GetFirmwareVersion(bool& isDebugBuild) {
    c_Spark_FirmwareVersion fwVersion;

    c_Spark_GetFirmwareVersion(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                               &fwVersion);

    isDebugBuild = fwVersion.debugBuild != 0;
    return fwVersion.versionRaw;
}

std::string SparkLowLevel::GetFirmwareString() {
    c_Spark_FirmwareVersion fwVersion;

    c_Spark_GetFirmwareVersion(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                               &fwVersion);

    if (fwVersion.debugBuild != 0) {
        return std::format("v{}.{}.{} Debug Build", fwVersion.major,
                           fwVersion.minor, fwVersion.build);
    } else {
        return std::format("v{}.{}.{}", fwVersion.major, fwVersion.minor,
                           fwVersion.build);
    }
}

std::vector<uint8_t> SparkLowLevel::GetSerialNumber() { return {}; }

wpi::CANPort SparkLowLevel::GetCanPort() const {
    return static_cast<wpi::CANPort>(m_busID);
}

int SparkLowLevel::GetDeviceId() const { return m_deviceID; }

SparkLowLevel::MotorType SparkLowLevel::GetMotorType() { return m_motorType; }

void SparkLowLevel::SetPeriodicFrameTimeout(int timeoutMs) {
    c_Spark_SetPeriodicFrameTimeout(
        static_cast<c_Spark_handle>(m_sparkMaxHandle), timeoutMs);
}

void SparkLowLevel::SetCANMaxRetries(int numRetries) {
    c_Spark_SetCANMaxRetries(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                             numRetries);
}

void SparkLowLevel::SetControlFramePeriodMs(int periodMs) {
    c_Spark_SetControlFramePeriod(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                                  periodMs);
}

Signal<SparkLowLevel::PeriodicStatus0> SparkLowLevel::GetPeriodicStatus0()
    const {
    c_Spark_PeriodicStatus0 cStatus0;
    auto revLibError = static_cast<REVLibError>(c_Spark_GetPeriodicStatus0(
        static_cast<c_Spark_handle>(m_sparkMaxHandle), &cStatus0));

    const PeriodicStatus0 status0{
        .appliedOutput = cStatus0.appliedOutput,
        .voltage = cStatus0.voltage,
        .current = cStatus0.current,
        .motorTemperature = cStatus0.motorTemperature,
        .hardForwardLimitReached = cStatus0.hardForwardLimitReached != 0,
        .hardReverseLimitReached = cStatus0.hardReverseLimitReached != 0,
        .softForwardLimitReached = cStatus0.softForwardLimitReached != 0,
        .softReverseLimitReached = cStatus0.softReverseLimitReached != 0,
        .inverted = cStatus0.inverted != 0,
        .primaryHeartbeatLock = cStatus0.primaryHeartbeatLock != 0,
        .timestamp = cStatus0.timestamp,
    };

    return Signal(status0, revLibError, status0.timestamp);
}

Signal<SparkLowLevel::PeriodicStatus1> SparkLowLevel::GetPeriodicStatus1()
    const {
    c_Spark_PeriodicStatus1 cStatus1;
    auto revLibError = static_cast<REVLibError>(c_Spark_GetPeriodicStatus1(
        static_cast<c_Spark_handle>(m_sparkMaxHandle), &cStatus1));

    const PeriodicStatus1 status1 = {
        .otherFault = cStatus1.otherFault != 0,
        .motorTypeFault = cStatus1.motorTypeFault != 0,
        .sensorFault = cStatus1.sensorFault != 0,
        .canFault = cStatus1.canFault != 0,
        .temperatureFault = cStatus1.temperatureFault != 0,
        .drvFault = cStatus1.drvFault != 0,
        .escEepromFault = cStatus1.escEepromFault != 0,
        .firmwareFault = cStatus1.firmwareFault != 0,
        .brownoutWarning = cStatus1.brownoutWarning != 0,
        .overcurrentWarning = cStatus1.overcurrentWarning != 0,
        .escEepromWarning = cStatus1.escEepromWarning != 0,
        .extEepromWarning = cStatus1.extEepromWarning != 0,
        .sensorWarning = cStatus1.sensorWarning != 0,
        .stallWarning = cStatus1.stallWarning != 0,
        .hasResetWarning = cStatus1.hasResetWarning != 0,
        .otherWarning = cStatus1.otherWarning != 0,
        .otherStickyFault = cStatus1.otherStickyFault != 0,
        .motorTypeStickyFault = cStatus1.motorTypeStickyFault != 0,
        .sensorStickyFault = cStatus1.sensorStickyFault != 0,
        .canStickyFault = cStatus1.canStickyFault != 0,
        .temperatureStickyFault = cStatus1.temperatureStickyFault != 0,
        .drvStickyFault = cStatus1.drvStickyFault != 0,
        .escEepromStickyFault = cStatus1.escEepromStickyFault != 0,
        .firmwareStickyFault = cStatus1.firmwareStickyFault != 0,
        .brownoutStickyWarning = cStatus1.brownoutStickyWarning != 0,
        .overcurrentStickyWarning = cStatus1.overcurrentStickyWarning != 0,
        .escEepromStickyWarning = cStatus1.escEepromStickyWarning != 0,
        .extEepromStickyWarning = cStatus1.extEepromStickyWarning != 0,
        .sensorStickyWarning = cStatus1.sensorStickyWarning != 0,
        .stallStickyWarning = cStatus1.stallStickyWarning != 0,
        .hasResetStickyWarning = cStatus1.hasResetStickyWarning != 0,
        .otherStickyWarning = cStatus1.otherStickyWarning != 0,
        .isFollower = cStatus1.isFollower != 0,
        .timestamp = cStatus1.timestamp,
    };

    return Signal(status1, revLibError, status1.timestamp);
}

Signal<SparkLowLevel::PeriodicStatus2> SparkLowLevel::GetPeriodicStatus2()
    const {
    c_Spark_PeriodicStatus2 cStatus2;
    auto revLibError = static_cast<REVLibError>(c_Spark_GetPeriodicStatus2(
        static_cast<c_Spark_handle>(m_sparkMaxHandle), &cStatus2));

    const PeriodicStatus2 status2 = {
        .primaryEncoderVelocity = cStatus2.primaryEncoderVelocity,
        .primaryEncoderPosition = cStatus2.primaryEncoderPosition,
        .timestamp = cStatus2.timestamp,
    };

    return Signal(status2, revLibError, status2.timestamp);
}

Signal<SparkLowLevel::PeriodicStatus3> SparkLowLevel::GetPeriodicStatus3()
    const {
    c_Spark_PeriodicStatus3 cStatus3;
    auto revLibError = static_cast<REVLibError>(c_Spark_GetPeriodicStatus3(
        static_cast<c_Spark_handle>(m_sparkMaxHandle), &cStatus3));

    const PeriodicStatus3 status3 = {
        .analogVoltage = cStatus3.analogVoltage,
        .analogVelocity = cStatus3.analogVelocity,
        .analogPosition = cStatus3.analogPosition,
        .timestamp = cStatus3.timestamp,
    };

    return Signal(status3, revLibError, status3.timestamp);
}

Signal<SparkLowLevel::PeriodicStatus4> SparkLowLevel::GetPeriodicStatus4()
    const {
    c_Spark_PeriodicStatus4 cStatus4;
    auto revLibError = static_cast<REVLibError>(c_Spark_GetPeriodicStatus4(
        static_cast<c_Spark_handle>(m_sparkMaxHandle), &cStatus4));

    const PeriodicStatus4 status4 = {
        .externalOrAltEncoderVelocity = cStatus4.externalOrAltEncoderVelocity,
        .externalOrAltEncoderPosition = cStatus4.externalOrAltEncoderPosition,
        .timestamp = cStatus4.timestamp,
    };

    return Signal(status4, revLibError, status4.timestamp);
}

Signal<SparkLowLevel::PeriodicStatus5> SparkLowLevel::GetPeriodicStatus5()
    const {
    c_Spark_PeriodicStatus5 cStatus5;
    auto revLibError = static_cast<REVLibError>(c_Spark_GetPeriodicStatus5(
        static_cast<c_Spark_handle>(m_sparkMaxHandle), &cStatus5));

    const PeriodicStatus5 status5 = {
        .dutyCycleEncoderVelocity = cStatus5.dutyCycleEncoderVelocity,
        .dutyCycleEncoderPosition = cStatus5.dutyCycleEncoderPosition,
        .timestamp = cStatus5.timestamp,
    };

    return Signal(status5, revLibError, status5.timestamp);
}

Signal<SparkLowLevel::PeriodicStatus6> SparkLowLevel::GetPeriodicStatus6()
    const {
    c_Spark_PeriodicStatus6 cStatus6;
    auto revLibError = static_cast<REVLibError>(c_Spark_GetPeriodicStatus6(
        static_cast<c_Spark_handle>(m_sparkMaxHandle), &cStatus6));

    const PeriodicStatus6 status6 = {
        .unadjustedDutyCycle = cStatus6.unadjustedDutyCycle,
        .dutyCyclePeriod = cStatus6.dutyCyclePeriod,
        .dutyCycleNoSignal = cStatus6.dutyCycleNoSignal != 0,
        .timestamp = cStatus6.timestamp,
    };

    return Signal(status6, revLibError, status6.timestamp);
}

Signal<SparkLowLevel::PeriodicStatus7> SparkLowLevel::GetPeriodicStatus7()
    const {
    c_Spark_PeriodicStatus7 cStatus7;
    auto revLibError = static_cast<REVLibError>(c_Spark_GetPeriodicStatus7(
        static_cast<c_Spark_handle>(m_sparkMaxHandle), &cStatus7));

    const PeriodicStatus7 status7 = {
        .iAccumulation = cStatus7.iAccumulation,
        .timestamp = cStatus7.timestamp,
    };

    return Signal(status7, revLibError, status7.timestamp);
}

Signal<SparkLowLevel::PeriodicStatus8> SparkLowLevel::GetPeriodicStatus8()
    const {
    c_Spark_PeriodicStatus8 cStatus8;
    auto revLibError = static_cast<REVLibError>(c_Spark_GetPeriodicStatus8(
        static_cast<c_Spark_handle>(m_sparkMaxHandle), &cStatus8));

    const PeriodicStatus8 status8 = {
        .setpoint = cStatus8.setpoint,
        .isAtSetpoint = cStatus8.isAtSetpoint != 0,
        .selectedPidSlot =
            static_cast<ClosedLoopSlot>(cStatus8.selectedPidSlot),
        .timestamp = cStatus8.timestamp,
    };

    return Signal(status8, revLibError, status8.timestamp);
}

Signal<SparkLowLevel::PeriodicStatus9> SparkLowLevel::GetPeriodicStatus9()
    const {
    c_Spark_PeriodicStatus9 cStatus9;
    auto revLibError = static_cast<REVLibError>(c_Spark_GetPeriodicStatus9(
        static_cast<c_Spark_handle>(m_sparkMaxHandle), &cStatus9));

    const PeriodicStatus9 status9 = {
        .maxMotionSetpointPosition = cStatus9.maxmotion_setpoint_position,
        .maxMotionSetpointVelocity = cStatus9.maxmotion_setpoint_velocity,
        .timestamp = cStatus9.timestamp,
    };

    return Signal(status9, revLibError, status9.timestamp);
}

rev::REVLibError SparkLowLevel::SetpointCommand(double value, ControlType ctrl,
                                                ClosedLoopSlot pidSlot,
                                                double arbFeedforward,
                                                int arbFFUnits) {
    auto status = c_Spark_SetpointCommand(
        static_cast<c_Spark_handle>(m_sparkMaxHandle), value,
        static_cast<c_Spark_ControlType>(ctrl), static_cast<int>(pidSlot),
        arbFeedforward, arbFFUnits);
    return static_cast<rev::REVLibError>(status);
}

float SparkLowLevel::GetSafeFloat(float f) {
    if (std::isinf(f) || std::isnan(f)) return 0;
    return f;
}

void SparkLowLevel::CreateSimFaultManager() {
    c_SIM_Spark_CreateSimFaultManager(
        static_cast<c_Spark_handle>(m_sparkMaxHandle));
}
