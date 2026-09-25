/*
 * Copyright (c) 2024-2026 REV Robotics
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

#include "rev/ServoHubLowLevel.h"

#include <format>
#include <stdexcept>
#include <string>

#include "rev/CANServoHubDriver.h"

using namespace rev;
using namespace rev::util;
using namespace rev::servohub;

ServoHubLowLevel::ServoHubLowLevel(wpi::CANPort canPort, int deviceID)
    : m_busID{static_cast<int>(canPort)}, m_deviceID{deviceID} {
    c_REVLib_ErrorCode status{c_ServoHub_RegisterId(m_busID, deviceID)};
    if (status == c_REVLibError_InvalidCANId) {
        throw std::runtime_error(
            std::format("deviceId must be between 0-62; ID: {}", deviceID));
    }

    if (status == c_REVLibError_DuplicateCANId) {
        throw std::runtime_error(
            std::format("A ServoHub instance has already been created on this "
                        "Bus: {} with this device "
                        "ID: {}",
                        m_busID, deviceID));
    }

    m_servoHubHandle =
        static_cast<void*>(c_ServoHub_Create(m_busID, deviceID, &status));

    if (status != c_REVLibError_None) {
        throw std::runtime_error(
            std::format("Error ({}) creating Bus# {} ServoHub#{}.",
                        static_cast<int>(status), m_busID, deviceID));
    }
}

ServoHubLowLevel::~ServoHubLowLevel() {
    c_ServoHub_Close(static_cast<c_ServoHub_handle>(m_servoHubHandle));
    c_ServoHub_Destroy(static_cast<c_ServoHub_handle>(m_servoHubHandle));
}

wpi::CANPort ServoHubLowLevel::GetCanPort() const {
    return static_cast<wpi::CANPort>(m_busID);
}

int ServoHubLowLevel::GetDeviceId() const { return m_deviceID; }

ServoHubLowLevel::FirmwareVersion ServoHubLowLevel::GetFirmwareVersion() const {
    c_ServoHub_FirmwareVersion fwVersion;

    c_ServoHub_GetFirmwareVersion(
        static_cast<c_ServoHub_handle>(m_servoHubHandle), &fwVersion);

    return FirmwareVersion{fwVersion.fwFix, fwVersion.fwMinor, fwVersion.fwYear,
                           fwVersion.hwMinor, fwVersion.hwMajor};
}

std::string ServoHubLowLevel::GetFirmwareVersionString() const {
    FirmwareVersion fwVersion{GetFirmwareVersion()};

    return std::format("fw v{}.{}.{}, hw v{}.{}", fwVersion.firmwareYear,
                       fwVersion.firmwareMinor, fwVersion.firmwareFix,
                       fwVersion.hardwareMajor, fwVersion.hardwareMinor);
}

void ServoHubLowLevel::SetPeriodicFrameTimeout(int timeout_ms) {
    c_ServoHub_SetPeriodicFrameTimeout(
        static_cast<c_ServoHub_handle>(m_servoHubHandle), timeout_ms);
}

REVLibError ServoHubLowLevel::SetCANTimeout(int timeout_ms) {
    auto status = c_ServoHub_SetCANTimeout(
        static_cast<c_ServoHub_handle>(m_servoHubHandle), timeout_ms);
    return static_cast<REVLibError>(status);
}

void ServoHubLowLevel::SetCANMaxRetries(int numRetries) {
    c_ServoHub_SetCANMaxRetries(
        static_cast<c_ServoHub_handle>(m_servoHubHandle), numRetries);
}

void ServoHubLowLevel::SetControlFramePeriodMs(int period_ms) {
    c_ServoHub_SetControlFramePeriod(
        static_cast<c_ServoHub_handle>(m_servoHubHandle), period_ms);
}
int ServoHubLowLevel::GetControlFramePeriodMs() const {
    return c_ServoHub_GetControlFramePeriod(
        static_cast<c_ServoHub_handle>(m_servoHubHandle));
}

Signal<ServoHubLowLevel::PeriodicStatus0> ServoHubLowLevel::GetPeriodicStatus0()
    const {
    c_ServoHub_PeriodicStatus0 c;
    auto revLibError = static_cast<REVLibError>(c_ServoHub_GetPeriodicStatus0(
        static_cast<c_ServoHub_handle>(m_servoHubHandle), &c));

    const PeriodicStatus0 s{
        .voltage = c.voltage,
        .servoVoltage = c.servoVoltage,
        .deviceCurrent = c.deviceCurrent,
        .primaryHeartbeatLock = static_cast<bool>(c.primaryHeartbeatLock),
        .systemEnabled = static_cast<bool>(c.systemEnabled),
        .communicationMode =
            static_cast<CommunicationMode>(c.communicationMode),
        .programmingEnabled = static_cast<bool>(c.programmingEnabled),
        .activelyProgramming = static_cast<bool>(c.activelyProgramming),
        .timestamp = c.timestamp,
    };
    return Signal(s, revLibError, s.timestamp);
}

Signal<ServoHubLowLevel::PeriodicStatus1> ServoHubLowLevel::GetPeriodicStatus1()
    const {
    c_ServoHub_PeriodicStatus1 c;
    auto revLibError = static_cast<REVLibError>(c_ServoHub_GetPeriodicStatus1(
        static_cast<c_ServoHub_handle>(m_servoHubHandle), &c));

    const PeriodicStatus1 s{
        .regulatorPowerGoodFault = static_cast<bool>(c.regulatorPowerGoodFault),
        .brownout = static_cast<bool>(c.brownout),
        .canWarning = static_cast<bool>(c.canWarning),
        .canBusOff = static_cast<bool>(c.canBusOff),
        .hardwareFault = static_cast<bool>(c.hardwareFault),
        .firmwareFault = static_cast<bool>(c.firmwareFault),
        .hasReset = static_cast<bool>(c.hasReset),
        .lowBatteryFault = static_cast<bool>(c.lowBatteryFault),
        .channel0Overcurrent = static_cast<bool>(c.channel0Overcurrent),
        .channel1Overcurrent = static_cast<bool>(c.channel1Overcurrent),
        .channel2Overcurrent = static_cast<bool>(c.channel2Overcurrent),
        .channel3Overcurrent = static_cast<bool>(c.channel3Overcurrent),
        .channel4Overcurrent = static_cast<bool>(c.channel4Overcurrent),
        .channel5Overcurrent = static_cast<bool>(c.channel5Overcurrent),
        .stickyRegulatorPowerGoodFault =
            static_cast<bool>(c.stickyRegulatorPowerGoodFault),
        .stickyBrownout = static_cast<bool>(c.stickyBrownout),
        .stickyCanWarning = static_cast<bool>(c.stickyCanWarning),
        .stickyCanBusOff = static_cast<bool>(c.stickyCanBusOff),
        .stickyHardwareFault = static_cast<bool>(c.stickyHardwareFault),
        .stickyFirmwareFault = static_cast<bool>(c.stickyFirmwareFault),
        .stickyHasReset = static_cast<bool>(c.stickyHasReset),
        .stickyLowBatteryFault = static_cast<bool>(c.stickyLowBatteryFault),
        .stickyChannel0Overcurrent =
            static_cast<bool>(c.stickyChannel0Overcurrent),
        .stickyChannel1Overcurrent =
            static_cast<bool>(c.stickyChannel1Overcurrent),
        .stickyChannel2Overcurrent =
            static_cast<bool>(c.stickyChannel2Overcurrent),
        .stickyChannel3Overcurrent =
            static_cast<bool>(c.stickyChannel3Overcurrent),
        .stickyChannel4Overcurrent =
            static_cast<bool>(c.stickyChannel4Overcurrent),
        .stickyChannel5Overcurrent =
            static_cast<bool>(c.stickyChannel5Overcurrent),
        .timestamp = c.timestamp,
    };
    return Signal(s, revLibError, s.timestamp);
}

Signal<ServoHubLowLevel::PeriodicStatus2> ServoHubLowLevel::GetPeriodicStatus2()
    const {
    c_ServoHub_PeriodicStatus2 c;
    auto revLibError = static_cast<REVLibError>(c_ServoHub_GetPeriodicStatus2(
        static_cast<c_ServoHub_handle>(m_servoHubHandle), &c));

    const PeriodicStatus2 s{
        .channel0PulseWidth = c.channel0PulseWidth,
        .channel1PulseWidth = c.channel1PulseWidth,
        .channel2PulseWidth = c.channel2PulseWidth,
        .channel0Enabled = static_cast<bool>(c.channel0Enabled),
        .channel1Enabled = static_cast<bool>(c.channel1Enabled),
        .channel2Enabled = static_cast<bool>(c.channel2Enabled),
        .channel0OutOfRange = static_cast<bool>(c.channel0OutOfRange),
        .channel1OutOfRange = static_cast<bool>(c.channel1OutOfRange),
        .channel2OutOfRange = static_cast<bool>(c.channel2OutOfRange),
        .timestamp = c.timestamp,
    };
    return Signal(s, revLibError, s.timestamp);
}

Signal<ServoHubLowLevel::PeriodicStatus3> ServoHubLowLevel::GetPeriodicStatus3()
    const {
    c_ServoHub_PeriodicStatus3 c;
    auto revLibError = static_cast<REVLibError>(c_ServoHub_GetPeriodicStatus3(
        static_cast<c_ServoHub_handle>(m_servoHubHandle), &c));

    const PeriodicStatus3 s{
        .channel3PulseWidth = c.channel3PulseWidth,
        .channel4PulseWidth = c.channel4PulseWidth,
        .channel5PulseWidth = c.channel5PulseWidth,
        .channel3Enabled = static_cast<bool>(c.channel3Enabled),
        .channel4Enabled = static_cast<bool>(c.channel4Enabled),
        .channel5Enabled = static_cast<bool>(c.channel5Enabled),
        .channel3OutOfRange = static_cast<bool>(c.channel3OutOfRange),
        .channel4OutOfRange = static_cast<bool>(c.channel4OutOfRange),
        .channel5OutOfRange = static_cast<bool>(c.channel5OutOfRange),
        .timestamp = c.timestamp,
    };
    return Signal(s, revLibError, s.timestamp);
}

Signal<ServoHubLowLevel::PeriodicStatus4> ServoHubLowLevel::GetPeriodicStatus4()
    const {
    c_ServoHub_PeriodicStatus4 c;
    auto revLibError = static_cast<REVLibError>(c_ServoHub_GetPeriodicStatus4(
        static_cast<c_ServoHub_handle>(m_servoHubHandle), &c));

    const PeriodicStatus4 s{
        .channel0Current = c.channel0Current,
        .channel1Current = c.channel1Current,
        .channel2Current = c.channel2Current,
        .channel3Current = c.channel3Current,
        .channel4Current = c.channel4Current,
        .channel5Current = c.channel5Current,
        .timestamp = c.timestamp,
    };
    return Signal(s, revLibError, s.timestamp);
}

void ServoHubLowLevel::CreateSimFaultManager() {
    c_SIM_ServoHub_CreateSimFaultManager(
        static_cast<c_ServoHub_handle>(m_servoHubHandle));
}
