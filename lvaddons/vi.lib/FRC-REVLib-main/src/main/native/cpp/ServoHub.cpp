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

#include "rev/ServoHub.h"

#include <string>

#include <fmt/format.h>

#include "rev/CANServoHubDriver.h"

using namespace rev;
using namespace rev::util;
using namespace rev::servohub;

ServoHub::ServoHub(int busID, int deviceID)
    : ServoHubLowLevel(busID, deviceID),
      configAccessor{m_servoHubHandle},
      m_servoChannels(
          {ServoChannel(ServoChannel::ChannelId::kChannelId0, this),
           ServoChannel(ServoChannel::ChannelId::kChannelId1, this),
           ServoChannel(ServoChannel::ChannelId::kChannelId2, this),
           ServoChannel(ServoChannel::ChannelId::kChannelId3, this),
           ServoChannel(ServoChannel::ChannelId::kChannelId4, this),
           ServoChannel(ServoChannel::ChannelId::kChannelId5, this)}) {}

ServoHub::~ServoHub() {}

REVLibError ServoHub::Configure(ServoHubConfig& config,
                                rev::ResetMode resetMode) {
    REVLibError status = static_cast<REVLibError>(c_ServoHub_Configure(
        static_cast<c_ServoHub_handle>(m_servoHubHandle),
        config.Flatten().c_str(),
        resetMode == rev::ResetMode::kResetSafeParameters));

    if (status != REVLibError::kOk) {
        throw std::runtime_error(
            c_REVLib_ErrorFromCode(static_cast<c_REVLib_ErrorCode>(status)));
    }
    return status;
}

REVLibError ServoHub::ConfigureAsync(ServoHubConfig& config,
                                     rev::ResetMode resetMode) {
    REVLibError status = static_cast<REVLibError>(c_ServoHub_ConfigureAsync(
        static_cast<c_ServoHub_handle>(m_servoHubHandle),
        config.Flatten().c_str(),
        resetMode == rev::ResetMode::kResetSafeParameters));
    return status;
}

Signal<bool> ServoHub::HasActiveFault() const {
    return GetFaults().Map([](const auto& f) { return f.rawBits != 0; });
}

Signal<bool> ServoHub::HasStickyFault() const {
    return GetStickyFaults().Map([](const auto& f) { return f.rawBits != 0; });
}

Signal<bool> ServoHub::HasActiveWarning() const {
    return GetWarnings().Map([](const auto& w) { return w.rawBits != 0; });
}

Signal<bool> ServoHub::HasStickyWarning() const {
    return GetStickyWarnings().Map(
        [](const auto& w) { return w.rawBits != 0; });
}

ServoHub::Faults::Faults(uint16_t faults) {
    rawBits = faults;
    regulatorPowerGood =
        (faults & c_ServoHub_FaultMask_kRegulatorPowerGood) != 0;
    hardware = (faults & c_ServoHub_FaultMask_kHardware) != 0;
    firmware = (faults & c_ServoHub_FaultMask_kFirmware) != 0;
    lowBattery = (faults & c_ServoHub_FaultMask_kLowBattery) != 0;
}

Signal<ServoHub::Faults> ServoHub::GetFaults() const {
    return GetPeriodicStatus1().Map([](const auto& s1) -> Faults {
        const uint16_t faults =
            0 |
            s1.regulatorPowerGoodFault
                << c_ServoHub_FaultOffset_kRegulatorPowerGood |
            s1.hardwareFault << c_ServoHub_FaultOffset_kHardware |
            s1.firmwareFault << c_ServoHub_FaultOffset_kFirmware |
            s1.lowBatteryFault << c_servoHub_FaultOffset_kLowBattery;

        return Faults(faults);
    });
}

Signal<ServoHub::Faults> ServoHub::GetStickyFaults() const {
    return GetPeriodicStatus1().Map([](const auto& s1) -> Faults {
        const uint16_t faults =
            0 |
            s1.stickyRegulatorPowerGoodFault
                << c_ServoHub_FaultOffset_kRegulatorPowerGood |
            s1.stickyHardwareFault << c_ServoHub_FaultOffset_kHardware |
            s1.stickyFirmwareFault << c_ServoHub_FaultOffset_kFirmware |
            s1.stickyLowBatteryFault << c_servoHub_FaultOffset_kLowBattery;

        return Faults(faults);
    });
}

ServoHub::Warnings::Warnings(uint16_t warnings) {
    rawBits = warnings;
    brownout = (warnings & c_ServoHub_WarningMask_kBrownout) != 0;
    canWarning = (warnings & c_ServoHub_WarningMask_kCanWarning) != 0;
    canBusOff = (warnings & c_ServoHub_WarningMask_kCanBusOff) != 0;
    hasReset = (warnings & c_ServoHub_WarningMask_kHasReset) != 0;
    channel0Overcurrent =
        (warnings & c_ServoHub_WarningMask_kChannel0Overcurrent) != 0;
    channel1Overcurrent =
        (warnings & c_ServoHub_WarningMask_kChannel1Overcurrent) != 0;
    channel2Overcurrent =
        (warnings & c_ServoHub_WarningMask_kChannel2Overcurrent) != 0;
    channel3Overcurrent =
        (warnings & c_ServoHub_WarningMask_kChannel3Overcurrent) != 0;
    channel4Overcurrent =
        (warnings & c_ServoHub_WarningMask_kChannel4Overcurrent) != 0;
    channel5Overcurrent =
        (warnings & c_ServoHub_WarningMask_kChannel5Overcurrent) != 0;
}

Signal<ServoHub::Warnings> ServoHub::GetWarnings() const {
    return GetPeriodicStatus1().Map([](const auto& s1) -> Warnings {
        const uint16_t warnings =
            0 | s1.brownout << c_ServoHub_WarningOffset_kBrownout |
            s1.canWarning << c_ServoHub_WarningOffset_kCanWarning |
            s1.canBusOff << c_ServoHub_WarningOffset_kCanBusOff |
            s1.hasReset << c_ServoHub_WarningOffset_kHasReset |
            s1.channel0Overcurrent
                << c_ServoHub_WarningOffset_kChannel0Overcurrent |
            s1.channel1Overcurrent
                << c_ServoHub_WarningOffset_kChannel1Overcurrent |
            s1.channel2Overcurrent
                << c_ServoHub_WarningOffset_kChannel2Overcurrent |
            s1.channel3Overcurrent
                << c_ServoHub_WarningOffset_kChannel3Overcurrent |
            s1.channel4Overcurrent
                << c_ServoHub_WarningOffset_kChannel4Overcurrent |
            s1.channel5Overcurrent
                << c_ServoHub_WarningOffset_kChannel5Overcurrent;

        return Warnings(warnings);
    });
}

Signal<ServoHub::Warnings> ServoHub::GetStickyWarnings() const {
    return GetPeriodicStatus1().Map([](const auto& s1) -> Warnings {
        const uint16_t warnings =
            0 | s1.stickyBrownout << c_ServoHub_WarningOffset_kBrownout |
            s1.stickyCanWarning << c_ServoHub_WarningOffset_kCanWarning |
            s1.stickyCanBusOff << c_ServoHub_WarningOffset_kCanBusOff |
            s1.stickyHasReset << c_ServoHub_WarningOffset_kHasReset |
            s1.stickyChannel0Overcurrent
                << c_ServoHub_WarningOffset_kChannel0Overcurrent |
            s1.stickyChannel1Overcurrent
                << c_ServoHub_WarningOffset_kChannel1Overcurrent |
            s1.stickyChannel2Overcurrent
                << c_ServoHub_WarningOffset_kChannel2Overcurrent |
            s1.stickyChannel3Overcurrent
                << c_ServoHub_WarningOffset_kChannel3Overcurrent |
            s1.stickyChannel4Overcurrent
                << c_ServoHub_WarningOffset_kChannel4Overcurrent |
            s1.stickyChannel5Overcurrent
                << c_ServoHub_WarningOffset_kChannel5Overcurrent;

        return Warnings(warnings);
    });
}

REVLibError ServoHub::ClearFaults() {
    c_REVLib_ErrorCode status = c_ServoHub_ClearFaults(
        static_cast<c_ServoHub_handle>(m_servoHubHandle));
    return status == c_REVLibError_None ? REVLibError::kOk
                                        : REVLibError::kHALError;
}

Signal<double> ServoHub::GetDeviceVoltage() const {
    return GetPeriodicStatus0().Map(
        [](const auto& s0) { return static_cast<double>(s0.voltage); });
}

Signal<double> ServoHub::GetDeviceCurrent() const {
    return GetPeriodicStatus0().Map(
        [](const auto& s0) { return static_cast<double>(s0.deviceCurrent); });
}

Signal<double> ServoHub::GetServoVoltage() const {
    return GetPeriodicStatus0().Map(
        [](const auto& s0) { return static_cast<double>(s0.servoVoltage); });
}

#define TO_SERVO_HUB_CHANNEL(channel) \
    static_cast<c_ServoHub_Channel>(static_cast<int>(channel))

ServoChannel& ServoHub::GetServoChannel(ServoChannel::ChannelId channelId) {
    return m_servoChannels[static_cast<size_t>(channelId)];
}

#define TO_SERVO_HUB_BANK(bank) \
    static_cast<c_ServoHub_Bank>(static_cast<int>(bank))

REVLibError ServoHub::SetBankPulsePeriod(Bank bank, int pulsePeriod_us) {
    auto status = c_ServoHub_SetBankPulsePeriod(
        static_cast<c_ServoHub_handle>(m_servoHubHandle),
        TO_SERVO_HUB_BANK(bank), pulsePeriod_us);
    return static_cast<REVLibError>(status);
}
