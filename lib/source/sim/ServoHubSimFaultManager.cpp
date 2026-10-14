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

#include "rev/sim/ServoHubSimFaultManager.h"

#include <format>

#include <wpi/simulation/SimDeviceSim.hpp>

#include "rev/CANServoHubDriver.h"

using namespace rev::servohub;

ServoHubSimFaultManager::ServoHubSimFaultManager(ServoHub* servoHub)
    : m_servoHub(servoHub),
      simDeviceName{std::format("Servo Hub [{},{}] FAULT MANAGER",
                                static_cast<int>(servoHub->GetCanPort()),
                                servoHub->GetDeviceId())} {
    SetupSimDevice();
}

void ServoHubSimFaultManager::SetupSimDevice() {
    c_SIM_ServoHub_CreateSimFaultManager(
        static_cast<c_ServoHub_handle>(m_servoHub->m_servoHubHandle));

    m_servoHub->CreateSimFaultManager();
    wpi::sim::SimDeviceSim faultManager(simDeviceName.c_str());

    m_regulatorPGoodFault = faultManager.GetBoolean("Regulator P Good Fault");
    m_firmwareFault = faultManager.GetBoolean("Firmware Fault");
    m_hardwareFault = faultManager.GetBoolean("Hardware Fault");
    m_lowBatteryFault = faultManager.GetBoolean("Low Battery Fault");

    m_regulatorPGoodStickyFault =
        faultManager.GetBoolean("Regulator P Good Sticky Fault");
    m_firmwareStickyFault = faultManager.GetBoolean("Firmware Sticky Fault");
    m_hardwareStickyFault = faultManager.GetBoolean("Hardware Sticky Fault");
    m_lowBatteryStickyFault =
        faultManager.GetBoolean("Low Battery Sticky Fault");

    m_brownoutWarning = faultManager.GetBoolean("Brownout Warning");
    m_canWarning = faultManager.GetBoolean("CAN Warning");
    m_canBusOffWarning = faultManager.GetBoolean("CAN Bus Off Warning");
    m_hasResetWarning = faultManager.GetBoolean("Has Reset Warning");
    m_channel0OvercurrentWarning =
        faultManager.GetBoolean("Channel 0 Over Current Warning");
    m_channel1OvercurrentWarning =
        faultManager.GetBoolean("Channel 1 Over Current Warning");
    m_channel2OvercurrentWarning =
        faultManager.GetBoolean("Channel 2 Over Current Warning");
    m_channel3OvercurrentWarning =
        faultManager.GetBoolean("Channel 3 Over Current Warning");
    m_channel4OvercurrentWarning =
        faultManager.GetBoolean("Channel 4 Over Current Warning");
    m_channel5OvercurrentWarning =
        faultManager.GetBoolean("Channel 5 Over Current Warning");

    m_brownoutStickyWarning =
        faultManager.GetBoolean("Brownout Sticky Warning");
    m_canStickyWarning = faultManager.GetBoolean("CAN Sticky Warning");
    m_canBusOffStickyWarning =
        faultManager.GetBoolean("CAN Bus Off Sticky Warning");
    m_hasResetStickyWarning =
        faultManager.GetBoolean("Has Reset Sticky Warning");
    m_channel0OvercurrentStickyWarning =
        faultManager.GetBoolean("Channel 0 Over Current Sticky Warning");
    m_channel1OvercurrentStickyWarning =
        faultManager.GetBoolean("Channel 1 Over Current Sticky Warning");
    m_channel2OvercurrentStickyWarning =
        faultManager.GetBoolean("Channel 2 Over Current Sticky Warning");
    m_channel3OvercurrentStickyWarning =
        faultManager.GetBoolean("Channel 3 Over Current Sticky Warning");
    m_channel4OvercurrentStickyWarning =
        faultManager.GetBoolean("Channel 4 Over Current Sticky Warning");
    m_channel5OvercurrentStickyWarning =
        faultManager.GetBoolean("Channel 5 Over Current Sticky Warning");
}

void ServoHubSimFaultManager::SetFaults(const ServoHub::Faults& faults) {
    m_regulatorPGoodFault.Set(faults.regulatorPowerGood);
    m_firmwareFault.Set(faults.firmware);
    m_hardwareFault.Set(faults.hardware);
    m_lowBatteryFault.Set(faults.lowBattery);
}

void ServoHubSimFaultManager::SetStickyFaults(const ServoHub::Faults& faults) {
    m_regulatorPGoodStickyFault.Set(faults.regulatorPowerGood);
    m_firmwareStickyFault.Set(faults.firmware);
    m_hardwareStickyFault.Set(faults.hardware);
    m_lowBatteryStickyFault.Set(faults.lowBattery);
}

void ServoHubSimFaultManager::SetWarnings(const ServoHub::Warnings& warnings) {
    m_brownoutWarning.Set(warnings.brownout);
    m_canWarning.Set(warnings.canWarning);
    m_canBusOffWarning.Set(warnings.canBusOff);
    m_hasResetWarning.Set(warnings.hasReset);
    m_channel0OvercurrentWarning.Set(warnings.channel0Overcurrent);
    m_channel1OvercurrentWarning.Set(warnings.channel1Overcurrent);
    m_channel2OvercurrentWarning.Set(warnings.channel2Overcurrent);
    m_channel3OvercurrentWarning.Set(warnings.channel3Overcurrent);
    m_channel4OvercurrentWarning.Set(warnings.channel4Overcurrent);
    m_channel5OvercurrentWarning.Set(warnings.channel5Overcurrent);
}

void ServoHubSimFaultManager::SetStickyWarnings(
    const ServoHub::Warnings& warnings) {
    m_brownoutStickyWarning.Set(warnings.brownout);
    m_canStickyWarning.Set(warnings.canWarning);
    m_canBusOffStickyWarning.Set(warnings.canBusOff);
    m_hasResetStickyWarning.Set(warnings.hasReset);
    m_channel0OvercurrentStickyWarning.Set(warnings.channel0Overcurrent);
    m_channel1OvercurrentStickyWarning.Set(warnings.channel1Overcurrent);
    m_channel2OvercurrentStickyWarning.Set(warnings.channel2Overcurrent);
    m_channel3OvercurrentStickyWarning.Set(warnings.channel3Overcurrent);
    m_channel4OvercurrentStickyWarning.Set(warnings.channel4Overcurrent);
    m_channel5OvercurrentStickyWarning.Set(warnings.channel5Overcurrent);
}
