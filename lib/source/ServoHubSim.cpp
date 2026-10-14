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

#include "rev/ServoHubSim.h"

#include <format>
#include <memory>
#include <stdexcept>

#include <wpi/simulation/SimDeviceSim.hpp>

#include "rev/CANServoHubDriver.h"
#include "rev/sim/ServoHubSimFaultManager.h"

using namespace rev::servohub;

ServoHubSim::ServoHubSim(ServoHub* servoHub)
    : m_servoHub(servoHub),
      m_deviceName{std::format("Servo Hub [{},{}]",
                               static_cast<int>(servoHub->GetCanPort()),
                               servoHub->GetDeviceId())} {
    wpi::sim::SimDeviceSim servoHubSim(m_deviceName.c_str());

    m_deviceVoltage = servoHubSim.GetDouble("Device Voltage");
    m_deviceCurrent = servoHubSim.GetDouble("Device Current");
    m_servoVoltage = servoHubSim.GetDouble("Servo Voltage");
    m_bankPulsePeriods[0] = servoHubSim.GetInt("Bank 0-2 Pulse Period");
    m_bankPulsePeriods[1] = servoHubSim.GetInt("Bank 3-5 Pulse Period");
}

double ServoHubSim::GetDeviceVoltage() const { return m_deviceVoltage.Get(); }

void ServoHubSim::SetDeviceVoltage(double voltage) {
    m_deviceVoltage.Set(voltage);
}

double ServoHubSim::GetDeviceCurrent() const { return m_deviceCurrent.Get(); }

void ServoHubSim::SetDeviceCurrent(double current) {
    m_deviceCurrent.Set(current);
}

double ServoHubSim::GetServoVoltage() const { return m_servoVoltage.Get(); }
void ServoHubSim::SetServoVoltage(double voltage) {
    m_servoVoltage.Set(voltage);
}

int ServoHubSim::GetBankPulsePeriod(ServoHub::Bank bank) const {
    return m_bankPulsePeriods[bank == ServoHub::Bank::kBank0_2 ? 0 : 1].Get();
}

void ServoHubSim::SetBankPulsePeriod(ServoHub::Bank bank, int pulsePeriod_us) {
    m_bankPulsePeriods[bank == ServoHub::Bank::kBank0_2 ? 0 : 1].Set(
        pulsePeriod_us);
}

void ServoHubSim::enable() { m_enable = std::make_unique<bool>(true); }

void ServoHubSim::disable() { m_enable = std::make_unique<bool>(false); }

void ServoHubSim::useDriverStationEnable() { m_enable.reset(); }

ServoHubSimFaultManager ServoHubSim::GetFaultManager() {
    c_SIM_ServoHub_CreateSimFaultManager(
        static_cast<c_ServoHub_handle>(m_servoHub->m_servoHubHandle));

    return ServoHubSimFaultManager(m_servoHub);
}
