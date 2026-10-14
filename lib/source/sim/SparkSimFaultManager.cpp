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

#include "rev/sim/SparkSimFaultManager.h"

#include "rev/CANSparkDriver.h"

using namespace rev::spark;

SparkSimFaultManager::SparkSimFaultManager(SparkMax* motor) : m_spark(motor) {
    simDeviceName = std::format("SPARK MAX [{},{}] FAULT MANAGER",
                                static_cast<int>(motor->GetCanPort()),
                                motor->GetDeviceId());
    SetupSimDevice();
}

SparkSimFaultManager::SparkSimFaultManager(SparkFlex* motor) : m_spark(motor) {
    simDeviceName = std::format("SPARK Flex [{},{}] FAULT MANAGER",
                                static_cast<int>(motor->GetCanPort()),
                                motor->GetDeviceId());
    SetupSimDevice();
}

void SparkSimFaultManager::SetupSimDevice() {
    c_SIM_Spark_CreateSimFaultManager(
        static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle));

    m_spark->CreateSimFaultManager();
    wpi::sim::SimDeviceSim faultManager(simDeviceName.c_str());
    m_otherFault = faultManager.GetBoolean("Other Fault");
    m_motorTypeFault = faultManager.GetBoolean("Motor Type Fault");
    m_sensorFault = faultManager.GetBoolean("Sensor Fault");
    m_canFault = faultManager.GetBoolean("CAN Fault");
    m_temperatureFault = faultManager.GetBoolean("Temperature Fault");
    m_drvFault = faultManager.GetBoolean("DRV Fault");
    m_escEepromFault = faultManager.GetBoolean("ESC Eeprom Fault");
    m_firmwareFault = faultManager.GetBoolean("Firmware Fault");
    m_brownoutWarning = faultManager.GetBoolean("Brownout Warning");
    m_overCurrentWarning = faultManager.GetBoolean("Over Current Warning");
    m_escEepromWarning = faultManager.GetBoolean("ESC Eeprom Warning");
    m_extEepromWarning = faultManager.GetBoolean("EXT Eeprom Warning");
    m_sensorWarning = faultManager.GetBoolean("Sensor Warning");
    m_stallWarning = faultManager.GetBoolean("Stall Warning");
    m_hasResetWarning = faultManager.GetBoolean("Has Reset Warning");
    m_otherWarning = faultManager.GetBoolean("Other Warning");
    m_otherStickyFault = faultManager.GetBoolean("Other Sticky Fault");
    m_motorTypeStickyFault = faultManager.GetBoolean("Motor Type Sticky Fault");
    m_sensorStickyFault = faultManager.GetBoolean("Sensor Sticky Fault");
    m_canStickyFault = faultManager.GetBoolean("CAN Sticky Fault");
    m_temperatureStickyFault =
        faultManager.GetBoolean("Temperature Sticky Fault");
    m_drvStickyFault = faultManager.GetBoolean("DRV Sticky Fault");
    m_escEepromStickyFault = faultManager.GetBoolean("ESC Eeprom Sticky Fault");
    m_firmwareStickyFault = faultManager.GetBoolean("Firmware Sticky Fault");
    m_brownoutStickyWarning =
        faultManager.GetBoolean("Brownout Sticky Warning");
    m_overCurrentStickyWarning =
        faultManager.GetBoolean("Over Current Sticky Warning");
    m_escEepromStickyWarning =
        faultManager.GetBoolean("ESC Eeprom Sticky Warning");
    m_extEepromStickyWarning =
        faultManager.GetBoolean("EXT Eeprom Sticky Warning");
    m_sensorStickyWarning = faultManager.GetBoolean("Sensor Sticky Warning");
    m_stallStickyWarning = faultManager.GetBoolean("Stall Sticky Warning");
    m_hasResetStickyWarning =
        faultManager.GetBoolean("Has Reset Sticky Warning");
    m_otherStickyWarning = faultManager.GetBoolean("Other Sticky Warning");
}

void SparkSimFaultManager::SetFaults(const SparkBase::Faults& faults) {
    m_otherFault.Set(faults.other);
    m_motorTypeFault.Set(faults.motorType);
    m_sensorFault.Set(faults.sensor);
    m_canFault.Set(faults.can);
    m_temperatureFault.Set(faults.temperature);
    m_drvFault.Set(faults.gateDriver);
    m_escEepromFault.Set(faults.escEeprom);
    m_firmwareFault.Set(faults.firmware);
}

void SparkSimFaultManager::SetStickyFaults(const SparkBase::Faults& faults) {
    m_otherStickyFault.Set(faults.other);
    m_motorTypeStickyFault.Set(faults.motorType);
    m_sensorStickyFault.Set(faults.sensor);
    m_canStickyFault.Set(faults.can);
    m_temperatureStickyFault.Set(faults.temperature);
    m_drvStickyFault.Set(faults.gateDriver);
    m_escEepromStickyFault.Set(faults.escEeprom);
    m_firmwareStickyFault.Set(faults.firmware);
}

void SparkSimFaultManager::SetWarnings(const SparkBase::Warnings& warnings) {
    m_brownoutWarning.Set(warnings.brownout);
    m_overCurrentWarning.Set(warnings.overcurrent);
    m_escEepromWarning.Set(warnings.escEeprom);
    m_extEepromWarning.Set(warnings.extEeprom);
    m_sensorWarning.Set(warnings.sensor);
    m_stallWarning.Set(warnings.stall);
    m_hasResetWarning.Set(warnings.hasReset);
    m_otherWarning.Set(warnings.other);
}

void SparkSimFaultManager::SetStickyWarnings(
    const SparkBase::Warnings& warnings) {
    m_brownoutStickyWarning.Set(warnings.brownout);
    m_overCurrentStickyWarning.Set(warnings.overcurrent);
    m_escEepromStickyWarning.Set(warnings.escEeprom);
    m_extEepromStickyWarning.Set(warnings.extEeprom);
    m_sensorStickyWarning.Set(warnings.sensor);
    m_stallStickyWarning.Set(warnings.stall);
    m_hasResetStickyWarning.Set(warnings.hasReset);
    m_otherStickyWarning.Set(warnings.other);
}
