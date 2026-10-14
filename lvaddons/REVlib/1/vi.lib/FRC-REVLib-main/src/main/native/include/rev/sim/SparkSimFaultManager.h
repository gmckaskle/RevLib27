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

#ifndef FRC_REVLIB_SRC_MAIN_NATIVE_INCLUDE_REV_SIM_SPARKSIMFAULTMANAGER_H_
#define FRC_REVLIB_SRC_MAIN_NATIVE_INCLUDE_REV_SIM_SPARKSIMFAULTMANAGER_H_

#include <string>

#include <wpi/hal/SimDevice.h>
#include <wpi/simulation/SimDeviceSim.hpp>

#include "rev/SparkBase.h"
#include "rev/SparkFlex.h"
#include "rev/SparkMax.h"

namespace rev::spark {

class SparkSimFaultManager {
public:
    explicit SparkSimFaultManager(SparkMax* motor);
    explicit SparkSimFaultManager(SparkFlex* motor);

    void SetFaults(const SparkBase::Faults& faults);
    void SetStickyFaults(const SparkBase::Faults& faults);
    void SetWarnings(const SparkBase::Warnings& warnings);
    void SetStickyWarnings(const SparkBase::Warnings& warnings);

private:
    void SetupSimDevice();

    wpi::hal::SimBoolean m_otherFault;
    wpi::hal::SimBoolean m_motorTypeFault;
    wpi::hal::SimBoolean m_sensorFault;
    wpi::hal::SimBoolean m_canFault;
    wpi::hal::SimBoolean m_temperatureFault;
    wpi::hal::SimBoolean m_drvFault;
    wpi::hal::SimBoolean m_escEepromFault;
    wpi::hal::SimBoolean m_firmwareFault;
    wpi::hal::SimBoolean m_brownoutWarning;
    wpi::hal::SimBoolean m_overCurrentWarning;
    wpi::hal::SimBoolean m_escEepromWarning;
    wpi::hal::SimBoolean m_extEepromWarning;
    wpi::hal::SimBoolean m_sensorWarning;
    wpi::hal::SimBoolean m_stallWarning;
    wpi::hal::SimBoolean m_hasResetWarning;
    wpi::hal::SimBoolean m_otherWarning;
    wpi::hal::SimBoolean m_otherStickyFault;
    wpi::hal::SimBoolean m_motorTypeStickyFault;
    wpi::hal::SimBoolean m_sensorStickyFault;
    wpi::hal::SimBoolean m_canStickyFault;
    wpi::hal::SimBoolean m_temperatureStickyFault;
    wpi::hal::SimBoolean m_drvStickyFault;
    wpi::hal::SimBoolean m_escEepromStickyFault;
    wpi::hal::SimBoolean m_firmwareStickyFault;
    wpi::hal::SimBoolean m_brownoutStickyWarning;
    wpi::hal::SimBoolean m_overCurrentStickyWarning;
    wpi::hal::SimBoolean m_escEepromStickyWarning;
    wpi::hal::SimBoolean m_extEepromStickyWarning;
    wpi::hal::SimBoolean m_sensorStickyWarning;
    wpi::hal::SimBoolean m_stallStickyWarning;
    wpi::hal::SimBoolean m_hasResetStickyWarning;
    wpi::hal::SimBoolean m_otherStickyWarning;

    SparkBase* m_spark;
    std::string simDeviceName;
};

}  // namespace rev::spark

#endif  // FRC_REVLIB_SRC_MAIN_NATIVE_INCLUDE_REV_SIM_SPARKSIMFAULTMANAGER_H_
