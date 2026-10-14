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

#include "rev/sim/SparkLimitSwitchSim.h"

#include <cstring>

#include "rev/CANSparkDriver.h"

using namespace rev::spark;

SparkLimitSwitchSim::SparkLimitSwitchSim(SparkMax* motor, bool forward)
    : m_spark(motor), isForward(forward) {
    simDeviceName =
        fmt::format("SPARK MAX [{},{}] LIMIT SWITCH ({})", motor->GetBusId(),
                    motor->GetDeviceId(), (forward ? "FORWARD" : "REVERSE"));
    SetupSimDevice();
}

SparkLimitSwitchSim::SparkLimitSwitchSim(SparkFlex* motor, bool forward)
    : m_spark(motor), isForward(forward) {
    simDeviceName =
        fmt::format("SPARK Flex [{},{}] LIMIT SWITCH ({})", motor->GetBusId(),
                    motor->GetDeviceId(), (forward ? "FORWARD" : "REVERSE"));
    SetupSimDevice();
}

void SparkLimitSwitchSim::SetupSimDevice() {
    if (isForward) {
        c_SIM_Spark_CreateSimForwardLimitSwitch(
            static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle));
    } else {
        c_SIM_Spark_CreateSimReverseLimitSwitch(
            static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle));
    }

    wpi::sim::SimDeviceSim limitSwitchSim(simDeviceName.c_str());
    m_isPressed = limitSwitchSim.GetBoolean("Is Pressed");
    m_isEnabled = limitSwitchSim.GetBoolean("Is Enabled");
}

void SparkLimitSwitchSim::SetPressed(bool state) { m_isPressed.Set(state); }

bool SparkLimitSwitchSim::GetPressed() const { return m_isPressed.Get(); }

bool SparkLimitSwitchSim::GetEnabled() const { return m_isEnabled.Get(); }
