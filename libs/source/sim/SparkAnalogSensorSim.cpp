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

#include "rev/sim/SparkAnalogSensorSim.h"

#include <cstring>

#include "rev/CANSparkDriver.h"

using namespace rev::spark;

SparkAnalogSensorSim::SparkAnalogSensorSim(SparkMax* motor) : m_spark(motor) {
    simDeviceName = std::format("SPARK MAX [{},{}] ANALOG SENSOR",
                                static_cast<int>(motor->GetCanPort()),
                                motor->GetDeviceId());
    SetupSimDevice();
}

SparkAnalogSensorSim::SparkAnalogSensorSim(SparkFlex* motor) : m_spark(motor) {
    simDeviceName = std::format("SPARK Flex [{},{}] ANALOG SENSOR",
                                static_cast<int>(motor->GetCanPort()),
                                motor->GetDeviceId());
    SetupSimDevice();
}

void SparkAnalogSensorSim::SetupSimDevice() {
    c_SIM_Spark_CreateSimAnalogSensor(
        static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle));

    wpi::sim::SimDeviceSim analogSensorSim(simDeviceName.c_str());
    m_voltage = analogSensorSim.GetDouble("Voltage");
    m_position = analogSensorSim.GetDouble("Position");
    m_velocity = analogSensorSim.GetDouble("Velocity");
    m_isInverted = analogSensorSim.GetBoolean("Is Inverted");
}

void SparkAnalogSensorSim::SetVoltage(double voltage) {
    m_voltage.Set(voltage);
}

double SparkAnalogSensorSim::GetVoltage() const { return m_voltage.Get(); }

void SparkAnalogSensorSim::SetPosition(double position) {
    m_position.Set(position);
}

double SparkAnalogSensorSim::GetPosition() const { return m_position.Get(); }

void SparkAnalogSensorSim::SetVelocity(double velocity) {
    m_velocity.Set(velocity);
}

double SparkAnalogSensorSim::GetVelocity() const { return m_velocity.Get(); }

void SparkAnalogSensorSim::SetInverted(bool inverted) {
    m_isInverted.Set(inverted);
}

bool SparkAnalogSensorSim::GetInverted() const { return m_isInverted.Get(); }

void SparkAnalogSensorSim::iterate(double velocity, double dt) {
    double velocityRPM = velocity;
    m_position.Set(m_position.Get() + ((velocityRPM / 60) * dt));
}
