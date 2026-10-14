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

#include "rev/sim/SparkAbsoluteEncoderSim.h"

#include <cstring>

#include "rev/CANSparkDriver.h"

using namespace rev::spark;

SparkAbsoluteEncoderSim::SparkAbsoluteEncoderSim(SparkMax* motor)
    : m_spark(motor) {
    simDeviceName = std::format("SPARK MAX [{},{}] ABSOLUTE ENCODER",
                                static_cast<int>(motor->GetCanPort()),
                                motor->GetDeviceId());
    SetupSimDevice();
}

SparkAbsoluteEncoderSim::SparkAbsoluteEncoderSim(SparkFlex* motor)
    : m_spark(motor) {
    simDeviceName = std::format("SPARK Flex [{},{}] ABSOLUTE ENCODER",
                                static_cast<int>(motor->GetCanPort()),
                                motor->GetDeviceId());
    SetupSimDevice();
}

void SparkAbsoluteEncoderSim::SetupSimDevice() {
    c_SIM_Spark_CreateSimAbsoluteEncoder(
        static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle));

    wpi::sim::SimDeviceSim absoluteEncoderSim(simDeviceName.c_str());
    m_position = absoluteEncoderSim.GetDouble("Position");
    m_velocity = absoluteEncoderSim.GetDouble("Velocity");
    m_isInverted = absoluteEncoderSim.GetBoolean("Is Inverted");
    m_zeroOffset = absoluteEncoderSim.GetDouble("Zero Offset");
}

void SparkAbsoluteEncoderSim::SetPosition(double position) {
    m_position.Set(position);
}

double SparkAbsoluteEncoderSim::GetPosition() const { return m_position.Get(); }

void SparkAbsoluteEncoderSim::SetVelocity(double velocity) {
    m_velocity.Set(velocity);
}

double SparkAbsoluteEncoderSim::GetVelocity() const { return m_velocity.Get(); }

void SparkAbsoluteEncoderSim::SetInverted(bool inverted) {
    m_isInverted.Set(inverted);
}

bool SparkAbsoluteEncoderSim::GetInverted() const { return m_isInverted.Get(); }

void SparkAbsoluteEncoderSim::SetZeroOffset(double zeroOffset) {
    m_zeroOffset.Set(zeroOffset);
}

double SparkAbsoluteEncoderSim::GetZeroOffset() const {
    return m_zeroOffset.Get();
}

void SparkAbsoluteEncoderSim::iterate(double velocity, double dt) {
    double velocityRPM = velocity;
    m_position.Set(m_position.Get() + ((velocityRPM / 60) * dt));
}
