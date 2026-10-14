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

#include "rev/sim/SparkMaxAlternateEncoderSim.h"

#include <cstring>

#include "rev/CANSparkDriver.h"

using namespace rev::spark;

SparkMaxAlternateEncoderSim::SparkMaxAlternateEncoderSim(SparkMax* motor)
    : m_spark(motor) {
    simDeviceName = std::format("SPARK MAX [{},{}] ALTERNATE ENCODER",
                                static_cast<int>(motor->GetCanPort()),
                                motor->GetDeviceId());
    SetupSimDevice();
}

void SparkMaxAlternateEncoderSim::SetupSimDevice() {
    c_SIM_Spark_CreateSimExtOrAltEncoder(
        static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle));

    wpi::sim::SimDeviceSim alternateEncoderSim(simDeviceName.c_str());
    m_position = alternateEncoderSim.GetDouble("Position");
    m_velocity = alternateEncoderSim.GetDouble("Velocity");
    m_isInverted = alternateEncoderSim.GetBoolean("Is Inverted");
    m_zeroOffset = alternateEncoderSim.GetDouble("Zero Offset");
}

void SparkMaxAlternateEncoderSim::SetPosition(double position) {
    m_position.Set(position);
}

double SparkMaxAlternateEncoderSim::GetPosition() const {
    return m_position.Get();
}

void SparkMaxAlternateEncoderSim::SetVelocity(double velocity) {
    m_velocity.Set(velocity);
}

double SparkMaxAlternateEncoderSim::GetVelocity() const {
    return m_velocity.Get();
}

void SparkMaxAlternateEncoderSim::SetInverted(bool inverted) {
    m_isInverted.Set(inverted);
}

bool SparkMaxAlternateEncoderSim::GetInverted() const {
    return m_isInverted.Get();
}

void SparkMaxAlternateEncoderSim::SetZeroOffset(double zeroOffset) {
    m_zeroOffset.Set(zeroOffset);
}

double SparkMaxAlternateEncoderSim::GetZeroOffset() const {
    return m_zeroOffset.Get();
}

void SparkMaxAlternateEncoderSim::iterate(double velocity, double dt) {
    double velocityRPM = velocity;
    m_position.Set(m_position.Get() + ((velocityRPM / 60) * dt));
}
