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

#include "rev/sim/SparkRelativeEncoderSim.h"

#include "rev/CANSparkDriver.h"

using namespace rev::spark;

SparkRelativeEncoderSim::SparkRelativeEncoderSim(SparkMax* motor)
    : m_spark(motor) {
    simDeviceName = fmt::format("SPARK MAX [{},{}] RELATIVE ENCODER",
                                motor->GetBusId(), motor->GetDeviceId());
    SetupSimDevice();
}

SparkRelativeEncoderSim::SparkRelativeEncoderSim(SparkFlex* motor)
    : m_spark(motor) {
    simDeviceName = fmt::format("SPARK Flex [{},{}] RELATIVE ENCODER",
                                motor->GetBusId(), motor->GetDeviceId());
    SetupSimDevice();
}

void SparkRelativeEncoderSim::SetupSimDevice() {
    c_SIM_Spark_CreateSimRelativeEncoder(
        static_cast<c_Spark_handle>(m_spark->m_sparkMaxHandle));

    wpi::sim::SimDeviceSim relativeEncoderSim(simDeviceName.c_str());
    m_position = relativeEncoderSim.GetDouble("Position");
    m_velocity = relativeEncoderSim.GetDouble("Velocity");
    m_isInverted = relativeEncoderSim.GetBoolean("Is Inverted");
    m_zeroOffset = relativeEncoderSim.GetDouble("Zero Offset");
    m_positionConversionFactor =
        relativeEncoderSim.GetDouble("Position Conversion Factor");
    m_velocityConversionFactor =
        relativeEncoderSim.GetDouble("Velocity Conversion Factor");
}

void SparkRelativeEncoderSim::SetPosition(double position) {
    m_position.Set(position);
}

double SparkRelativeEncoderSim::GetPosition() const { return m_position.Get(); }

void SparkRelativeEncoderSim::SetVelocity(double velocity) {
    m_velocity.Set(velocity);
}

double SparkRelativeEncoderSim::GetVelocity() const { return m_velocity.Get(); }

void SparkRelativeEncoderSim::SetInverted(bool inverted) {
    m_isInverted.Set(inverted);
}

bool SparkRelativeEncoderSim::GetInverted() const { return m_isInverted.Get(); }

void SparkRelativeEncoderSim::SetZeroOffset(double zeroOffset) {
    m_zeroOffset.Set(zeroOffset);
}

double SparkRelativeEncoderSim::GetZeroOffset() const {
    return m_zeroOffset.Get();
}

double SparkRelativeEncoderSim::GetPositionConversionFactor() const {
    return m_positionConversionFactor.Get();
}

double SparkRelativeEncoderSim::GetVelocityConversionFactor() const {
    return m_velocityConversionFactor.Get();
}

void SparkRelativeEncoderSim::iterate(double velocity, double dt) {
    double velocityRPM = velocity / GetVelocityConversionFactor();
    m_position.Set(m_position.Get() +
                   ((velocityRPM / 60) * dt) * GetPositionConversionFactor());
}
