/*
 * Copyright (c) 2025-2026 REV Robotics
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

#include "rev/sim/DetachedEncoderSim.h"

#include <cmath>

#include <fmt/format.h>
#include <wpi/simulation/SimDeviceSim.hpp>

#include "rev/CANDetachedEncoderDriver.h"

using namespace rev::detached;

DetachedEncoderSim::DetachedEncoderSim(DetachedEncoder* encoder)
    : m_encoder(encoder),
      simDeviceName{fmt::format("Detached Encoder [{},{}]", encoder->GetBusId(),
                                encoder->GetDeviceId())} {
    wpi::sim::SimDeviceSim encoderSim(simDeviceName.c_str());
    m_position = encoderSim.GetDouble("Position");
    m_velocity = encoderSim.GetDouble("Velocity");
    m_angle = encoderSim.GetDouble("Angle");
    m_rawAngle = encoderSim.GetDouble("Raw Angle");
    m_isInverted = encoderSim.GetBoolean("Is Inverted");
    m_zeroOffset = encoderSim.GetDouble("Zero Offset");
    m_positionConversionFactor =
        encoderSim.GetDouble("Position Conversion Factor");
    m_velocityConversionFactor =
        encoderSim.GetDouble("Velocity Conversion Factor");
}

void rev::detached::DetachedEncoderSim::SetPosition(double position) {
    m_position.Set(position);
}

double rev::detached::DetachedEncoderSim::GetPosition() const {
    return m_position.Get();
}

void rev::detached::DetachedEncoderSim::SetVelocity(double velocity) {
    m_velocity.Set(velocity);
}

double rev::detached::DetachedEncoderSim::GetVelocity() const {
    return m_velocity.Get();
}

void DetachedEncoderSim::SetInverted(bool inverted) {
    m_isInverted.Set(inverted);
}

void rev::detached::DetachedEncoderSim::SetAngle(double angle) {
    m_angle.Set(angle);
}

double rev::detached::DetachedEncoderSim::GetAngle() const {
    return m_angle.Get();
}

void rev::detached::DetachedEncoderSim::SetRawAngle(double rawAngle) {
    m_rawAngle.Set(rawAngle);
}

double rev::detached::DetachedEncoderSim::GetRawAngle() const {
    return m_rawAngle.Get();
}

bool DetachedEncoderSim::GetInverted() const { return m_isInverted.Get(); }

void DetachedEncoderSim::SetZeroOffset(double zeroOffset) {
    m_zeroOffset.Set(zeroOffset);
}

double DetachedEncoderSim::GetZeroOffset() const { return m_zeroOffset.Get(); }

double rev::detached::DetachedEncoderSim::GetPositionConversionFactor() const {
    return m_positionConversionFactor.Get();
}

double rev::detached::DetachedEncoderSim::GetVelocityConversionFactor() const {
    return m_velocityConversionFactor.Get();
}

void rev::detached::DetachedEncoderSim::iterate(double velocity, double dt) {
    const double velocityRPM{velocity / GetVelocityConversionFactor()};
    const double velocityRPS = velocityRPM / 60.0;
    const double turns = (velocityRPS * dt) * GetPositionConversionFactor();

    m_position.Set(m_position.Get() + turns);

    const double angle{std::fmod(m_position.Get(), 1.0)};
    m_angle.Set(angle);
    m_rawAngle.Set(angle);
}
