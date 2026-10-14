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

#include "rev/config/ClosedLoopConfigAccessor.h"

#include <stdint.h>

#include "rev/CANSparkDriver.h"

using namespace rev::spark;

namespace {

static constexpr int kSlotOffset{8};
static constexpr int kAdditionalSlotOffset{4};

}  // namespace

ClosedLoopConfigAccessor::ClosedLoopConfigAccessor(void* sparkHandle)
    : m_sparkHandle(sparkHandle),
      maxMotion{sparkHandle},
      feedForward{sparkHandle} {}

double ClosedLoopConfigAccessor::GetP(ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(
        static_cast<c_Spark_handle>(m_sparkHandle),
        static_cast<c_Spark_ConfigParameter>(c_Spark_kP_0 + slot * kSlotOffset),
        &value);
    return value;
}

double ClosedLoopConfigAccessor::GetI(ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(
        static_cast<c_Spark_handle>(m_sparkHandle),
        static_cast<c_Spark_ConfigParameter>(c_Spark_kI_0 + slot * kSlotOffset),
        &value);
    return value;
}

double ClosedLoopConfigAccessor::GetD(ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(
        static_cast<c_Spark_handle>(m_sparkHandle),
        static_cast<c_Spark_ConfigParameter>(c_Spark_kD_0 + slot * kSlotOffset),
        &value);
    return value;
}

double ClosedLoopConfigAccessor::GetDFilter(ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                static_cast<c_Spark_ConfigParameter>(
                                    c_Spark_kDFilter_0 + slot * kSlotOffset),
                                &value);
    return value;
}

double ClosedLoopConfigAccessor::GetIZone(ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                static_cast<c_Spark_ConfigParameter>(
                                    c_Spark_kIZone_0 + slot * kSlotOffset),
                                &value);
    return value;
}

double ClosedLoopConfigAccessor::GetMinOutput(ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                static_cast<c_Spark_ConfigParameter>(
                                    c_Spark_kOutputMin_0 + slot * kSlotOffset),
                                &value);
    return value;
}

double ClosedLoopConfigAccessor::GetMaxOutput(ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                static_cast<c_Spark_ConfigParameter>(
                                    c_Spark_kOutputMax_0 + slot * kSlotOffset),
                                &value);
    return value;
}

double ClosedLoopConfigAccessor::GetMaxIAccumulation(ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(
        static_cast<c_Spark_handle>(m_sparkHandle),
        static_cast<c_Spark_ConfigParameter>(c_Spark_kIMaxAccum_0 +
                                             slot * kAdditionalSlotOffset),
        &value);
    return value;
}

double ClosedLoopConfigAccessor::GetAllowedClosedLoopError(
    ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(
        static_cast<c_Spark_handle>(m_sparkHandle),
        static_cast<c_Spark_ConfigParameter>(c_Spark_kAllowedClosedLoopError_0 +
                                             slot * kAdditionalSlotOffset),
        &value);
    return value;
}

bool ClosedLoopConfigAccessor::GetPositionWrappingEnabled() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kPositionPIDWrapEnable, &value);
    return value != 0;
}

FeedbackSensor ClosedLoopConfigAccessor::GetFeedbackSensor() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kClosedLoopControlSensor, &value);
    return static_cast<FeedbackSensor>(value);
}
