/*
 * Copyright (c) 2024-2025 REV Robotics
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

#include "rev/config/SparkBaseConfigAccessor.h"

#include <stdint.h>

#include "rev/CANSparkDriver.h"

using namespace rev::spark;

SparkBaseConfigAccessor::SparkBaseConfigAccessor(void* sparkHandle)
    : m_sparkHandle(sparkHandle),
      absoluteEncoder{sparkHandle},
      analogSensor{sparkHandle},
      closedLoop{sparkHandle},
      encoder{sparkHandle},
      limitSwitch{sparkHandle},
      signals{sparkHandle},
      softLimit{sparkHandle} {}

SparkBaseConfig::IdleMode SparkBaseConfigAccessor::GetIdleMode() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kIdleMode, &value);
    return static_cast<SparkBaseConfig::IdleMode>(value);
}

bool SparkBaseConfigAccessor::GetInverted() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kInverted, &value);
    return value != 0;
}

int SparkBaseConfigAccessor::GetSmartCurrentLimit() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kSmartCurrentStallLimit, &value);
    return value;
}

int SparkBaseConfigAccessor::GetSmartCurrentFreeLimit() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kSmartCurrentFreeLimit, &value);
    return value;
}

int SparkBaseConfigAccessor::GetSmartCurrentRPMLimit() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kSmartCurrentConfig, &value);
    return value;
}

double SparkBaseConfigAccessor::GetSecondaryCurrentLimit() {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                c_Spark_kCurrentChop, &value);
    return value;
}

int SparkBaseConfigAccessor::GetSecondaryCurrentLimitChopCycles() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kCurrentChopCycles, &value);
    return value;
}

double SparkBaseConfigAccessor::GetAdvanceCommutation() {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                c_Spark_kCommutationAdvance, &value);
    return value;
}

double SparkBaseConfigAccessor::GetOpenLoopRampRate() {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                c_Spark_kOpenLoopRampRate, &value);
    return value == 0.0 ? value : 1.0 / value;
}

double SparkBaseConfigAccessor::GetClosedLoopRampRate() {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                c_Spark_kClosedLoopRampRate, &value);
    return value == 0.0 ? value : 1.0 / value;
}

double SparkBaseConfigAccessor::GetVoltageCompensation() {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                c_Spark_kCompensatedNominalVoltage, &value);
    return value;
}

bool SparkBaseConfigAccessor::GetVoltageCompensationEnabled() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kVoltageCompensationMode, &value);
    return value != 0;
}

int SparkBaseConfigAccessor::GetFollowerModeLeaderId() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kFollowerModeLeaderId, &value);
    return value;
}

bool SparkBaseConfigAccessor::GetFollowerModeInverted() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kFollowerModeIsInverted, &value);
    return value != 0;
}
