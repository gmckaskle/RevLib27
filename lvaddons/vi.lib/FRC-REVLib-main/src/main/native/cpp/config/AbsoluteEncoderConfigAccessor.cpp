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

#include "rev/config/AbsoluteEncoderConfigAccessor.h"

#include <stdint.h>

#include "rev/CANSparkDriver.h"

using namespace rev::spark;

AbsoluteEncoderConfigAccessor::AbsoluteEncoderConfigAccessor(void* sparkHandle)
    : m_sparkHandle(sparkHandle) {}

bool AbsoluteEncoderConfigAccessor::GetInverted() const {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kDutyCycleInverted, &value);
    return value != 0;
}

double AbsoluteEncoderConfigAccessor::GetPositionConversionFactor() const {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                c_Spark_kDutyCyclePositionFactor, &value);
    return value;
}

double AbsoluteEncoderConfigAccessor::GetVelocityConversionFactor() const {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                c_Spark_kDutyCycleVelocityFactor, &value);
    return value;
}

double AbsoluteEncoderConfigAccessor::GetZeroOffset() const {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                c_Spark_kDutyCycleOffset, &value);
    return value;
}

int AbsoluteEncoderConfigAccessor::GetAverageDepth() const {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kDutyCycleAverageDepth, &value);
    return 1 << value;
}

double AbsoluteEncoderConfigAccessor::GetStartPulseUs() const {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                c_Spark_kDutyCycleEncoderStartPulseUs, &value);
    return value;
}

double AbsoluteEncoderConfigAccessor::GetEndPulseUs() const {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                c_Spark_kDutyCycleEncoderEndPulseUs, &value);
    return value;
}

bool AbsoluteEncoderConfigAccessor::IsZeroCentered() const {
    return GetRangeOffset() == 0.0;
}

double AbsoluteEncoderConfigAccessor::GetRangeOffset() const {
    float value;
    c_Spark_GetParameterFloat32(static_cast<c_Spark_handle>(m_sparkHandle),
                                c_Spark_kDutyCycleRangeOffset, &value);
    return value;
}
