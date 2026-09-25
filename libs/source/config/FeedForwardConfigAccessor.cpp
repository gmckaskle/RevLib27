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

#include "rev/config/FeedForwardConfigAccessor.h"

#include <stdint.h>

#include "rev/CANSparkDriver.h"

using namespace rev::spark;

FeedForwardConfigAccessor::FeedForwardConfigAccessor(void* sparkHandle)
    : m_sparkHandle(sparkHandle) {}

double FeedForwardConfigAccessor::getkS(ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(
        static_cast<c_Spark_handle>(m_sparkHandle),
        static_cast<c_Spark_ConfigParameter>(c_Spark_kS_0 + slot * 5), &value);
    return value;
}

double FeedForwardConfigAccessor::getkV(ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(
        static_cast<c_Spark_handle>(m_sparkHandle),
        static_cast<c_Spark_ConfigParameter>(c_Spark_kV_0 + slot * 8), &value);
    return value;
}

double FeedForwardConfigAccessor::getkA(ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(
        static_cast<c_Spark_handle>(m_sparkHandle),
        static_cast<c_Spark_ConfigParameter>(c_Spark_kA_0 + slot * 5), &value);
    return value;
}

double FeedForwardConfigAccessor::getkG(ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(
        static_cast<c_Spark_handle>(m_sparkHandle),
        static_cast<c_Spark_ConfigParameter>(c_Spark_kG_0 + slot * 5), &value);
    return value;
}

double FeedForwardConfigAccessor::getkCos(ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(
        static_cast<c_Spark_handle>(m_sparkHandle),
        static_cast<c_Spark_ConfigParameter>(c_Spark_kCos_0 + slot * 5),
        &value);
    return value;
}

double FeedForwardConfigAccessor::getkCosRatio(ClosedLoopSlot slot) {
    float value;
    c_Spark_GetParameterFloat32(
        static_cast<c_Spark_handle>(m_sparkHandle),
        static_cast<c_Spark_ConfigParameter>(c_Spark_kCosRatio_0 + slot * 5),
        &value);
    return value;
}
