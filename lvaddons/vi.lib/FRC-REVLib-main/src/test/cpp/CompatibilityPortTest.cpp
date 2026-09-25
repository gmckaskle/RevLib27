/*
 * Copyright (c) 2018-2024 REV Robotics
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

// TODO(jan): Fix tests
#if 0

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "gtest/gtest.h"

// we're all friends here. This gives us access to the spark handle.
#define protected public
#include "rev/SparkMax.h"
#include "rev/CANSparkDriver.h"
#include "rev/CANSparkDriverPrivate.h"
#undef protected

/**
 *
 */
TEST(CompatibilityPortTest, CreateFLSThenAltEnc) {
    rev::SparkMax m_spark{1, rev::SparkMax::MotorType::kBrushless};
    ((c_Spark_handle)m_spark.m_sparkMaxHandle)->m_sparkModel = c_Spark_SparkMax;
    rev::SparkLimitSwitch fls = m_spark.GetForwardLimitSwitch(
        rev::SparkLimitSwitch::Type::kNormallyOpen);
    fls.Get();

    EXPECT_THROW(m_spark.GetAlternateEncoder(
                     rev::SparkMaxAlternateEncoder::Type::kQuadrature, 4096),
                 std::runtime_error);
}

TEST(CompatibilityPortTest, CreateRLSThenAltEnc) {
    rev::SparkMax m_spark{1, rev::SparkMax::MotorType::kBrushless};
    ((c_Spark_handle)m_spark.m_sparkMaxHandle)->m_sparkModel = c_Spark_SparkMax;
    rev::SparkLimitSwitch rls = m_spark.GetReverseLimitSwitch(
        rev::SparkLimitSwitch::Type::kNormallyOpen);
    rls.Get();

    EXPECT_THROW(m_spark.GetAlternateEncoder(
                     rev::SparkMaxAlternateEncoder::Type::kQuadrature, 4096),
                 std::runtime_error);
}

TEST(CompatibilityPortTest, CreateAltEncThenFLS) {
    rev::SparkMax m_spark{1, rev::SparkMax::MotorType::kBrushless};
    ((c_Spark_handle)m_spark.m_sparkMaxHandle)->m_sparkModel = c_Spark_SparkMax;

    rev::SparkMaxAlternateEncoder altEnc = m_spark.GetAlternateEncoder(
        rev::SparkMaxAlternateEncoder::Type::kQuadrature, 4096);
    altEnc.GetInverted();

    EXPECT_THROW(m_spark.GetForwardLimitSwitch(
                     rev::SparkLimitSwitch::Type::kNormallyOpen),
                 std::runtime_error);
}

TEST(CompatibilityPortTest, CreateAltEncThenRLS) {
    rev::SparkMax m_spark{1, rev::SparkMax::MotorType::kBrushless};
    ((c_Spark_handle)m_spark.m_sparkMaxHandle)->m_sparkModel = c_Spark_SparkMax;

    rev::SparkMaxAlternateEncoder altEnc = m_spark.GetAlternateEncoder(
        rev::SparkMaxAlternateEncoder::Type::kQuadrature, 4096);
    altEnc.GetInverted();

    EXPECT_THROW(m_spark.GetReverseLimitSwitch(
                     rev::SparkLimitSwitch::Type::kNormallyOpen),
                 std::runtime_error);
}

#endif
