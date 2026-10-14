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

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "gtest/gtest.h"
#include "rev/CANSparkDriver.h"
#include "rev/REVLibError.h"
#include "rev/SparkMax.h"

// TODO(jan): Fix tests
#if 0

TEST(ParameterTest, GetSetParameters) {
    rev::SparkMax m_spark{1, rev::SparkMax::MotorType::kBrushless};
    m_spark.SetIdleMode(rev::SparkMax::IdleMode::kBrake);

    ASSERT_EQ(m_spark.GetIdleMode(), rev::SparkMax::IdleMode::kBrake);
    m_spark.SetIdleMode(rev::SparkMax::IdleMode::kCoast);

    ASSERT_EQ(m_spark.GetIdleMode(), rev::SparkMax::IdleMode::kCoast);
}

TEST(ParameterTest, GetSetParameters_Multi_Device) {
    rev::SparkMax m_spark{1, rev::SparkMax::MotorType::kBrushless};
    rev::SparkMax m_spark2{2, rev::SparkMax::MotorType::kBrushless};

    m_spark.SetIdleMode(rev::SparkMax::IdleMode::kBrake);
    m_spark2.SetIdleMode(rev::SparkMax::IdleMode::kCoast);

    ASSERT_EQ(m_spark.GetIdleMode(), rev::SparkMax::IdleMode::kBrake);
    ASSERT_EQ(m_spark2.GetIdleMode(), rev::SparkMax::IdleMode::kCoast);

    m_spark.SetIdleMode(rev::SparkMax::IdleMode::kCoast);
    m_spark2.SetIdleMode(rev::SparkMax::IdleMode::kBrake);

    ASSERT_EQ(m_spark.GetIdleMode(), rev::SparkMax::IdleMode::kCoast);
    ASSERT_EQ(m_spark2.GetIdleMode(), rev::SparkMax::IdleMode::kBrake);
}

TEST(ParameterTest, FollowingAndRestore) {
    rev::SparkMax m_spark{1, rev::SparkMax::MotorType::kBrushless};
    rev::SparkMax m_spark2{2, rev::SparkMax::MotorType::kBrushless};
    m_spark.Follow(m_spark2);
    m_spark.SetIdleMode(rev::SparkMax::IdleMode::kBrake);

    ASSERT_EQ(m_spark.IsFollower(), true);
    ASSERT_EQ(m_spark2.IsFollower(), false);

    m_spark.RestoreFactoryDefaults();

    // Follow mode *is* reset, but CAN ID & Idle Mode is not
    ASSERT_EQ(m_spark.GetIdleMode(), rev::SparkMax::IdleMode::kBrake);
    ASSERT_EQ(m_spark.GetDeviceId(), 1);
    ASSERT_EQ(m_spark.IsFollower(), false);
}

#endif
