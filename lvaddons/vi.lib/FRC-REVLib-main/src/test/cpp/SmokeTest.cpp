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

#include "gtest/gtest.h"
#include "rev/SparkBase.h"
#include "rev/SparkFlex.h"
#include "rev/SparkMax.h"
#include "rev/SplineEncoder.h"
#include "rev/config/SparkFlexConfig.h"
#include "rev/config/SparkMaxConfig.h"

using namespace rev::spark;
using namespace rev::detached;

TEST(SmokeTest, ConstructSparkFlex) {
    SparkFlex sparkFlex{0, 42, SparkFlex::MotorType::kBrushless};
    SparkFlex sparkFlex2{1, 43, SparkFlex::MotorType::kBrushless};

    auto& absEnc = sparkFlex.GetAbsoluteEncoder();
    auto& absEnc2 = sparkFlex.GetAbsoluteEncoder();
    EXPECT_EQ(&absEnc, &absEnc2);

    auto& absEnc3 = sparkFlex.GetAbsoluteEncoder();
    auto& absEnc4 = sparkFlex2.GetAbsoluteEncoder();
    EXPECT_NE(&absEnc3, &absEnc4);

    auto& analog = sparkFlex.GetAnalog();
    auto& analog2 = sparkFlex.GetAnalog();
    EXPECT_EQ(&analog, &analog2);

    auto& analog3 = sparkFlex.GetAnalog();
    auto& analog4 = sparkFlex2.GetAnalog();
    EXPECT_NE(&analog3, &analog4);

    auto& clc = sparkFlex.GetClosedLoopController();
    auto& clc2 = sparkFlex.GetClosedLoopController();
    EXPECT_EQ(&clc, &clc2);

    auto& clc3 = sparkFlex.GetClosedLoopController();
    auto& clc4 = sparkFlex2.GetClosedLoopController();
    EXPECT_NE(&clc3, &clc4);

    auto& enc = sparkFlex.GetEncoder();
    auto& enc2 = sparkFlex.GetEncoder();
    EXPECT_EQ(&enc, &enc2);

    auto& enc3 = sparkFlex.GetEncoder();
    auto& enc4 = sparkFlex2.GetEncoder();
    EXPECT_NE(&enc3, &enc4);

    auto& extEnc = sparkFlex.GetExternalEncoder();
    auto& extEnc2 = sparkFlex.GetExternalEncoder();
    EXPECT_EQ(&extEnc, &extEnc2);

    auto& extEnc3 = sparkFlex.GetExternalEncoder();
    auto& extEnc4 = sparkFlex2.GetExternalEncoder();
    EXPECT_NE(&extEnc3, &extEnc4);

    auto& fwdLimSw = sparkFlex.GetForwardLimitSwitch();
    auto& fwdLimSw2 = sparkFlex.GetForwardLimitSwitch();
    EXPECT_EQ(&fwdLimSw, &fwdLimSw2);

    auto& fwdLimSw3 = sparkFlex.GetForwardLimitSwitch();
    auto& fwdLimSw4 = sparkFlex2.GetForwardLimitSwitch();
    EXPECT_NE(&fwdLimSw3, &fwdLimSw4);

    auto& revLimSw = sparkFlex.GetReverseLimitSwitch();
    auto& revLimSw2 = sparkFlex.GetReverseLimitSwitch();
    EXPECT_EQ(&revLimSw, &revLimSw2);

    auto& revLimSw3 = sparkFlex.GetReverseLimitSwitch();
    auto& revLimSw4 = sparkFlex2.GetReverseLimitSwitch();
    EXPECT_NE(&revLimSw3, &revLimSw4);
}

TEST(SmokeTest, ConstructSparkMax) {
    SparkMax sparkMax{0, 43, SparkMax::MotorType::kBrushed};
    SparkMax sparkMax2{1, 45, SparkMax::MotorType::kBrushed};

    SparkMaxConfig maxConfig;
    maxConfig.alternateEncoder.SetSparkMaxDataPortConfig();
    sparkMax.Configure(maxConfig, rev::ResetMode::kResetSafeParameters,
                       rev::PersistMode::kPersistParameters);
    sparkMax2.Configure(maxConfig, rev::ResetMode::kResetSafeParameters,
                        rev::PersistMode::kPersistParameters);

    auto& altEnc = sparkMax.GetAlternateEncoder();
    auto& altEnc2 = sparkMax.GetAlternateEncoder();
    EXPECT_EQ(&altEnc, &altEnc2);

    auto& altEnc3 = sparkMax.GetAlternateEncoder();
    auto& altEnc4 = sparkMax2.GetAlternateEncoder();
    EXPECT_NE(&altEnc3, &altEnc4);

    auto& analog = sparkMax.GetAnalog();
    auto& analog2 = sparkMax.GetAnalog();
    EXPECT_EQ(&analog, &analog2);

    auto& analog3 = sparkMax.GetAnalog();
    auto& analog4 = sparkMax2.GetAnalog();
    EXPECT_NE(&analog3, &analog4);

    auto& clc = sparkMax.GetClosedLoopController();
    auto& clc2 = sparkMax.GetClosedLoopController();
    EXPECT_EQ(&clc, &clc2);

    auto& clc3 = sparkMax.GetClosedLoopController();
    auto& clc4 = sparkMax2.GetClosedLoopController();
    EXPECT_NE(&clc3, &clc4);

    auto& enc = sparkMax.GetEncoder();
    auto& enc2 = sparkMax.GetEncoder();
    EXPECT_EQ(&enc, &enc2);

    auto& enc3 = sparkMax.GetEncoder();
    auto& enc4 = sparkMax2.GetEncoder();
    EXPECT_NE(&enc3, &enc4);
}

TEST(SmokeTest, CanSetMultipleStatus0Periods) {
    SparkFlexConfig config;
    config.signals.BusVoltagePeriodMs(2)
        .OutputCurrentPeriodMs(2)
        .AppliedOutputPeriodMs(2)
        .MotorTemperaturePeriodMs(2)
        .LimitsPeriodMs(2);

    SparkFlex m_flex{0, 42, SparkFlex::MotorType::kBrushless};
    m_flex.Configure(config, rev::ResetMode::kResetSafeParameters,
                     rev::PersistMode::kPersistParameters);
}

TEST(SmokeTest, CanSetMultipleStatus2Periods) {
    SparkFlexConfig config;
    config.signals.PrimaryEncoderVelocityPeriodMs(2)
        .PrimaryEncoderPositionPeriodMs(2);

    SparkFlex m_flex{0, 42, SparkFlex::MotorType::kBrushless};
    m_flex.Configure(config, rev::ResetMode::kResetSafeParameters,
                     rev::PersistMode::kPersistParameters);
}

TEST(SmokeTest, CanSetMultipleStatus3Periods) {
    SparkFlexConfig config;
    config.signals.AnalogPositionPeriodMs(2)
        .AnalogVelocityPeriodMs(2)
        .AnalogVoltagePeriodMs(2);

    SparkFlex m_flex{0, 42, SparkFlex::MotorType::kBrushless};
    m_flex.Configure(config, rev::ResetMode::kResetSafeParameters,
                     rev::PersistMode::kPersistParameters);
}

TEST(SmokeTest, CanSetMultipleStatus4Periods) {
    SparkFlexConfig config;
    config.signals.ExternalOrAltEncoderVelocity(2).ExternalOrAltEncoderPosition(
        2);

    SparkFlex m_flex{0, 42, SparkFlex::MotorType::kBrushless};
    m_flex.Configure(config, rev::ResetMode::kResetSafeParameters,
                     rev::PersistMode::kPersistParameters);
}

TEST(SmokeTest, CanSetMultipleStatus5Periods) {
    SparkFlexConfig config;
    config.signals.AbsoluteEncoderVelocityPeriodMs(2)
        .AbsoluteEncoderPositionPeriodMs(2);

    SparkFlex m_flex{0, 42, SparkFlex::MotorType::kBrushless};
    m_flex.Configure(config, rev::ResetMode::kResetSafeParameters,
                     rev::PersistMode::kPersistParameters);
}

TEST(DetachedSmokeTest, ConstructSplineEncoder) {
    DetachedEncoderConfig config;
    config.Inverted(true)
        .PositionConversionFactor(1.23)
        .signals.EncoderPositionPeriodMs(20);

    SplineEncoder spline{0, 13};
    spline.Configure(config, rev::ResetMode::kResetSafeParameters);
}

TEST(DetachedSmokeTest, CanSetStatus2Period) {
    DetachedEncoderConfig config;
    config.signals.EncoderAnglePeriodMs(2);

    SplineEncoder m_spline{1, 13};
    m_spline.Configure(config, rev::ResetMode::kResetSafeParameters);
}

TEST(DetachedSmokeTest, CanSetStatus3Period) {
    DetachedEncoderConfig config;
    config.signals.EncoderPositionPeriodMs(2);

    SplineEncoder m_spline{2, 13};
    m_spline.Configure(config, rev::ResetMode::kResetSafeParameters);
}

TEST(DetachedSmokeTest, CanSetStatus4Period) {
    DetachedEncoderConfig config;
    config.signals.EncoderVelocityPeriodMs(2);

    SplineEncoder m_spline{3, 13};
    m_spline.Configure(config, rev::ResetMode::kResetSafeParameters);
}
